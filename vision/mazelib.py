#!/usr/bin/env python3
# =============================================================================
#  MTRN3100 Micromouse - shared vision library (off-board, laptop-side).
#
#  Everything the week-12 vision tools share: camera capture, corner selection
#  (manual click - staff-sanctioned on Ed #156 - plus automatic detection),
#  rectification with lattice-phase refinement, wall detection, cylinder
#  detection, the (cell, heading) Dijkstra solver, command emission/simulation
#  and overlay rendering.
#
#  Conventions (match assignment spec 1.2 / 3.4 and the Rust firmware):
#    rows: 0 = North = top of image after --rotate is applied
#    dirs: 0=N, 1=E, 2=S, 3=W  (bit i of a wall mask = wall on that side)
#    'f' = forward one cell (180 mm), 'l' = +90 CCW, 'r' = -90 CW
#
#  AI ASSISTANCE (assignment 5.1): written with the assistance of a generative
#  AI (Anthropic Claude), reviewed and tested on real lab-camera photos.
# =============================================================================
import json
import os
from collections import namedtuple
from heapq import heappush, heappop

import cv2
import numpy as np

N, E, S, W = 0, 1, 2, 3
DR = (-1, 0, 1, 0)          # row delta per direction
DC = (0, 1, 0, -1)          # col delta per direction
DIR_NAMES = "NESW"
OPP = (S, W, N, E)

CELL_MM = 180.0             # assignment spec 1.2
K = 100                     # rectified px per cell -> 1.8 mm/px

# Appearance constants measured on the real lab-camera photo (Ed #140):
# floor grey ~150-180, dark acrylic walls ~17-45, cyan clips on wall tops.
DARK_MAX_THR = 100          # ceiling for the adaptive dark threshold
CYAN_LO = np.array([78, 60, 50])
CYAN_HI = np.array([108, 255, 255])
WALL_MIN_RUN_PX = 2         # min transverse thickness; near-axis walls show
                            # only a ~3 px top edge (the darkness threshold
                            # already excludes the light-grey floor seam)
EDGE_SPAN = 0.55            # central fraction of an edge that is sampled


# ---------------------------------------------------------------------------
# Capture / IO
# ---------------------------------------------------------------------------

def capture_frame(cam_index, width=1920, height=1080, warmup=10):
    """Grab one frame from the overhead lab camera (Ed #131: laptop plugs into
    the demo-desk USB; two cameras exist, either must work)."""
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera {cam_index}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    for _ in range(warmup):                    # let auto-exposure settle
        cap.read()
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise SystemExit(f"camera {cam_index} returned no frame")
    return frame


# ---------------------------------------------------------------------------
# Corner selection
# ---------------------------------------------------------------------------

def _corner_cache_path(image_path):
    return os.path.splitext(image_path)[0] + ".corners.json"


def load_cached_corners(image_path, shape):
    """Cached corners are only valid for the SAME image content: keyed by
    shape + file mtime, so overwriting a photo (or a camera nudge producing a
    new capture) invalidates the cache instead of silently reusing it."""
    p = _corner_cache_path(image_path)
    if os.path.exists(p) and os.path.exists(image_path):
        d = json.load(open(p))
        if d.get("shape") == list(shape[:2]) \
                and d.get("mtime") == os.path.getmtime(image_path):
            return np.array(d["corners"], dtype=np.float32)
    return None


def save_corners(image_path, shape, corners):
    p = _corner_cache_path(image_path)
    mtime = os.path.getmtime(image_path) if os.path.exists(image_path) else None
    json.dump({"shape": list(shape[:2]), "mtime": mtime,
               "corners": np.asarray(corners).tolist()},
              open(p, "w"))


def write_image(path, img):
    """cv2.imwrite that fails loudly - a demo-day 'overlay saved' message must
    never lie about evidence that wasn't written."""
    if not cv2.imwrite(path, img):
        raise SystemExit(f"FAILED to write {path}")


def auto_corners(img):
    """Find the maze outline automatically: the floor is the largest bright
    region; fit lines to its four straight sides (skipping the chamfered
    corners) and intersect them. Refined later by lattice phase, so ~1/3 cell
    of error here is fine."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, bright = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(bright)
    if n < 2:
        return None
    big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    mask = (lab == big).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)
    cx, cy = c.mean(axis=0)
    w = c[:, 0].max() - c[:, 0].min()
    h = c[:, 1].max() - c[:, 1].min()
    sides = {
        "W": c[(c[:, 0] < cx - 0.38 * w) & (np.abs(c[:, 1] - cy) < 0.30 * h)],
        "E": c[(c[:, 0] > cx + 0.38 * w) & (np.abs(c[:, 1] - cy) < 0.30 * h)],
        "N": c[(c[:, 1] < cy - 0.38 * h) & (np.abs(c[:, 0] - cx) < 0.30 * w)],
        "S": c[(c[:, 1] > cy + 0.38 * h) & (np.abs(c[:, 0] - cx) < 0.30 * w)],
    }
    if any(len(v) < 20 for v in sides.values()):
        return None
    lines = {}
    for k, pts in sides.items():
        vx, vy, x0, y0 = cv2.fitLine(pts.astype(np.float32), cv2.DIST_HUBER,
                                     0, 0.01, 0.01).flatten()
        lines[k] = (float(vx), float(vy), float(x0), float(y0))

    def cross(l1, l2):
        vx1, vy1, x1, y1 = l1
        vx2, vy2, x2, y2 = l2
        A = np.array([[vx1, -vx2], [vy1, -vy2]])
        b = np.array([x2 - x1, y2 - y1])
        t = np.linalg.solve(A, b)
        return [x1 + t[0] * vx1, y1 + t[0] * vy1]

    return np.array([cross(lines["N"], lines["W"]), cross(lines["N"], lines["E"]),
                     cross(lines["S"], lines["E"]), cross(lines["S"], lines["W"])],
                    dtype=np.float32)


def click_corners(img, window="click corners: TL, TR, BR, BL  (u=undo, Enter=done)"):
    """Manual 4-corner picker (Ed #156: officially fine). Click the four outer
    corners of the maze square in TL, TR, BR, BL order."""
    disp_scale = min(1.0, 1400 / img.shape[1])
    disp = cv2.resize(img, None, fx=disp_scale, fy=disp_scale)
    pts = []

    def redraw():
        v = disp.copy()
        for i, p in enumerate(pts):
            q = tuple(np.int32(np.array(p) * disp_scale))
            cv2.circle(v, q, 6, (0, 0, 255), -1)
            cv2.putText(v, "TL TR BR BL".split()[i], (q[0] + 8, q[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow(window, v)

    def on_mouse(ev, x, y, flags, param):
        if ev == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((x / disp_scale, y / disp_scale))
            redraw()

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)
    redraw()
    while True:
        k = cv2.waitKey(50) & 0xFF
        if k in (ord("u"), ord("U")) and pts:
            pts.pop()
            redraw()
        elif k in (13, 10) and len(pts) == 4:
            break
        elif k in (27, ord("q")):
            cv2.destroyWindow(window)
            raise SystemExit("corner picking aborted")
    cv2.destroyWindow(window)
    return np.array(pts, dtype=np.float32)


# ---------------------------------------------------------------------------
# Rectification
# ---------------------------------------------------------------------------

def dark_cyan_masks(warp):
    """(dark, cyan) binary masks: dark acrylic walls and the cyan clips on
    wall tops. Dark threshold adapts by Otsu but is clamped so a dim image
    can't drag it up into floor greys."""
    gray = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
    otsu, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thr = min(float(otsu) * 0.75, DARK_MAX_THR)
    dark = (gray < thr).astype(np.uint8)
    hsv = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)
    cyan = (cv2.inRange(hsv, CYAN_LO, CYAN_HI) > 0).astype(np.uint8)
    return dark, cyan


def wall_mask_of(warp):
    """Binary mask of wall evidence: dark acrylic OR cyan clips. A 3x3 close
    consolidates the thin (2-4 px) anti-aliased top edge of near-axis walls."""
    dark, cyan = dark_cyan_masks(warp)
    return cv2.morphologyEx(dark | cyan, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


def _run_candidates(mask, side, pad, size, samples=11, min_run=6, max_gap=2):
    """Per sample position along one side, list every thick dark/cyan run's
    inner end within the search band - candidate wall-base points.

    The coarse quad may be the true maze corners (band right at pad) or the
    aluminium frame outline (wall up to ~1.5 cells inside, behind frame seams
    and shadows), so ALL candidates are collected; the caller disambiguates
    with global lattice consistency. `mask` must be a STRICT wall mask: walls
    are gray 17-45, frame seams/shadows 70-100, so with a min_run thickness
    gate the seam mostly drops out already. Runs' INNER ends are used - wall
    tops parallax-shift outward, only bases sit on the true lattice.
    Returns [(a, [pos, ...]), ...]."""
    out = []
    depth = int(0.30 * size)
    for t in np.linspace(0.32, 0.68, samples):   # clear of the chamfers
        a = pad + int(t * size)
        if side in ("top", "bottom"):
            line = mask[:, a - 2:a + 3].mean(axis=1) >= 0.5
        else:
            line = mask[a - 2:a + 3, :].mean(axis=0) >= 0.5
        if side in ("top", "left"):
            rng = range(max(0, pad - 40), pad + depth)
        else:
            rng = range(min(len(line) - 1, pad + size + 40),
                        pad + size - depth, -1)
        cands = []
        run_len, gap, run_end = 0, 0, None
        for i in rng:
            if line[i]:
                run_len += 1
                gap = 0
                run_end = i
            elif run_len:
                gap += 1
                if gap > max_gap:
                    if run_len >= min_run:
                        cands.append(float(run_end))
                    run_len, gap, run_end = 0, 0, None
        if run_len >= min_run:
            cands.append(float(run_end))
        out.append((a, cands))
    return out


def _side_lines(cands, axis, wall_half=4, sign=1, min_support=6):
    """Group per-sample candidate points into straight-line hypotheses.

    1-D RANSAC on position: for each candidate value, collect the nearest
    candidate of every sample within +/-8 px, fit a line if enough samples
    agree. Returns [(m, b, support)] sorted outermost first, with b shifted
    outward by wall_half so lines are wall CENTRES, not inner faces."""
    seeds = sorted({p for _, ps in cands for p in ps})
    lines = []
    for seed in seeds:
        pts = []
        for a, ps in cands:
            near = [p for p in ps if abs(p - seed) < 8]
            if near:
                pts.append((a, min(near, key=lambda p: abs(p - seed))))
        if len(pts) < min_support:
            continue
        arr = np.array(pts, dtype=np.float64)
        m, b = np.polyfit(arr[:, 0], arr[:, 1], 1)
        res = np.abs(arr[:, 1] - (m * arr[:, 0] + b))
        keep = res < 5.0
        if keep.sum() < min_support:
            continue
        m, b = np.polyfit(arr[keep, 0], arr[keep, 1], 1)
        if abs(m) > 0.12:
            continue
        if not any(abs(b - b2) < 6 and abs(m - m2) < 0.02 for m2, b2, _ in lines):
            lines.append((m, b + sign * wall_half, int(keep.sum())))
    if axis == "y":
        lines.sort(key=lambda l: l[1], reverse=(sign > 0))
    else:
        lines.sort(key=lambda l: l[1], reverse=(sign > 0))
    return lines


def rectify(img, corners, n=9, k=K, refine=True, debug=False):
    """Warp the maze to an n*k square.

    `corners` may be the true maze corners (manual clicks) or any quad that
    CONTAINS the maze (e.g. auto_corners' bright-region quad, which is really
    the aluminium frame outline, up to ~20% outside the walls and on a higher
    plane). With refine=True the boundary wall BASE lines are found inside the
    coarse warp, intersected into the true maze corners, and the ORIGINAL
    image is re-warped through them - one resample, correct projectivity."""
    size = n * k
    dst = np.array([[0, 0], [size, 0], [size, size], [0, size]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(np.asarray(corners, np.float32), dst)
    pad = k
    T = np.array([[1, 0, pad], [0, 1, pad], [0, 0, 1]], dtype=np.float64)
    coarse = cv2.warpPerspective(img, T @ H, (size + 2 * pad, size + 2 * pad))
    if not refine:
        return coarse[pad:pad + size, pad:pad + size], H

    # STRICT wall mask for boundary finding: near-black acrylic or cyan clips.
    # The adaptive dark threshold (~90) would also pass frame seams/shadows.
    gray = cv2.cvtColor(coarse, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(coarse, cv2.COLOR_BGR2HSV)
    cyan = (cv2.inRange(hsv, CYAN_LO, CYAN_HI) > 0).astype(np.uint8)
    mask = ((gray < 60).astype(np.uint8) | cyan)

    cand_lines = {
        "top": _side_lines(_run_candidates(mask, "top", pad, size), "y", sign=-1),
        "bottom": _side_lines(_run_candidates(mask, "bottom", pad, size), "y", sign=+1),
        "left": _side_lines(_run_candidates(mask, "left", pad, size), "x", sign=-1),
        "right": _side_lines(_run_candidates(mask, "right", pad, size), "x", sign=+1),
    }

    # Disambiguate frame seams / interior walls by GLOBAL lattice consistency:
    # the correct opposite-side pair implies a cell pitch, and lattice lines
    # at that pitch must land on interior wall mass.
    a0, a1 = pad + int(0.32 * size), pad + int(0.68 * size)
    proj_y = mask[:, a0:a1].mean(axis=1)
    proj_x = mask[a0:a1, :].mean(axis=0)
    centre = pad + size / 2

    def best_pair(near_lines, far_lines, proj):
        best, best_score = None, -1.0
        for mn_, bn, _sn in near_lines[:3]:
            for mf, bf, _sf in far_lines[:3]:
                near_c = mn_ * centre + bn
                far_c = mf * centre + bf
                pitch = (far_c - near_c) / n
                if not 0.55 * k < pitch < 1.15 * k:
                    continue
                score = 0.0
                for i in range(1, n):
                    p = int(round(near_c + i * pitch))
                    lo2, hi2 = max(0, p - 3), min(len(proj), p + 4)
                    if lo2 < hi2:
                        score += float(proj[lo2:hi2].max())
                if score > best_score:
                    best, best_score = ((mn_, bn), (mf, bf)), score
        return best

    pair_y = best_pair(cand_lines["top"], cand_lines["bottom"], proj_y)
    pair_x = best_pair(cand_lines["left"], cand_lines["right"], proj_x)
    if debug:
        import sys
        print(f"# side candidates: "
              + ", ".join(f"{s}:{[(round(b, 1), sup) for _m, b, sup in v]}"
                          for s, v in cand_lines.items()), file=sys.stderr)
        print(f"# chosen pairs: y={pair_y} x={pair_x}", file=sys.stderr)
    if pair_y is None or pair_x is None:
        return coarse[pad:pad + size, pad:pad + size], H
    lines = {"top": pair_y[0], "bottom": pair_y[1],
             "left": pair_x[0], "right": pair_x[1]}

    def cross(h_line, v_line):
        mh, bh = h_line            # y = mh*x + bh
        mv, bv = v_line            # x = mv*y + bv
        y = (mh * bv + bh) / (1 - mh * mv)
        x = mv * y + bv
        return [x, y]

    quad = np.array([cross(lines["top"], lines["left"]),
                     cross(lines["top"], lines["right"]),
                     cross(lines["bottom"], lines["right"]),
                     cross(lines["bottom"], lines["left"])], dtype=np.float32)
    # Sanity: refined quad must be a plausible maze square inside the coarse
    # quad; otherwise keep the coarse warp.
    side_px = [np.linalg.norm(quad[i] - quad[(i + 1) % 4]) for i in range(4)]
    if not all(0.55 * size < s < 1.10 * size for s in side_px):
        return coarse[pad:pad + size, pad:pad + size], H
    # Map refined corners back to original image coords, re-warp in one step.
    Hinv = np.linalg.inv(T @ H)
    orig = cv2.perspectiveTransform(quad.reshape(-1, 1, 2).astype(np.float64), Hinv)
    H2 = cv2.getPerspectiveTransform(orig.reshape(4, 2).astype(np.float32), dst)
    out = cv2.warpPerspective(img, H2, (size, size))
    return out, H2


# ---------------------------------------------------------------------------
# Maze grid + wall detection
# ---------------------------------------------------------------------------

class Grid:
    """Wall map for an n x n maze. walls[r][c] is a 4-bit NESW mask."""

    def __init__(self, n=9, chamfer=1):
        self.n = n
        self.walls = np.zeros((n, n), dtype=np.uint8)
        self.blocked = np.zeros((n, n), dtype=bool)
        for i in range(n):
            self.add_wall(0, i, N)
            self.add_wall(n - 1, i, S)
            self.add_wall(i, 0, W)
            self.add_wall(i, n - 1, E)
        # Chamfered corners (real arena, Ed #140 photo): 45-degree cuts spanning
        # `chamfer` cells; the cells in each corner triangle are unusable.
        for r in range(n):
            for c in range(n):
                if (r + c < chamfer or (r + (n - 1 - c)) < chamfer
                        or ((n - 1 - r) + c) < chamfer
                        or ((n - 1 - r) + (n - 1 - c)) < chamfer):
                    self.block(r, c)

    def in_bounds(self, r, c):
        return 0 <= r < self.n and 0 <= c < self.n

    def add_wall(self, r, c, d):
        self.walls[r, c] |= 1 << d
        r2, c2 = r + DR[d], c + DC[d]
        if self.in_bounds(r2, c2):
            self.walls[r2, c2] |= 1 << OPP[d]

    def remove_wall(self, r, c, d):
        self.walls[r, c] &= ~(1 << d) & 0xF
        r2, c2 = r + DR[d], c + DC[d]
        if self.in_bounds(r2, c2):
            self.walls[r2, c2] &= ~(1 << OPP[d]) & 0xF

    def has_wall(self, r, c, d):
        return bool(self.walls[r, c] >> d & 1)

    def block(self, r, c):
        self.blocked[r, c] = True
        for d in range(4):
            self.add_wall(r, c, d)

    def open_neighbours(self, r, c):
        for d in range(4):
            if not self.has_wall(r, c, d):
                r2, c2 = r + DR[d], c + DC[d]
                if self.in_bounds(r2, c2) and not self.blocked[r2, c2]:
                    yield d, r2, c2

    def interior_edges(self):
        """Yield (r, c, d) for EVERY unique interior cell boundary (S and E
        only so each shared edge appears once), including edges of blocked
        cells - callers that care filter on self.blocked themselves."""
        for r in range(self.n):
            for c in range(self.n):
                if c + 1 < self.n:
                    yield r, c, E
                if r + 1 < self.n:
                    yield r, c, S


EdgeScore = namedtuple("EdgeScore", "r c d score")


def detect_walls(warp, n=9, k=K, chamfer=1, lean_gain=0.055):
    """Detect walls on a rectified maze image.

    The homography maps the FLOOR plane, but the walls are 150 mm tall: a
    wall's dark face projects OUTWARD (away from the camera axis) from its
    lattice line, by roughly lean_gain * distance-from-image-centre (~30 px at
    the far cells). So each edge is sampled with an asymmetric strip: a few px
    on the inner side of the line, and a parallax-scaled reach on the outer
    side.

    A position along the edge counts as wall if the darkest transverse pixel
    is well below the local floor brightness (thin near-axis walls are only a
    1-2 px grey mix, so a binary mask + thickness gate misses them; the
    floor-plate seam, Ed #118, is light grey and fails the darkness test), or
    if a cyan clip crosses the strip. Score = covered fraction of the central
    EDGE_SPAN. Returns (Grid, [EdgeScore]) with scores for the review UI.
    """
    gray = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY).astype(np.float32)
    hsv = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)
    cyan = (cv2.inRange(hsv, CYAN_LO, CYAN_HI) > 0)
    grid = Grid(n, chamfer)
    scores = []
    lo = int(k * (1 - EDGE_SPAN) / 2)
    hi = int(k * (1 + EDGE_SPAN) / 2)
    centre = n * k / 2
    inner = 6
    for r, c, d in grid.interior_edges():
        if d == S:                      # horizontal edge, y = (r+1)*k
            y = (r + 1) * k
            reach = int(8 + lean_gain * abs(y - centre))
            y0 = max(0, y - (inner if y >= centre else reach))
            y1 = y + (reach if y >= centre else inner)
            g = gray[y0:y1, c * k + lo:c * k + hi]
            cy = cyan[y0:y1, c * k + lo:c * k + hi]
            mn, mx = g.min(axis=0), g.max(axis=0)
            med = np.median(g, axis=0)
            has_clip = cy.any(axis=0)
        else:                           # vertical edge, x = (c+1)*k
            x = (c + 1) * k
            reach = int(8 + lean_gain * abs(x - centre))
            x0 = max(0, x - (inner if x >= centre else reach))
            x1 = x + (reach if x >= centre else inner)
            g = gray[r * k + lo:r * k + hi, x0:x1]
            cy = cyan[r * k + lo:r * k + hi, x0:x1]
            mn, mx = g.min(axis=1), g.max(axis=1)
            med = np.median(g, axis=1)
            has_clip = cy.any(axis=1)
        # A position is wall if its darkest transverse pixel is (a) absolutely
        # dark AND well below the brightest (floor) pixel in the slice, or
        # (b) a localized dip below the slice's own median - catches thin
        # walls washed out by reflections, while broad soft shadows (uniform,
        # so min ~= median) stay rejected - or (c) crossed by a cyan clip.
        covered = ((mn < np.minimum(DARK_MAX_THR * 0.9, 0.62 * mx))
                   | (mn < 0.85 * med) | has_clip)
        score = float(covered.mean()) if len(covered) else 0.0
        scores.append(EdgeScore(r, c, d, score))
        if score >= 0.5:
            grid.add_wall(r, c, d)
    return grid, scores


# ---------------------------------------------------------------------------
# Solver: Dijkstra over (row, col, heading)
# ---------------------------------------------------------------------------

def solve(grid, start, goal, turn_cost=1.0):
    """start = (r, c, dir), goal = (r, c). Minimises actions: 1 per forward,
    turn_cost per 90-degree pivot (Ed #144: fewest actions is an accepted
    definition of shortest path - it is also what is fastest to execute).
    Returns (commands, cell_path) or (None, None)."""
    if turn_cost < 0:
        raise ValueError("turn_cost must be >= 0 (negative cost cycles forever)")
    sr, sc, sd = start
    gr, gc = goal
    for name, (r, c) in (("start", (sr, sc)), ("goal", (gr, gc))):
        if not grid.in_bounds(r, c):
            raise ValueError(f"{name} cell {(r, c)} is outside the {grid.n}x{grid.n} maze")
        if grid.blocked[r, c]:
            raise ValueError(
                f"{name} cell {(r, c)} is a blocked (chamfered corner) cell; "
                f"if the maze has no chamfers, run with --chamfer 0")
    dist = {}
    prev = {}
    q = [(0.0, sr, sc, sd)]
    dist[(sr, sc, sd)] = 0.0
    while q:
        cost, r, c, d = heappop(q)
        if dist.get((r, c, d), np.inf) < cost:
            continue
        if (r, c) == (gr, gc):
            # reconstruct
            cmds = []
            cells = [(r, c)]
            cur = (r, c, d)
            while cur in prev:
                pst, mv = prev[cur]
                cmds.append(mv)
                if mv == "f":
                    cells.append(pst[:2])
                cur = pst
            cmds.reverse()
            cells.reverse()
            return "".join(cmds), cells
        for mv, nd in (("l", (d + 3) % 4), ("r", (d + 1) % 4)):
            nc = cost + turn_cost
            if nc < dist.get((r, c, nd), np.inf):
                dist[(r, c, nd)] = nc
                prev[(r, c, nd)] = ((r, c, d), mv)
                heappush(q, (nc, r, c, nd))
        if not grid.has_wall(r, c, d):
            r2, c2 = r + DR[d], c + DC[d]
            if grid.in_bounds(r2, c2) and not grid.blocked[r2, c2]:
                ncst = cost + 1.0
                if ncst < dist.get((r2, c2, d), np.inf):
                    dist[(r2, c2, d)] = ncst
                    prev[(r2, c2, d)] = ((r, c, d), "f")
                    heappush(q, (ncst, r2, c2, d))
    return None, None


def simulate(grid, start, commands):
    """Replay a command string on a wall map. Returns final (r, c, d) or raises
    if a move crosses a wall / leaves the maze - the pipeline's self-check."""
    r, c, d = start
    for i, mv in enumerate(commands):
        if mv == "l":
            d = (d + 3) % 4
        elif mv == "r":
            d = (d + 1) % 4
        elif mv == "f":
            if grid.has_wall(r, c, d):
                raise ValueError(f"command {i} ('f') crosses a wall at {(r, c)}")
            r, c = r + DR[d], c + DC[d]
            if not grid.in_bounds(r, c) or grid.blocked[r, c]:
                raise ValueError(f"command {i} ('f') leaves the maze at {(r, c)}")
        else:
            raise ValueError(f"bad command {mv!r}")
    return r, c, d


def parse_start(s):
    r, c, d = s.split(",")
    return int(r), int(c), DIR_NAMES.index(d.strip().upper())


def parse_cell(s):
    r, c = s.split(",")
    return int(r), int(c)


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def render_overlay(warp, grid, scores=None, path=None, start=None, goal=None,
                   k=K):
    """Detected walls + path over the rectified photo: the demonstrator
    evidence that the solution is image-derived, not hard-coded."""
    vis = warp.copy()
    n = grid.n
    for r in range(n):
        for c in range(n):
            if grid.blocked[r, c]:
                cv2.rectangle(vis, (c * k + 2, r * k + 2),
                              ((c + 1) * k - 2, (r + 1) * k - 2), (60, 60, 200), 1)
    for r, c, d in grid.interior_edges():
        present = grid.has_wall(r, c, d)
        if d == S:
            p1, p2 = (c * k, (r + 1) * k), ((c + 1) * k, (r + 1) * k)
        else:
            p1, p2 = ((c + 1) * k, r * k), ((c + 1) * k, (r + 1) * k)
        if present:
            cv2.line(vis, p1, p2, (0, 0, 255), 3)
    cv2.rectangle(vis, (0, 0), (n * k - 1, n * k - 1), (0, 0, 255), 3)
    if scores:
        for e in scores:
            if 0.25 <= e.score < 0.75:      # ambiguous - flag for the eye
                if e.d == S:
                    p = (e.c * k + k // 2, (e.r + 1) * k)
                else:
                    p = ((e.c + 1) * k, e.r * k + k // 2)
                cv2.circle(vis, p, 10, (0, 255, 255), 2)
                cv2.putText(vis, f"{e.score:.2f}", (p[0] - 18, p[1] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    if path:
        pts = [(c * k + k // 2, r * k + k // 2) for r, c in path]
        for a, b in zip(pts, pts[1:]):
            cv2.line(vis, a, b, (0, 200, 0), 4)
    if start is not None:
        sr, sc = start[0], start[1]
        cv2.circle(vis, (sc * k + k // 2, sr * k + k // 2), 14, (255, 120, 0), -1)
        cv2.putText(vis, "S", (sc * k + k // 2 - 8, sr * k + k // 2 + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    if goal is not None:
        gr, gc = goal
        cv2.circle(vis, (gc * k + k // 2, gr * k + k // 2), 14, (0, 0, 255), -1)
        cv2.putText(vis, "G", (gc * k + k // 2 - 8, gr * k + k // 2 + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return vis


def review_walls(warp, grid, scores, k=K,
                 window="wall review: click an edge to toggle, Enter=accept"):
    """Interactive wall check before solving. Toggling a misread edge is input
    correction shown to the demonstrator (like corner clicking); detection
    thresholds stay frozen. Returns the (possibly edited) grid."""
    def nearest_edge(x, y):
        best, bd = None, 18
        for r, c, d in grid.interior_edges():
            if d == S:
                ex, ey = c * k + k / 2, (r + 1) * k
                if abs(y - ey) < bd and c * k <= x <= (c + 1) * k:
                    best, bd = (r, c, d), abs(y - ey)
            else:
                ex, ey = (c + 1) * k, r * k + k / 2
                if abs(x - ex) < bd and r * k <= y <= (r + 1) * k:
                    best, bd = (r, c, d), abs(x - ex)
        return best

    def redraw():
        cv2.imshow(window, render_overlay(warp, grid, scores))

    def on_mouse(ev, x, y, flags, param):
        if ev == cv2.EVENT_LBUTTONDOWN:
            e = nearest_edge(x, y)
            if e:
                r, c, d = e
                if grid.has_wall(r, c, d):
                    grid.remove_wall(r, c, d)
                else:
                    grid.add_wall(r, c, d)
                redraw()

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)
    redraw()
    while True:
        key = cv2.waitKey(50) & 0xFF
        if key in (13, 10):
            break
        if key in (27, ord("q")):
            cv2.destroyWindow(window)
            raise SystemExit("review aborted")
    cv2.destroyWindow(window)
    return grid


# ---------------------------------------------------------------------------
# Obstacle course (4.2)
# ---------------------------------------------------------------------------

Cylinder = namedtuple("Cylinder", "cx cy r")   # rectified px


def detect_cylinders(warp, region, region_cells=5, k=K, lean_gain=0.055):
    """Find the 100 mm cylinders inside the obstacle region. region = (row,
    col) of its NW cell.

    Distance-transform approach: a cylinder (r ~= 28 px at K=100) contains
    points far from any background, while wall bands are thin (<= ~12 px
    half-width) - so cores of the distance map above ~18 px are cylinders,
    regardless of where they sit relative to the wall lattice. Centres are
    corrected for the parallax lean of the cylinder body (100 mm tall)."""
    r0, c0 = region
    x0, y0 = c0 * k, r0 * k
    size = region_cells * k
    sub = warp[y0:y0 + size, x0:x0 + size]
    gray = cv2.cvtColor(sub, cv2.COLOR_BGR2GRAY)
    otsu, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    dark = (gray < min(float(otsu) * 0.75, DARK_MAX_THR)).astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    dt = cv2.distanceTransform(dark, cv2.DIST_L2, 5)
    cores = (dt > 18).astype(np.uint8)
    ncc, lab = cv2.connectedComponents(cores)
    centre = warp.shape[0] / 2
    out = []
    for i in range(1, ncc):
        ys, xs = np.nonzero(lab == i)
        rad = float(dt[ys, xs].max())          # max inscribed disc ~ radius
        if not 20 <= rad <= 45:                # 100 mm dia = 27.8 px
            continue
        j = int(np.argmax(dt[ys, xs]))
        cx, cy = float(xs[j]), float(ys[j])
        # blob = base disc + leaning body; pull the centre back toward the
        # camera axis by half the lean to land on the base
        lx = lean_gain * (cx + x0 - centre)
        ly = lean_gain * (cy + y0 - centre)
        out.append(Cylinder(cx - 0.5 * lx + x0, cy - 0.5 * ly + y0, rad))
    return out


def plan_course(grid, cylinders, region, entry, exit_cell, region_cells=5,
                k=K, robot_radius_mm=75.0, margin_mm=None, exit_dir=None):
    """Occupancy-grid A* through the obstacle region, then line-of-sight
    shortcutting. entry = (r, c, dir of travel INTO the region), exit_cell =
    (r, c), exit_dir = the boundary side the exit gap is on (a corner exit
    cell touches TWO boundary sides; opening both would erase a real wall
    from the occupancy grid). Works in rectified px (1 px = CELL_MM / k mm).

    margin_mm=None tries a graduated safety margin (25 -> 15 -> 8 -> 2 mm):
    randomly-placed cylinders can leave gaps barely wider than the robot, so
    prefer the safest route that exists rather than failing outright.
    Returns (waypoints_px, occupancy_debug_image_mask)."""
    if margin_mm is None:
        blocked = None
        for m in (25.0, 15.0, 8.0, 2.0):
            wps, blocked = plan_course(grid, cylinders, region, entry,
                                       exit_cell, region_cells, k,
                                       robot_radius_mm, m, exit_dir)
            if wps is not None:
                if m < 25.0:
                    import sys
                    print(f"# note: route needs reduced safety margin {m} mm",
                          file=sys.stderr)
                return wps, blocked
        return None, blocked
    r0, c0 = region
    x0, y0 = c0 * k, r0 * k
    size = region_cells * k
    mm_per_px = CELL_MM / k
    inflate_px = int(round((robot_radius_mm + margin_mm) / mm_per_px))

    occ = np.zeros((size, size), dtype=np.uint8)
    for cyl in cylinders:
        cv2.circle(occ, (int(cyl.cx - x0), int(cyl.cy - y0)),
                   int(cyl.r + 10.0 / mm_per_px), 255, -1)
    # Region boundary walls (the course keeps its outer walls; interior walls
    # are removed in the obstacle area). Entry/exit gaps stay open.
    er, ec, ed = entry
    xr, xc = exit_cell
    # The course keeps its outer boundary walls; the only gaps are the entry
    # cell's entry side (the robot crosses it travelling with heading `ed`,
    # so it enters through the OPP[ed] side) and the exit cell's boundary side.
    wall_t = int(6 / mm_per_px)
    for i in range(region_cells):
        cells = {
            N: (r0, c0 + i), S: (r0 + region_cells - 1, c0 + i),
            W: (r0 + i, c0), E: (r0 + i, c0 + region_cells - 1),
        }
        for side, (rr, cc) in cells.items():
            is_gap = ((rr, cc) == (er, ec) and side == OPP[ed]) or \
                     ((rr, cc) == (xr, xc)
                      and (exit_dir is None or side == exit_dir))
            if is_gap:
                continue
            if side == N:
                occ[0:wall_t, i * k:(i + 1) * k] = 255
            elif side == S:
                occ[size - wall_t:size, i * k:(i + 1) * k] = 255
            elif side == W:
                occ[i * k:(i + 1) * k, 0:wall_t] = 255
            elif side == E:
                occ[i * k:(i + 1) * k, size - wall_t:size] = 255
    blocked = cv2.dilate(occ, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * inflate_px + 1, 2 * inflate_px + 1))) > 0

    def cell_centre_px(r, c):
        return (int((c + 0.5) * k) - x0, int((r + 0.5) * k) - y0)

    start_px = cell_centre_px(er, ec)
    goal_px = cell_centre_px(xr, xc)
    step = max(4, int(20.0 / mm_per_px))   # ~20 mm planning resolution

    def to_node(p):
        return (int(round(p[0] / step)), int(round(p[1] / step)))

    def node_free(nx, ny):
        x, y = nx * step, ny * step
        return 0 <= x < size and 0 <= y < size and not blocked[y, x]

    def nearest_free(node):
        """Blocked entry/exit centre (a cylinder can sit close to it): snap to
        the nearest free node, but never beyond the cell itself - planning
        from a neighbouring cell would silently redefine where the robot is."""
        if node_free(*node):
            return node
        best, bd = None, None
        span = int(0.45 * k / step)
        for dx in range(-span, span + 1):
            for dy in range(-span, span + 1):
                cand = (node[0] + dx, node[1] + dy)
                if node_free(*cand):
                    d2 = dx * dx + dy * dy
                    if bd is None or d2 < bd:
                        best, bd = cand, d2
        return best

    start_n = nearest_free(to_node(start_px))
    goal_n = nearest_free(to_node(goal_px))
    if start_n is None or goal_n is None:
        return None, blocked
    # A* 8-connected, corner cutting forbidden
    dist = {start_n: 0.0}
    prev = {}
    q = [(0.0, start_n)]
    moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
             (1, 1, 2 ** 0.5), (1, -1, 2 ** 0.5), (-1, 1, 2 ** 0.5), (-1, -1, 2 ** 0.5)]
    found = False
    while q:
        f, node = heappop(q)
        if node == goal_n:
            found = True
            break
        nx, ny = node
        g = dist[node]
        for dx, dy, w in moves:
            nn = (nx + dx, ny + dy)
            if not node_free(*nn):
                continue
            if dx and dy and (not node_free(nx + dx, ny) or not node_free(nx, ny + dy)):
                continue
            ng = g + w
            if ng < dist.get(nn, np.inf):
                dist[nn] = ng
                prev[nn] = node
                h = ((nn[0] - goal_n[0]) ** 2 + (nn[1] - goal_n[1]) ** 2) ** 0.5
                heappush(q, (ng + h, nn))
    if not found:
        return None, blocked
    path = [goal_n]
    while path[-1] in prev:
        path.append(prev[path[-1]])
    path.reverse()

    def free_segment(a, b):
        ax, ay = a[0] * step, a[1] * step
        bx, by = b[0] * step, b[1] * step
        length = max(abs(bx - ax), abs(by - ay))
        for t in np.linspace(0, 1, max(2, int(length / 2))):
            x, y = int(round(ax + (bx - ax) * t)), int(round(ay + (by - ay) * t))
            if blocked[min(y, size - 1), min(x, size - 1)]:
                return False
        return True

    simple = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not free_segment(path[i], path[j]):
            j -= 1
        simple.append(path[j])
        i = j
    wps = [(nx * step + x0, ny * step + y0) for nx, ny in simple]
    return wps, blocked


def waypoints_to_robot_frame(wps_px, entry, k=K):
    """Convert rectified-px waypoints to metres in the robot frame at course
    entry: origin = entry cell centre, +x = heading into the region, +y = left
    (CCW-positive angles, matching the firmware's nalgebra Rotation2)."""
    er, ec, ed = entry
    ox, oy = (ec + 0.5) * k, (er + 0.5) * k
    m_per_px = CELL_MM / k / 1000.0
    fwd = {N: (0, -1), E: (1, 0), S: (0, 1), W: (-1, 0)}[ed]
    left = {N: (-1, 0), E: (0, -1), S: (1, 0), W: (0, 1)}[ed]
    out = []
    for x, y in wps_px:
        du, dv = (x - ox) * m_per_px, (y - oy) * m_per_px
        out.append((du * fwd[0] + dv * fwd[1], du * left[0] + dv * left[1]))
    return out


def waypoints_to_segments(wps_px, entry, exit_dir=None, k=K):
    """The 4.2 handoff contract with the Rust side: firmware-ready
    TURN-AND-DRIVE segments (relative pivot in degrees CCW-positive, then
    drive distance in metres).

    The firmware has no primitive that can chase a lateral waypoint
    (Motion::Line only terminates on collinear targets, Motion::Arc is
    todo!()), but Motion::Pivot takes any Rotation2 and Motion::Line handles
    any straight run - so the polyline is decomposed into exactly those. The
    first pivot is relative to the robot's heading entering the course; the
    path starts at the entry cell centre (where the robot stands); a final
    zero-distance pivot aligns the robot with exit_dir so the exit->goal flr
    commands run without any hand-derived turn."""
    pts = waypoints_to_robot_frame(wps_px, entry, k)
    if pts and (abs(pts[0][0]) > 1e-6 or abs(pts[0][1]) > 1e-6):
        pts = [(0.0, 0.0)] + pts        # robot stands at the entry centre
    # drop sub-2cm intermediate points (occupancy-node snap noise): not worth
    # a robot motion, and the position error is within drive tolerance
    kept = [pts[0]]
    for p in pts[1:-1]:
        if np.hypot(p[0] - kept[-1][0], p[1] - kept[-1][1]) >= 0.02:
            kept.append(p)
    if len(pts) > 1:
        kept.append(pts[-1])
    pts = kept
    segs = []
    heading = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        dx, dy = x1 - x0, y1 - y0
        dist = float(np.hypot(dx, dy))
        if dist < 1e-4:
            continue
        bearing = float(np.degrees(np.arctan2(dy, dx)))
        turn = (bearing - heading + 180.0) % 360.0 - 180.0
        segs.append((turn, dist))
        heading = bearing
    if exit_dir is not None:
        er, ec, ed = entry
        # angle of compass heading exit_dir relative to the entry heading,
        # CCW-positive: one left turn goes ed -> (ed+3)%4 and is +90 deg
        want = (((ed - exit_dir) % 4) * 90.0 + 180.0) % 360.0 - 180.0
        turn = (want - heading + 180.0) % 360.0 - 180.0
        if abs(turn) > 0.5:
            segs.append((turn, 0.0))
    return segs
