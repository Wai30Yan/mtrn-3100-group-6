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
import copy
import json
import math
import os
import warnings
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
    # Robust extents (5th/95th percentile): a bystander's paper or shoe
    # poking over the frame edge stretches the raw bbox, which shifts the
    # side bands clean off the real edges (a real capture put the whole
    # north band onto a sheet of paper).
    rx0, rx1 = np.percentile(c[:, 0], [5, 95])
    ry0, ry1 = np.percentile(c[:, 1], [5, 95])
    cx, cy = (rx0 + rx1) / 2, (ry0 + ry1) / 2
    w, h = rx1 - rx0, ry1 - ry0
    sides = {
        "W": c[(c[:, 0] < cx - 0.38 * w) & (np.abs(c[:, 1] - cy) < 0.30 * h)],
        "E": c[(c[:, 0] > cx + 0.38 * w) & (np.abs(c[:, 1] - cy) < 0.30 * h)],
        "N": c[(c[:, 1] < cy - 0.38 * h) & (np.abs(c[:, 0] - cx) < 0.30 * w)],
        "S": c[(c[:, 1] > cy + 0.38 * h) & (np.abs(c[:, 0] - cx) < 0.30 * w)],
    }
    if any(len(v) < 20 for v in sides.values()):
        return None
    lines = {}
    for key, pts in sides.items():
        # Trimmed refit: a bystander's shoe or a sheet of paper poking over
        # the frame edge joins the bright blob and drags a plain fit wildly
        # off (seen on a real capture: the fitted top line left the image).
        pts = pts.astype(np.float32)
        for _ in range(3):
            vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_HUBER,
                                         0, 0.01, 0.01).flatten()
            resid = np.abs((pts[:, 0] - x0) * vy - (pts[:, 1] - y0) * vx)
            keep = resid < max(6.0, float(np.percentile(resid, 80)))
            if keep.all() or keep.sum() < 20:
                break
            pts = pts[keep]
        lines[key] = (float(vx), float(vy), float(x0), float(y0))

    def cross(l1, l2):
        vx1, vy1, x1, y1 = l1
        vx2, vy2, x2, y2 = l2
        A = np.array([[vx1, -vx2], [vy1, -vy2]])
        b = np.array([x2 - x1, y2 - y1])
        t = np.linalg.solve(A, b)
        return [x1 + t[0] * vx1, y1 + t[0] * vy1]

    quad = np.array([cross(lines["N"], lines["W"]), cross(lines["N"], lines["E"]),
                     cross(lines["S"], lines["E"]), cross(lines["S"], lines["W"])],
                    dtype=np.float32)
    # Sanity: a real maze quad is convex, near-square, and on the image.
    # Returning None (-> manual click) beats returning garbage that warps
    # into a diagonal smear and still "solves".
    Hi, Wi = img.shape[:2]
    if (quad[:, 0] < -0.05 * Wi).any() or (quad[:, 0] > 1.05 * Wi).any() \
            or (quad[:, 1] < -0.05 * Hi).any() or (quad[:, 1] > 1.05 * Hi).any():
        return None
    if not cv2.isContourConvex(quad.reshape(-1, 1, 2)):
        return None
    lens = [float(np.hypot(*(quad[i] - quad[(i + 1) % 4]))) for i in range(4)]
    if max(lens) > 1.45 * min(lens):
        return None
    return quad


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


def _wall_evidence(warp, ratio=0.86):
    """Per-pixel wall evidence by LOCAL contrast: a pixel is wall-ish if it is
    clearly darker than the surrounding floor, or is a cyan clip. How a wall
    looks depends entirely on how overhead the camera is - a near-vertical
    view shows only its 1-2 px top edge at grey ~90, an oblique one shows a
    15 px face at grey ~25 - so an absolute darkness cut cannot serve both."""
    gray = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
    bg = cv2.medianBlur(gray, 51).astype(np.float32)
    hsv = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)
    cyan = cv2.inRange(hsv, CYAN_LO, CYAN_HI) > 0
    return ((gray.astype(np.float32) < ratio * bg) | cyan).astype(np.float32)


def _window_max(prof, r=3):
    """Sliding max of a 1-D profile over +/-r px."""
    m = np.copy(prof)
    for s in range(1, r + 1):
        m[:-s] = np.maximum(m[:-s], prof[s:])
        m[s:] = np.maximum(m[s:], prof[:-s])
    return m


def _comb_quad(coarse, n, k, pad, size):
    """Alternative refinement: fit the cell lattice directly, with ONE pitch
    SHARED by both axes - the maze is physically square, so anything else is
    impossible. The axis with strong wall evidence pins the pitch; the weak
    axis (e.g. a washed-out hairline boundary) then only needs its offset,
    which the interior walls determine. Wide search: the coarse quad can be
    the aluminium frame with several cells of floor apron inside it (the
    ed279 lab captures), putting the true pitch far from k.
    Returns a quad in coarse-warp coordinates, or None."""
    ev = _wall_evidence(coarse)
    a0, a1 = pad + int(0.30 * size), pad + int(0.70 * size)
    mx = _window_max(ev[a0:a1, :].mean(axis=0))
    my = _window_max(ev[:, a0:a1].mean(axis=1))
    W = len(mx)
    lo = max(0, pad - 60)
    ends = [0, n]
    best = None
    for p in np.arange(0.55 * k, 1.16 * k, 0.5):
        hi = W - int(n * p) - 1
        if hi <= lo:
            continue
        offs = np.arange(lo, hi)
        idx = np.round(offs[:, None] + np.arange(n + 1) * p).astype(int)
        vx, vy = mx[idx], my[idx]
        vx[:, ends] *= 3.0             # boundary walls always exist
        vy[:, ends] *= 3.0
        sx, sy = vx.sum(axis=1), vy.sum(axis=1)
        s = float(sx.max() + sy.max())
        if best is None or s > best[0]:
            best = (s, float(offs[sx.argmax()]), float(offs[sy.argmax()]),
                    float(p))
    if best is None:
        return None
    _, ox, oy, p = best
    return np.array([[ox, oy], [ox + n * p, oy], [ox + n * p, oy + n * p],
                     [ox, oy + n * p]], dtype=np.float32)


def _decisiveness(warp, n, k):
    """How confidently the wall detector reads a rectified image:each edge score
    should sit near 0 or 1, never near the 0.5 decision boundary. A
    misregistered grid slices through walls and produces mushy middling
    scores, so this is a self-check that needs no ground truth."""
    try:
        _g, scores = detect_walls(warp, n=n, k=k)
    except Exception:
        return -1.0
    if not scores:
        return -1.0
    return float(np.mean([abs(e.score - 0.5) for e in scores]))


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

    # Disambiguate frame seams / interior walls by GLOBAL lattice consistency,
    # scored on the LENIENT local-contrast evidence (a washed-out hairline
    # boundary is invisible to the strict mask but present here).
    a0, a1 = pad + int(0.32 * size), pad + int(0.68 * size)
    ev = _wall_evidence(coarse)
    proj_y = _window_max(ev[:, a0:a1].mean(axis=1))
    proj_x = _window_max(ev[a0:a1, :].mean(axis=0))
    centre = pad + size / 2

    def comb_score(c0, pitch, proj):
        s = 0.0
        for i in range(n + 1):
            p = int(round(c0 + i * pitch))
            if 0 <= p < len(proj):
                s += float(proj[p]) * (3.0 if i in (0, n) else 1.0)
        return s

    def scored_pairs(near_lines, far_lines, proj):
        out = []
        for mn_, bn, _sn in near_lines[:4]:
            for mf, bf, _sf in far_lines[:4]:
                near_c = mn_ * centre + bn
                pitch = (mf * centre + bf - near_c) / n
                if 0.55 * k < pitch < 1.15 * k:
                    out.append((comb_score(near_c, pitch, proj), pitch,
                                (mn_, bn), (mf, bf)))
        out.sort(key=lambda t: -t[0])
        return out

    pairs_y = scored_pairs(cand_lines["top"], cand_lines["bottom"], proj_y)
    pairs_x = scored_pairs(cand_lines["left"], cand_lines["right"], proj_x)

    # The maze is SQUARE: both axes must agree on the cell pitch. Pick the
    # jointly best consistent (y-pair, x-pair); if one axis has no line pair
    # consistent with the other (its far wall was invisible - seen on the
    # ed279 captures, where the boundary refinement latched onto an interior
    # wall and stretched 5.5 real columns over the 9-column lattice), trust
    # the strong axis's pitch and DERIVE the weak axis's missing side from
    # its one good line shifted by n * pitch.
    def best_synth(pitch, near_side, far_side, proj):
        """Best derived opposite-side pair at a given pitch: each single line
        proposes the missing side n * pitch away; score the whole comb."""
        syn = [(comb_score(m * centre + b, pitch, proj), (m, b),
                (m, b + n * pitch)) for m, b, _ in cand_lines[near_side][:4]]
        syn += [(comb_score(m * centre + b - n * pitch, pitch, proj),
                 (m, b - n * pitch), (m, b))
                for m, b, _ in cand_lines[far_side][:4]]
        return max(syn, key=lambda t: t[0]) if syn else None

    # Candidates: (total comb score, top, bottom, left, right). A real
    # opposite-side pair can still be a symmetric interior-wall trap (both
    # axes one cell in, pitches agreeing), so synthesis candidates always
    # compete on score - they win when their combs land on more wall mass.
    options = []
    for sy_, py_, t_, b_ in pairs_y[:6]:
        for sx_, px_, l_, r_ in pairs_x[:6]:
            if abs(px_ - py_) <= 0.06 * max(px_, py_):
                options.append((sx_ + sy_, t_, b_, l_, r_))
    for sy_, py_, t_, b_ in pairs_y[:2]:
        s = best_synth(py_, "left", "right", proj_x)
        if s:
            options.append((sy_ + s[0], t_, b_, s[1], s[2]))
    for sx_, px_, l_, r_ in pairs_x[:2]:
        s = best_synth(px_, "top", "bottom", proj_y)
        if s:
            options.append((sx_ + s[0], s[1], s[2], l_, r_))
    joint = max(options, key=lambda t: t[0]) if options else None
    if debug:
        import sys
        print(f"# side candidates: "
              + ", ".join(f"{s}:{[(round(b, 1), sup) for _m, b, sup in v]}"
                          for s, v in cand_lines.items()), file=sys.stderr)
        print(f"# joint pair: {joint}", file=sys.stderr)
    if joint is None:
        return _pick_rectification(img, coarse, None, T @ H, dst, n, k, pad,
                                   size, debug)
    lines = {"top": joint[1], "bottom": joint[2],
             "left": joint[3], "right": joint[4]}

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
        quad = None
    return _pick_rectification(img, coarse, quad, T @ H, dst, n, k, pad, size,
                               debug)


def _pick_rectification(img, coarse, quad, TH, dst, n, k, pad, size, debug):
    """Two independent refinements disagree in different situations: the
    boundary-run fit is exact when the supplied quad already hugs the maze,
    while the lattice-comb fit wins when the quad is the aluminium FRAME
    (several cells oversized) and the boundary wall is a faint hairline.
    Rather than guess which case we are in, rectify with each and keep the
    one the wall detector reads most decisively."""
    Hinv = np.linalg.inv(TH)
    cands = []
    if quad is not None:
        cands.append(("boundary", quad))
    cq = _comb_quad(coarse, n, k, pad, size)
    if cq is not None:
        cands.append(("lattice", cq))
    cands.append(("coarse", np.array([[pad, pad], [pad + size, pad],
                                      [pad + size, pad + size],
                                      [pad, pad + size]], dtype=np.float32)))
    best = None
    for name, q in cands:
        sides = [np.linalg.norm(q[i] - q[(i + 1) % 4]) for i in range(4)]
        if not all(0.55 * size < s < 1.15 * size for s in sides):
            continue
        orig = cv2.perspectiveTransform(q.reshape(-1, 1, 2).astype(np.float64), Hinv)
        H2 = cv2.getPerspectiveTransform(orig.reshape(4, 2).astype(np.float32), dst)
        out = cv2.warpPerspective(img, H2, (size, size))
        d = _decisiveness(out, n, k)
        if debug:
            import sys
            print(f"#   {name:9s} decisiveness={d:.4f}", file=sys.stderr)
        if best is None or d > best[0] + 0.02:
            best = (d, out, H2, name)
    if best is None:
        return coarse[pad:pad + size, pad:pad + size], np.linalg.inv(Hinv)
    if debug:
        import sys
        print(f"# chose {best[3]} rectification", file=sys.stderr)
    out, H2 = _micro_align(best[1], n, k, best[2], debug)
    return out, H2


def _micro_align(out, n, k, H2, debug=False, span=30):
    """Final per-axis lattice snap, applied to WHATEVER refinement won: find
    the (offset, scale) that puts the most wall evidence exactly ON the k-px
    lattice lines, and resample. A refinement can be self-consistent yet off
    by a fraction of a cell in phase AND a few percent in pitch (seen on a
    real capture: x phase off ~23 px with ~2.5% pitch error - walls aligned
    on one side of the image drifted off-strip on the other); offset+scale
    removes exactly that failure mode for each axis independently."""
    ev = _wall_evidence(out)
    size = n * k
    band = slice(int(0.15 * size), int(0.85 * size))
    prof_x = ev[band, :].mean(axis=0)
    prof_y = ev[:, band].mean(axis=1)

    def best_fit(prof):
        best = (0.0, 1.0, -1.0)
        for scale in np.arange(0.96, 1.041, 0.005):
            for o in range(-span, span + 1):
                s = 0.0
                for i in range(n + 1):
                    key = int(round(i * k * scale)) + o
                    lo, hi = max(0, key - 2), min(len(prof), key + 3)
                    if lo < hi:
                        s += float(prof[lo:hi].max())
                if s > best[2]:
                    best = (float(o), float(scale), s)
        return best[:2]

    (ox, sx), (oy, sy) = best_fit(prof_x), best_fit(prof_y)
    if debug:
        import sys
        print(f"# micro-align: x offset={ox:.0f} scale={sx:.3f}  "
              f"y offset={oy:.0f} scale={sy:.3f}", file=sys.stderr)
    if ox == 0 and oy == 0 and sx == 1.0 and sy == 1.0:
        return out, H2
    # wall at ox + i*k*sx must land on i*k:  x' = (x - ox) / sx
    M = np.array([[1 / sx, 0, -ox / sx],
                  [0, 1 / sy, -oy / sy]], dtype=np.float32)
    out2 = cv2.warpAffine(out, M, (size, size))
    T2 = np.array([[1 / sx, 0, -ox / sx],
                   [0, 1 / sy, -oy / sy],
                   [0, 0, 1]], dtype=np.float64)
    return out2, (T2 @ H2)


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


def cylinder_mask(shape, cylinders, k=K):
    """Bool mask of the detected cylinder discs, inflated 40% + 4 px to also
    cover the leaning body of the 100 mm-tall pillar."""
    m = np.zeros(shape[:2], np.uint8)
    for c in cylinders or []:
        cv2.circle(m, (int(c.cx), int(c.cy)), int(c.r * 1.4) + 4, 1, -1)
    return m > 0


def detect_walls(warp, n=9, k=K, chamfer=1, lean_gain=0.055, exclude=None):
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

    exclude: optional bool mask of pixels that must contribute NO wall
    evidence — the detected cylinder discs (cylinder_mask): a dark pillar
    body sitting on a lattice line otherwise reads as a phantom wall.
    """
    gray = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY).astype(np.float32)
    hsv = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)
    cyan = (cv2.inRange(hsv, CYAN_LO, CYAN_HI) > 0)
    if exclude is not None:
        gray = np.where(exclude, np.nan, gray)   # NaN drops out of the stats
        cyan = cyan & ~exclude
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
            axis = 0
        else:                           # vertical edge, x = (c+1)*k
            x = (c + 1) * k
            reach = int(8 + lean_gain * abs(x - centre))
            x0 = max(0, x - (inner if x >= centre else reach))
            x1 = x + (reach if x >= centre else inner)
            g = gray[r * k + lo:r * k + hi, x0:x1]
            cy = cyan[r * k + lo:r * k + hi, x0:x1]
            axis = 1
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")     # all-NaN slices -> NaN below
            mn, mx = np.nanmin(g, axis=axis), np.nanmax(g, axis=axis)
            med = np.nanmedian(g, axis=axis)
        has_clip = cy.any(axis=axis)
        # A position is wall if its darkest transverse pixel is (a) absolutely
        # dark AND well below the brightest (floor) pixel in the slice, or
        # (b) a localized dip below the slice's own median - catches thin
        # walls washed out by reflections, while broad soft shadows (uniform,
        # so min ~= median) stay rejected - or (c) crossed by a cyan clip.
        # NaN comparisons are False, so fully-excluded slices are not-wall.
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

def reachable_from(grid, cell):
    """Set of cells reachable from `cell` through the detected walls. Used to
    explain a NO PATH result (a sealed pocket vs a genuinely blocked maze)."""
    from collections import deque
    seen = {tuple(cell)}
    q = deque([tuple(cell)])
    while q:
        r, c = q.popleft()
        for _d, r2, c2 in grid.open_neighbours(r, c):
            if (r2, c2) not in seen:
                seen.add((r2, c2))
                q.append((r2, c2))
    return seen


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
                   k=K, cylinders=None, extra_walls=None):
    """Detected walls + path over the rectified photo: the demonstrator
    evidence that the solution is image-derived, not hard-coded.
    extra_walls: (r, c, d) edges drawn ORANGE — planning-only closures
    (corridors a cylinder makes unsafe), distinct from real detected walls."""
    vis = warp.copy()
    n = grid.n
    for cyl in cylinders or []:              # measured discs, as-detected
        cv2.circle(vis, (int(cyl.cx), int(cyl.cy)), int(cyl.r),
                   (255, 0, 255), 2)
    for r, c, d in extra_walls or []:
        if d == S:
            p1, p2 = (c * k, (r + 1) * k), ((c + 1) * k, (r + 1) * k)
        else:
            p1, p2 = ((c + 1) * k, r * k), ((c + 1) * k, (r + 1) * k)
        cv2.line(vis, p1, p2, (0, 160, 255), 2)
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
    n_img = warp.shape[0] // k
    corners_cells = {(0, 0), (0, n_img - 1), (n_img - 1, 0),
                     (n_img - 1, n_img - 1)}
    out = []
    for i in range(1, ncc):
        ys, xs = np.nonzero(lab == i)
        rad = float(dt[ys, xs].max())          # max inscribed disc ~ radius
        if not 20 <= rad <= 45:                # 100 mm dia = 27.8 px
            continue
        j = int(np.argmax(dt[ys, xs]))
        cx, cy = float(xs[j]), float(ys[j])
        # the dark chamfer corner plates read as thick blobs too - and a
        # pillar can never physically be in a chamfered corner cell
        if (int((cy + y0) // k), int((cx + x0) // k)) in corners_cells:
            continue
        # blob = base disc + leaning body; pull the centre back toward the
        # camera axis by half the lean to land on the base
        lx = lean_gain * (cx + x0 - centre)
        ly = lean_gain * (cy + y0 - centre)
        out.append(Cylinder(cx - 0.5 * lx + x0, cy - 0.5 * ly + y0, rad))
    return out


def find_course_region(grid, cylinders, region_cells=5, k=K):
    """NW cell of the region_cells-square block that best matches the
    obstacle course: contains the most detected cylinder centres, ties
    broken by fewest detected interior walls (the course has none)."""
    if not cylinders:
        return None
    best = None
    for r0 in range(grid.n - region_cells + 1):
        for c0 in range(grid.n - region_cells + 1):
            inside = sum(1 for cy in cylinders
                         if r0 <= cy.cy // k < r0 + region_cells
                         and c0 <= cy.cx // k < c0 + region_cells)
            walls = sum(1 for r in range(r0, r0 + region_cells)
                        for c in range(c0, c0 + region_cells)
                        for d in (S, E)
                        if r0 <= r + DR[d] < r0 + region_cells
                        and c0 <= c + DC[d] < c0 + region_cells
                        and grid.has_wall(r, c, d))
            key = (inside, -walls)
            if best is None or key > best[0]:
                best = (key, (r0, c0))
    return best[1]


def course_gates(grid, region, region_cells=5):
    """Open boundary edges of the course as (cell_r, cell_c, open_side):
    course cells whose detected boundary side leads to an in-bounds,
    unblocked cell outside the region."""
    r0, c0 = region
    gates = []
    for i in range(region_cells):
        for side, (rr, cc) in {N: (r0, c0 + i),
                               S: (r0 + region_cells - 1, c0 + i),
                               W: (r0 + i, c0),
                               E: (r0 + i, c0 + region_cells - 1)}.items():
            r2, c2 = rr + DR[side], cc + DC[side]
            if grid.in_bounds(r2, c2) and not grid.blocked[r2, c2] \
                    and not grid.has_wall(rr, cc, side):
                gates.append((rr, cc, side))
    return gates


def solve_hybrid(grid, cylinders, start, goal, region=None, region_cells=5,
                 k=K, r_turn=0.09, margin_mm=5.0, turn_cost=1.0):
    """Normal lattice navigation stitched with a continuous crossing of the
    obstacle course (occupancy-grid A*, config-space dilation — the Path
    Planning assignment architecture), gates discovered automatically.

    Seals the course off the lattice, then for every (entry gate, exit gate)
    pair: lattice leg to the entry, plan_course through the pillars, lattice
    leg from the exit — each candidate geometrically verified against the
    physical walls + measured discs; the cheapest verified one wins.
    Returns (motions, info dict) or (None, reason)."""
    region = region if region is not None else find_course_region(grid, cylinders)
    if region is None:
        return None, "no cylinders detected - nothing to cross"
    r0, c0 = region

    def inside(r, c):
        return r0 <= r < r0 + region_cells and c0 <= c < c0 + region_cells

    if inside(*start[:2]) or inside(*goal):
        return None, ("start or goal is inside the obstacle course - use "
                      "obstacle_planner.py with explicit gates")
    cyl_in = [cy for cy in cylinders
              if inside(int(cy.cy // k), int(cy.cx // k))]
    cyl_out = [cy for cy in cylinders if cy not in cyl_in]
    lat = wall_off_cylinders(copy.deepcopy(grid), cyl_out)
    for r in range(r0, r0 + region_cells):
        for c in range(c0, c0 + region_cells):
            lat.block(r, c)
    # physical map for verification: course interior is obstacles, not walls
    phys = copy.deepcopy(grid)
    for r in range(r0, r0 + region_cells):
        for c in range(c0, c0 + region_cells):
            for d in (S, E):
                if inside(r + DR[d], c + DC[d]):
                    phys.remove_wall(r, c, d)
    gates = course_gates(grid, region, region_cells)
    if len(gates) < 2:
        return None, (f"only {len(gates)} open gate(s) detected on the "
                      f"course boundary - check the wall overlay")
    best = None
    for er, ec, eside in gates:
        pre = (er + DR[eside], ec + DC[eside])
        try:
            leg1, path1 = solve(lat, start, pre, turn_cost=turn_cost)
        except ValueError:
            continue
        if leg1 is None:
            continue
        for xr, xc, xside in gates:
            if (xr, xc, xside) == (er, ec, eside):
                continue
            outc = (xr + DR[xside], xc + DC[xside])
            try:
                leg2, path2 = solve(lat, (*outc, xside), goal,
                                    turn_cost=turn_cost)
            except ValueError:
                continue
            if leg2 is None:
                continue
            wps, _, _ = plan_course(grid, cyl_in, region,
                                    (er, ec, OPP[eside]), (xr, xc),
                                    region_cells, k, exit_dir=xside,
                                    margin_floor_mm=margin_mm)
            if wps is None:
                continue
            try:
                # leg1 ends at the pre-gate cell centre; the course polyline
                # owns the whole crossing from there to the post-gate cell
                # centre; leg2 continues from it.
                motions = path_to_motions(path1, anchor=start,
                                          start_heading=start[2],
                                          r_turn=r_turn)
                motions += course_to_motions(wps, anchor=start,
                                             exit_dir=xside)
                motions += path_to_motions(path2, anchor=start,
                                           start_heading=xside,
                                           r_turn=r_turn)
            except ValueError:
                continue
            ok, clear, _msg = check_motions(
                motions, phys, start, goal,
                circles=cylinders_to_circles(cylinders), margin_mm=margin_mm)
            if not ok:
                continue
            cost = len(path1) + len(path2) + len(wps)
            if best is None or cost < best[0]:
                best = (cost, motions,
                        dict(region=region, entry=(er, ec, eside),
                             exit=(xr, xc, xside), wps=wps, path1=path1,
                             path2=path2, clearance=clear))
    if best is None:
        return None, ("no verified route through the course between any "
                      "gate pair (a tight course may need "
                      "obstacle_planner.py with --margin 2)")
    return best[1], best[2]


def wall_off_cylinders(grid, cylinders, k=K, clear_mm=80.0):
    """Solver-grid safety net for photos that contain cylinders: add a wall
    on every corridor (cell-centre-to-centre segment) that a measured disc
    + robot radius (75) + margin (5) mm threatens, so solve() cannot route
    the robot through or beside a pillar. Cylinders sit at arbitrary
    measured positions, not on the lattice, so this is a segment-distance
    test per corridor, not a blocked cell. Mutates and returns grid — pass
    a copy if the physical map must stay clean (check_motions needs it)."""
    threat_px = clear_mm / (CELL_MM / k)
    for r, c, d in grid.interior_edges():
        ax, ay = (c + 0.5) * k, (r + 0.5) * k
        bx = ax + (k if d == E else 0)
        by = ay + (k if d == S else 0)
        for cyl in cylinders:
            t = max(0.0, min(1.0, ((cyl.cx - ax) * (bx - ax)
                                   + (cyl.cy - ay) * (by - ay)) / k ** 2))
            if math.hypot(cyl.cx - (ax + t * (bx - ax)),
                          cyl.cy - (ay + t * (by - ay))) < cyl.r + threat_px:
                grid.add_wall(r, c, d)
    return grid


def cylinders_to_circles(cylinders, k=K):
    """Measured discs as world-frame (x, y, radius_m) for check_motions."""
    return [(*px_to_world(c.cx, c.cy, k=k), c.r * CELL_MM / k / 1000.0)
            for c in cylinders]


def plan_course(grid, cylinders, region, entry, exit_cell, region_cells=5,
                k=K, robot_radius_mm=75.0, margin_mm=None, exit_dir=None,
                margin_floor_mm=2.0):
    """Continuous crossing of the obstacle course: occupancy-grid A* +
    shortcutting + clearance refinement, planned over the region PADDED BY
    ONE CELL of the detected maze - from the centre of the cell BEFORE the
    entry gate, through the gate, between the cylinders, out the exit gate,
    to the centre of the cell BEYOND it. Making the gate transits part of
    the optimisation (instead of pinning the polyline to gate-cell centres
    afterwards) is worth 3-5 mm of clearance exactly where a tight course
    can least afford losing it.

    entry = (r, c, dir of travel INTO the region); exit_dir = the boundary
    side of exit_cell the robot leaves through (required). grid = the
    detected Grid (ring-cell walls come from it; course-interior edges are
    ignored - obstacles replace walls there; other detected gates on the
    course boundary are closed so the route uses the designated ones).

    margin_mm=None tries a graduated safety margin (25 -> 15 -> 8 ->
    margin_floor_mm); the floor must match the margin check_motions will
    verify with. Returns (waypoints_px_absolute, blocked_window_mask,
    window_origin_px)."""
    if exit_dir is None:
        raise ValueError("plan_course needs exit_dir")
    if margin_mm is None:
        blocked, origin = None, (0, 0)
        ladder = [m for m in (25.0, 15.0, 8.0) if m > margin_floor_mm]
        for m in ladder + [float(margin_floor_mm)]:
            wps, blocked, origin = plan_course(
                grid, cylinders, region, entry, exit_cell, region_cells, k,
                robot_radius_mm, m, exit_dir)
            if wps is not None:
                if m < 25.0:
                    import sys
                    print(f"# note: route needs reduced safety margin {m} mm",
                          file=sys.stderr)
                return wps, blocked, origin
        return None, blocked, origin
    r0, c0 = region
    er, ec, ed = entry
    xr, xc = exit_cell
    pre = (er - DR[ed], ec - DC[ed])
    out = (xr + DR[exit_dir], xc + DC[exit_dir])
    wr0, wc0 = max(0, r0 - 1), max(0, c0 - 1)
    wr1 = min(grid.n, r0 + region_cells + 1)
    wc1 = min(grid.n, c0 + region_cells + 1)
    wx0, wy0 = wc0 * k, wr0 * k
    W, H = (wc1 - wc0) * k, (wr1 - wr0) * k
    mm_per_px = CELL_MM / k
    inflate_px = int(round((robot_radius_mm + margin_mm) / mm_per_px))

    occ = np.zeros((H, W), dtype=np.uint8)
    for cyl in cylinders:
        cv2.circle(occ, (int(cyl.cx - wx0), int(cyl.cy - wy0)),
                   int(cyl.r + 10.0 / mm_per_px), 255, -1)

    def in_region(r, c):
        return r0 <= r < r0 + region_cells and c0 <= c < c0 + region_cells

    ht = max(2, int(round(3 / mm_per_px)))       # half wall thickness in px
    entry_gate = (er, ec, OPP[ed])               # boundary side robot enters
    exit_gate = (xr, xc, exit_dir)

    def draw_edge(r, c, d):
        if d == S:
            y = (r + 1 - wr0) * k
            occ[max(0, y - ht):y + ht, (c - wc0) * k:(c + 1 - wc0) * k] = 255
        elif d == N:
            y = (r - wr0) * k
            occ[max(0, y - ht):y + ht, (c - wc0) * k:(c + 1 - wc0) * k] = 255
        elif d == E:
            x = (c + 1 - wc0) * k
            occ[(r - wr0) * k:(r + 1 - wr0) * k, max(0, x - ht):x + ht] = 255
        else:
            x = (c - wc0) * k
            occ[(r - wr0) * k:(r + 1 - wr0) * k, max(0, x - ht):x + ht] = 255

    for r in range(wr0, wr1):
        for c in range(wc0, wc1):
            if grid.blocked[r, c]:               # chamfer plates in the ring
                occ[(r - wr0) * k:(r + 1 - wr0) * k,
                    (c - wc0) * k:(c + 1 - wc0) * k] = 255
                continue
            for d in range(4):
                r2, c2 = r + DR[d], c + DC[d]
                if in_region(r, c) and in_region(r2, c2):
                    continue                     # obstacles replace walls
                on_boundary = in_region(r, c) != in_region(r2, c2) \
                    and grid.in_bounds(r2, c2)
                if (r, c, d) in (entry_gate, exit_gate) or \
                        (in_region(r2, c2) and (r2, c2, OPP[d])
                         in (entry_gate, exit_gate)):
                    continue                     # the designated gates
                if grid.has_wall(r, c, d) or on_boundary:
                    draw_edge(r, c, d)           # real wall, or a non-
                                                 # designated course gap
    blocked = cv2.dilate(occ, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * inflate_px + 1, 2 * inflate_px + 1))) > 0

    def cell_centre_px(r, c):
        return (int((c + 0.5) * k) - wx0, int((r + 0.5) * k) - wy0)

    start_px = cell_centre_px(*pre)
    goal_px = cell_centre_px(*out)
    # ~10 mm planning resolution: a tight course leaves free channels barely
    # ~15 mm wide in config space - a 20 mm grid misses them by luck of
    # where the nodes fall. The grid is still tiny.
    step = max(3, int(10.0 / mm_per_px))

    def to_node(p):
        return (int(round(p[0] / step)), int(round(p[1] / step)))

    def node_free(nx, ny):
        x, y = nx * step, ny * step
        return 0 <= x < W and 0 <= y < H and not blocked[y, x]

    def nearest_free(node):
        """Blocked start/goal centre (a cylinder or wall shadow can sit close
        to it): snap to the nearest free node, but never beyond the cell
        itself - planning from a neighbouring cell would silently redefine
        where the robot is."""
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
        return None, blocked, (wx0, wy0)
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
        return None, blocked, (wx0, wy0)
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
            if blocked[min(y, H - 1), min(x, W - 1)]:
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
    loc = [(nx * step, ny * step) for nx, ny in simple]

    # Clearance refinement: the shortcut hugs the inflation boundary, and
    # the waypoints->motions conversion sheds a few more mm - enough to
    # fail the final check in a tight course. Push each interior waypoint
    # toward the local maximum of the obstacle distance field (config-space
    # thinking, as in the path planning assignment) while keeping both
    # adjacent segments free.
    dt = cv2.distanceTransform((~blocked).astype(np.uint8), cv2.DIST_L2, 5)

    def px_free_seg(a, b):
        length = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
        for t in np.linspace(0, 1, max(2, int(length / 2))):
            x = min(int(round(a[0] + (b[0] - a[0]) * t)), W - 1)
            y = min(int(round(a[1] + (b[1] - a[1]) * t)), H - 1)
            if blocked[y, x]:
                return False
        return True

    def refine():
        for idx in range(1, len(loc) - 1):
            best = loc[idx]
            best_d = dt[min(int(best[1]), H - 1), min(int(best[0]), W - 1)]
            for dx in range(-8, 9, 2):
                for dy in range(-8, 9, 2):
                    cand = (loc[idx][0] + dx, loc[idx][1] + dy)
                    if not (0 <= cand[0] < W and 0 <= cand[1] < H):
                        continue
                    d = dt[int(cand[1]), int(cand[0])]
                    if d > best_d and px_free_seg(loc[idx - 1], cand) \
                            and px_free_seg(cand, loc[idx + 1]):
                        best, best_d = cand, d
            loc[idx] = best

    refine()
    refine()
    # Enforce >= MIN_LINE_M waypoint spacing HERE, where the inflation mask
    # can arbitrate: course_to_motions must drop sub-40mm hops (the firmware
    # cannot drive a Line shorter than its tolerance), and a blind drop
    # there bridges a corner straight through the clearance the refinement
    # just won. Removing a point is only allowed if the bridged segment
    # still respects the full inflation.
    min_gap = MIN_LINE_M * 1000.0 / mm_per_px
    i = 1
    while i < len(loc) - 1:
        d_prev = math.hypot(loc[i][0] - loc[i - 1][0],
                            loc[i][1] - loc[i - 1][1])
        d_next = math.hypot(loc[i + 1][0] - loc[i][0],
                            loc[i + 1][1] - loc[i][1])
        if (d_prev < min_gap or d_next < min_gap) \
                and px_free_seg(loc[i - 1], loc[i + 1]):
            loc.pop(i)
            i = max(1, i - 1)
        else:
            i += 1
    refine()

    wps = [(x + wx0, y + wy0) for x, y in loc]
    return wps, blocked, (wx0, wy0)


# ---------------------------------------------------------------------------
# Motion emission (the week-12 robot interface)
#
# The firmware consumes `let solution: &[Motion] = ...;` where Motion is
#   Line  { final_position: Translation2<f32>, final_speed: f32 }   straight
#   Arc   { final_position: Translation2<f32>, final_speed: f32 }   circular
#   Pivot { rotation: Rotation2<f32> }                              in place
# All coordinates ABSOLUTE in a frame FIXED TO THE MAZE: origin = the
# maze's top-left (NW) outer corner, +x = east (image right), +y = north
# (image up, so everything in the maze has negative y), angles radians
# CCW-positive with 0 = east. The robot's odometry wakes up at (0,0,0), so
# the firmware must seed it with the emitted `initial_pose` (start cell
# centre + start heading in this frame) before running `solution`.
# Rotations are implicit along Line/Arc (robot faces tangent); final_speed
# is 0.0 at the end / before a Pivot, else the firmware's TRAVEL_SPEED
# constant. (Frame convention set by David, 2026-08-12.)
# ---------------------------------------------------------------------------

CELL_M = CELL_MM / 1000.0
# Mirrors motion_manager.rs: a Line completes when the reference pose is
# within LINEAR_TOLERANCE of final_position, so a Line shorter than this
# completes instantly WITHOUT the robot moving, and a Line whose target is
# more than this off its heading ray can never complete at all.
LINEAR_TOLERANCE_M = 0.03
MIN_LINE_M = 0.04                 # shortest Line worth emitting (> tolerance)
WALL_HALF_THICK_M = 0.003         # acrylic wall half-thickness (~6 mm total)
CYLINDER_RADIUS_M = 0.05          # spec 4.2: 100 mm diameter obstacles

_FWD = {N: (0, -1), E: (1, 0), S: (0, 1), W: (-1, 0)}     # (east, south)
_LEFT = {N: (-1, 0), E: (0, -1), S: (1, 0), W: (0, 1)}


def _world_from_maze(du, dv, anchor_dir=None):
    """Maze-frame offset (metres east, metres south of the maze's top-left
    corner) -> world frame: +x = east/right, +y = north/up, so south is
    negative y. anchor_dir is ignored (kept so call sites read unchanged:
    the frame is fixed to the maze, not to the robot's start heading)."""
    return (du, -dv)


def cell_to_world(r, c, anchor=None):
    """World coordinates (metres) of cell (r, c)'s centre. Origin = the
    maze's top-left (NW) outer corner; +x east, +y north — every cell has
    positive x and negative y. anchor is ignored (fixed maze frame)."""
    return _world_from_maze((c + 0.5) * CELL_M, (r + 0.5) * CELL_M)


def px_to_world(x, y, anchor=None, k=K):
    """World coordinates (metres) of a rectified-image pixel, same fixed
    maze frame as cell_to_world."""
    return _world_from_maze(x / k * CELL_M, y / k * CELL_M)


def heading_world(d, anchor_dir=None):
    """Absolute world-frame angle (radians, CCW-positive, in (-pi, pi]) of
    compass direction d in the fixed maze frame: E = 0, N = +pi/2 (up the
    image is +y), W = pi, S = -pi/2. anchor_dir is ignored."""
    return {E: 0.0, N: math.pi / 2, W: math.pi, S: -math.pi / 2}[d]


def path_to_motions(cells, anchor, start_heading=None, r_turn=0.09):
    """Convert a cell path (from solve()) into Line/Arc motions.

    Straight runs become one combined Line; each 90-degree turn becomes an
    Arc of radius r_turn entered r_turn before the turn-cell centre and
    exited r_turn after (r_turn = half a cell keeps the robot >= 75 mm from
    both the inside corner post and the outer walls). Consecutive
    same-sense arcs with no straight between merge (U-turns become one
    180-degree Arc). A Pivot is emitted first if the robot's heading
    (start_heading; None = unknown, always pivot) doesn't match the first
    leg. Returns [("pivot", theta) | ("line", x, y) | ("arc", x, y)], all in
    the fixed maze frame (pivot theta = absolute target heading)."""
    if not 0.0 < r_turn <= CELL_M / 2:
        raise ValueError(
            f"turn radius {r_turn} m must be in (0, {CELL_M / 2}] - a larger "
            f"arc would start behind the previous cell centre")
    if len(cells) < 2:
        return []
    dirs = []
    for (r0, c0), (r1, c1) in zip(cells, cells[1:]):
        dirs.append(next(d for d in range(4)
                         if (r0 + DR[d], c0 + DC[d]) == (r1, c1)))
    pts = [cell_to_world(r, c, anchor) for r, c in cells]
    motions = []
    if start_heading is None or dirs[0] != start_heading:
        motions.append(("pivot", heading_world(dirs[0], anchor[2])))
    # runs of constant direction: (dir, index of first pt, index of last pt)
    runs = []
    s = 0
    for i in range(1, len(dirs)):
        if dirs[i] != dirs[i - 1]:
            runs.append((dirs[i - 1], s, i))
            s = i
    runs.append((dirs[-1], s, len(dirs)))

    def unit(d):
        a = heading_world(d, anchor[2])
        return (math.cos(a), math.sin(a))

    pos = pts[0]
    for j, (d, a, b) in enumerate(runs):
        u = unit(d)
        end = pts[b]
        has_turn = j + 1 < len(runs)
        if has_turn:
            end = (end[0] - r_turn * u[0], end[1] - r_turn * u[1])
        gap = math.hypot(end[0] - pos[0], end[1] - pos[1])
        if gap > 1e-6:
            motions.append(("line", end[0], end[1]))
            pos = end
        if has_turn:
            v = unit(runs[j + 1][0])
            arc_end = (pts[b][0] + r_turn * v[0], pts[b][1] + r_turn * v[1])
            sense = 1 if runs[j + 1][0] == (d + 3) % 4 else -1
            # merge with the previous motion if it is a same-sense arc that
            # ended exactly where this one starts (U-turn)
            if motions and motions[-1][0] == "arc" and motions[-1][3] == sense \
                    and math.hypot(pos[0] - motions[-1][1],
                                   pos[1] - motions[-1][2]) < 1e-6 \
                    and abs(gap) < 1e-6:
                motions[-1] = ("arc", arc_end[0], arc_end[1], sense)
            else:
                motions.append(("arc", arc_end[0], arc_end[1], sense))
            pos = arc_end
    return [m[:3] for m in motions]


def course_to_motions(wps_px, anchor, exit_dir=None, entry_cell=None,
                      exit_cell=None, k=K):
    """Obstacle-course polyline -> Pivot + Line motions (per the robot side:
    no arcs in the course).

    Every leg is an explicit Pivot to its absolute bearing followed by a
    Line: pivots are never skipped, so the robot's heading is pinned exactly
    (a skipped near-aligned pivot leaves a Line target a few degrees off its
    heading, which the firmware's Line cannot reach). A final Pivot faces
    exit_dir so the leg out of the course starts aligned. `entry_cell` and
    `exit_cell` = (row, col) pin the polyline to those cell centres: the
    entry centre is where the previous leg left the robot, and ending at the
    exit centre (rather than at A*'s snapped node, up to ~40 mm off) means
    the following leg's cell-to-cell geometry starts from a known pose.
    World frame of `anchor`; sub-2cm hops dropped."""
    pts = [px_to_world(x, y, anchor, k) for x, y in wps_px]
    if entry_cell is not None:
        c0 = cell_to_world(entry_cell[0], entry_cell[1], anchor)
        if not pts or math.hypot(pts[0][0] - c0[0], pts[0][1] - c0[1]) > 1e-6:
            pts = [c0] + pts
    if exit_cell is not None:
        cN = cell_to_world(exit_cell[0], exit_cell[1], anchor)
        if not pts or math.hypot(pts[-1][0] - cN[0], pts[-1][1] - cN[1]) > 1e-6:
            pts = pts + [cN]
    # Drop hops the firmware cannot drive (a Line shorter than
    # LINEAR_TOLERANCE completes without moving). The FINAL point is pinned,
    # so a short residual there is FOLDED INTO the previous leg by retargeting
    # it - never emitted as its own Pivot/creep/Pivot, which would spend ~200
    # degrees of in-place rotation to travel a few millimetres.
    kept = pts[:1]
    for p in pts[1:]:
        if math.hypot(p[0] - kept[-1][0], p[1] - kept[-1][1]) >= MIN_LINE_M:
            kept.append(p)
    if len(pts) > 1 and math.hypot(kept[-1][0] - pts[-1][0],
                                   kept[-1][1] - pts[-1][1]) > 1e-9:
        if len(kept) > 1:
            kept[-1] = pts[-1]        # retarget the last Line to the exact end
        else:
            kept.append(pts[-1])
    motions = []
    heading = None
    for p0, p1 in zip(kept, kept[1:]):
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        if math.hypot(dx, dy) < 1e-6:
            continue
        heading = math.atan2(dy, dx)
        motions.append(("pivot", heading))
        motions.append(("line", p1[0], p1[1]))
    if exit_dir is not None:
        motions.append(("pivot", heading_world(exit_dir, anchor[2])))
    return motions


def save_masks(warp, base, cylinders=None, k=K):
    """Lab-task-6 evidence: the binary colour-masking stages, as images.
    <base>_mask_binary.png     dark|cyan evidence (photo -> black/white)
    <base>_mask_obstacles.png  detected cylinder discs only
    <base>_mask_walls.png      evidence minus the discs = walls only
    (paths/free floor = the black remainder of the walls mask)"""
    ev = (_wall_evidence(warp) * 255).astype(np.uint8)
    if cylinders is None:
        cylinders = detect_cylinders(warp, (0, 0), warp.shape[0] // k, k=k)
    obs = cylinder_mask(warp.shape, cylinders, k).astype(np.uint8) * 255
    walls = ev.copy()
    walls[obs > 0] = 0
    paths = []
    for suffix, m in (("binary", ev), ("obstacles", obs), ("walls", walls)):
        p = f"{base}_mask_{suffix}.png"
        write_image(p, m)
        paths.append(p)
    return paths


def format_initial_pose(start):
    """Rust literal for `let initial_pose: Isometry2<f32> = todo!();`.

    The world frame is fixed to the maze (origin = top-left corner, +x east,
    +y north), so the robot's start pose is NOT the identity: the firmware
    must seed its odometry with this before executing `solution`."""
    r, c, d = start
    x, y = cell_to_world(r, c)
    th = heading_world(d)
    return (f"let initial_pose: Isometry2<f32> = "
            f"Isometry2::new(Vector2::new({x:.4f}, {y:.4f}), {th:.4f}); "
            f"// start cell ({r},{c}) facing {DIR_NAMES[d]}")


def format_motions(motions, indent="    "):
    """Rust literal pastable in place of `let solution: &[Motion] = todo!();`.
    final_speed: 0.0 on the last motion and before any Pivot, else the
    firmware's TRAVEL_SPEED constant."""
    parts = []
    for i, m in enumerate(motions):
        stop = i == len(motions) - 1 or motions[i + 1][0] == "pivot"
        speed = "0.0" if stop else "TRAVEL_SPEED"
        if m[0] == "pivot":
            parts.append(f"Motion::Pivot {{ rotation: "
                         f"Rotation2::new({m[1]:.4f}) }}")
        elif m[0] == "line":
            parts.append(f"Motion::Line {{ final_position: "
                         f"Translation2::new({m[1]:.4f}, {m[2]:.4f}), "
                         f"final_speed: {speed} }}")
        else:
            parts.append(f"Motion::Arc {{ final_position: "
                         f"Translation2::new({m[1]:.4f}, {m[2]:.4f}), "
                         f"final_speed: {speed} }}")
    if not parts:
        return "&[]"
    inner = (",\n" + indent).join(parts)
    return "&[\n" + indent + inner + ",\n]"


def simulate_motions(motions, samples_per_m=200, start=(0.0, 0.0, 0.0)):
    """Geometrically integrate a motion list from the pose `start` =
    (x, y, heading) — the robot's initial_pose in the fixed maze frame.
    Returns (sample_points, final_pose). Raises if a Line target is not
    collinear with the heading at its start - the firmware's Line primitive
    cannot reach lateral targets."""
    x, y, th = start
    pts = [(x, y)]
    for m in motions:
        if m[0] == "pivot":
            th = m[1]
            continue
        tx, ty = m[1], m[2]
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        if dist < 1e-9:
            continue
        if m[0] == "line":
            bearing = math.atan2(dy, dx)
            off = abs((bearing - th + math.pi) % (2 * math.pi) - math.pi)
            # The firmware marches its reference pose along the CURRENT
            # heading and completes only within LINEAR_TOLERANCE of the
            # target, so reachability is a LATERAL-OFFSET test, not a fixed
            # angle: dist*sin(off) must stay inside the tolerance (halved
            # for margin). A fixed 3-degree gate is far too loose on a 1 m
            # line and needlessly tight on a 0.05 m one.
            if dist * math.sin(off) > LINEAR_TOLERANCE_M / 2:
                raise ValueError(
                    f"Line target ({tx:.3f},{ty:.3f}) is "
                    f"{dist * math.sin(off) * 1000:.0f} mm off the heading ray "
                    f"({math.degrees(off):.1f} deg over {dist:.2f} m) - the "
                    f"firmware's Line would never complete")
            if dist < LINEAR_TOLERANCE_M:
                raise ValueError(
                    f"Line of {dist * 1000:.0f} mm is below the firmware's "
                    f"{LINEAR_TOLERANCE_M * 1000:.0f} mm tolerance - it would "
                    f"complete without the robot moving")
            n = max(2, int(dist * samples_per_m))
            for t in np.linspace(0, 1, n)[1:]:
                pts.append((x + dx * t, y + dy * t))
            x, y, th = tx, ty, bearing
        else:                                   # arc tangent to heading
            chord_ang = math.atan2(dy, dx)
            alpha = (chord_ang - th + math.pi) % (2 * math.pi) - math.pi
            if abs(alpha) < 1e-6:               # degenerate: straight
                pts.append((tx, ty))
                x, y = tx, ty
                continue
            r = dist / (2 * math.sin(abs(alpha)))
            sense = 1 if alpha > 0 else -1
            ox = x + r * math.cos(th + sense * math.pi / 2)
            oy = y + r * math.sin(th + sense * math.pi / 2)
            a0 = math.atan2(y - oy, x - ox)
            sweep = 2 * alpha
            n = max(3, int(abs(sweep) * r * samples_per_m))
            for t in np.linspace(0, 1, n)[1:]:
                a = a0 + sweep * t
                pts.append((ox + r * math.cos(a), oy + r * math.sin(a)))
            x, y = tx, ty
            th = (th + sweep + math.pi) % (2 * math.pi) - math.pi
    return pts, (x, y, th)


def wall_segments_world(grid, anchor):
    """Every present wall as a world-frame segment ((x0,y0),(x1,y1)) metres."""
    segs = []
    n = grid.n
    for r in range(n):
        for c in range(n):
            for d in range(4):
                if not grid.has_wall(r, c, d):
                    continue
                if d in (S, E) or r == 0 and d == N or c == 0 and d == W:
                    pass
                elif d == N and grid.has_wall(r - 1, c, S):
                    continue                      # already emitted as S
                elif d == W and grid.has_wall(r, c - 1, E):
                    continue                      # already emitted as E
                # corners of the cell in maze coords (east, south) metres
                half = CELL_M / 2
                cx, cy = 0.0, 0.0                 # relative to cell centre
                if d == N:
                    a, b = (cx - half, cy - half), (cx + half, cy - half)
                elif d == S:
                    a, b = (cx - half, cy + half), (cx + half, cy + half)
                elif d == W:
                    a, b = (cx - half, cy - half), (cx - half, cy + half)
                else:
                    a, b = (cx + half, cy - half), (cx + half, cy + half)
                base = cell_to_world(r, c, anchor)
                pa = _world_from_maze(a[0], a[1], anchor[2])
                pb = _world_from_maze(b[0], b[1], anchor[2])
                segs.append(((base[0] + pa[0], base[1] + pa[1]),
                             (base[0] + pb[0], base[1] + pb[1])))
    return segs


def min_wall_clearance(points, segs):
    """Smallest distance from any sampled path point to any wall segment."""
    best = float("inf")
    for px, py in points:
        for (x0, y0), (x1, y1) in segs:
            dx, dy = x1 - x0, y1 - y0
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x0) * dx +
                                                       (py - y0) * dy) / L2))
            d = math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
            if d < best:
                best = d
    return best


def check_motions(motions, grid, anchor, goal=None, circles=None,
                  robot_radius_mm=75.0, margin_mm=5.0):
    """Simulate the emitted motions and verify they are physically runnable:
    every Line reachable, the swept robot centre clear of all real obstacles
    (walls, plus `circles` = (x, y, radius_m) cylinders in world coords), and
    (if given) the final pose inside the goal cell.

    `grid` must be the PHYSICAL wall map - not one with planning cells
    block()ed, since blocking synthesises walls that do not exist.
    Returns (ok, clearance_mm, message)."""
    try:
        ax, ay = cell_to_world(anchor[0], anchor[1])
        pts, (fx, fy, _th) = simulate_motions(
            motions, start=(ax, ay, heading_world(anchor[2])))
    except ValueError as e:
        return False, 0.0, str(e)
    # Walls are modelled as centrelines, so subtract their half-thickness -
    # the same 6 mm band plan_course inflates, keeping the two models honest.
    clear_mm = (min_wall_clearance(pts, wall_segments_world(grid, anchor))
                - WALL_HALF_THICK_M) * 1000.0
    for cx, cy, rad in (circles or []):
        # never trust a measured radius smaller than the spec's 100 mm dia:
        # an eroded blob would otherwise certify a path that clips the cylinder
        rad = max(rad, CYLINDER_RADIUS_M)
        for px, py in pts:
            d = (math.hypot(px - cx, py - cy) - rad) * 1000.0
            if d < clear_mm:
                clear_mm = d
    need = robot_radius_mm + margin_mm
    if clear_mm < need - 0.05:      # equality is a pass, not a float lottery
        return (False, clear_mm,
                f"path passes {clear_mm:.1f} mm from a wall; the robot needs "
                f">= {need:.1f} mm (reduce --turn-radius)")
    if goal is not None:
        gx, gy = cell_to_world(goal[0], goal[1], anchor)
        if math.hypot(fx - gx, fy - gy) > CELL_M / 2:
            return (False, clear_mm,
                    f"path ends at ({fx:.3f},{fy:.3f}), not in goal cell {goal}")
    return True, clear_mm, "ok"


