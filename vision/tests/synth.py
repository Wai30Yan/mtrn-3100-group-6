#!/usr/bin/env python3
# =============================================================================
#  Synthetic maze photo generator - mimics the real overhead lab-camera frames
#  (Ed #140) closely enough to exercise every detection stage: white mottled
#  floor, dark acrylic walls WITH parallax lean and specular streaks, cyan
#  clips on wall tops, lone post clips (false-positive bait), floor-plate
#  seam, chamfered corners with coloured plates, aluminium frame, perspective
#  into a 1920x1080 frame, sensor noise.
#
#  Each maze is saved as <name>.png + <name>.json (ground-truth walls).
#
#  AI ASSISTANCE (assignment 5.1): written with the assistance of a generative
#  AI (Anthropic Claude).
# =============================================================================
import json
import os
import random
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import mazelib as ml


def random_grid(n=9, chamfer=1, rng=None, extra_open=0.12):
    """Perfect maze via recursive backtracker over open cells, then remove a
    fraction of the remaining interior walls to create loops (the real maze
    has multiple routes)."""
    rng = rng or random.Random()
    grid = ml.Grid(n, chamfer)
    # start fully walled
    for r, c, d in list(grid.interior_edges()):
        grid.add_wall(r, c, d)
    open_cells = [(r, c) for r in range(n) for c in range(n)
                  if not grid.blocked[r, c]]
    start = rng.choice(open_cells)
    seen = {start}
    stack = [start]
    while stack:
        r, c = stack[-1]
        nbrs = []
        for d in range(4):
            r2, c2 = r + ml.DR[d], c + ml.DC[d]
            if grid.in_bounds(r2, c2) and not grid.blocked[r2, c2] \
                    and (r2, c2) not in seen:
                nbrs.append((d, r2, c2))
        if not nbrs:
            stack.pop()
            continue
        d, r2, c2 = rng.choice(nbrs)
        grid.remove_wall(r, c, d)
        seen.add((r2, c2))
        stack.append((r2, c2))
    # loops
    interior = [(r, c, d) for r, c, d in grid.interior_edges()
                if grid.has_wall(r, c, d)
                and not grid.blocked[r, c]
                and not grid.blocked[r + ml.DR[d], c + ml.DC[d]]]
    for r, c, d in rng.sample(interior, int(len(interior) * extra_open)):
        grid.remove_wall(r, c, d)
    return grid


CYAN_BGR = (205, 185, 45)


def _clip(img, x, y, along_x, rng):
    """Small cyan clip pair like the real wall clips."""
    L = rng.randint(10, 14)
    w = 3
    for off in (-3, 2):
        if along_x:
            cv2.rectangle(img, (int(x - L / 2), int(y + off)),
                          (int(x + L / 2), int(y + off + w)), CYAN_BGR, -1)
        else:
            cv2.rectangle(img, (int(x + off), int(y - L / 2)),
                          (int(x + off + w), int(y + L / 2)), CYAN_BGR, -1)


def render(grid, seed=0, k=ml.K, cam=None, lean_gain=0.055, extras=None):
    """Render the maze to a 1920x1080 'photo' + return (image, corners) where
    corners are the true maze corners in image coords (TL TR BR BL)."""
    rng = random.Random(seed)
    n = grid.n
    size = n * k
    margin = 70
    S = size + 2 * margin
    if cam is None:  # camera axis: near centre, offset like the real rig
        cam = (margin + size / 2 + rng.uniform(-90, 90),
               margin + size / 2 + rng.uniform(-60, 110))

    img = np.zeros((S, S, 3), np.uint8)
    # concrete outside
    img[:] = (95, 100, 104)
    noise = rng.randrange(1 << 30)
    rs = np.random.RandomState(noise & 0x7FFFFFFF)
    img = cv2.add(img, rs.randint(0, 18, (S, S, 3)).astype(np.uint8))
    # frame: silver band around the floor
    f0, f1 = margin - 46, S - margin + 46
    cv2.rectangle(img, (f0, f0), (f1, f1), (185, 186, 188), -1)
    img[f0:f1, f0:f1] = (np.array([185, 186, 188])
                         + rs.randint(-25, 25, (f1 - f0, f1 - f0, 3))).clip(0, 255)
    # floor sheet (extends a little past the walls, like the real arena)
    e0, e1 = margin - 22, S - margin + 22
    base = rng.randint(158, 175)
    floor = np.full((e1 - e0, e1 - e0, 3), base, np.uint8)
    # low-frequency mottling / lighting clouds
    small = rs.randint(-18, 14, (9, 9, 1)).astype(np.float32)
    cloud = cv2.resize(small, (e1 - e0, e1 - e0), interpolation=cv2.INTER_CUBIC)
    floor = (floor.astype(np.float32) + cloud[..., None]).clip(0, 255).astype(np.uint8)
    img[e0:e1, e0:e1] = floor
    # dark seam between floor sheet edge and frame (the trap that broke naive
    # boundary detection on the real photo)
    cv2.rectangle(img, (e0, e0), (e1 - 1, e1 - 1), (98, 100, 102), 2)
    # floor-plate seam through the middle (Ed #118)
    sx = margin + size // 2 + rng.randint(-8, 8)
    cv2.line(img, (sx, e0), (sx, e1), (128, 131, 133), 2)

    def lean(px, py):
        """Displacement of a wall-top point away from the camera axis."""
        return ((px - cam[0]) * lean_gain, (py - cam[1]) * lean_gain)

    def draw_wall(x0, y0, x1, y1):
        """Wall standing on segment (x0,y0)-(x1,y1): filled quad from base to
        displaced top, plus clips at the (displaced) ends."""
        d0, d1 = lean(x0, y0), lean(x1, y1)
        quad = np.array([[x0, y0], [x1, y1],
                         [x1 + d1[0], y1 + d1[1]], [x0 + d0[0], y0 + d0[1]]],
                        np.int32)
        shade = rng.randint(26, 48)
        cv2.fillPoly(img, [quad], (shade, shade, shade + 2))
        if rng.random() < 0.35:      # specular streak on the acrylic
            t = rng.uniform(0.2, 0.8)
            mx, my = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            md = lean(mx, my)
            g = rng.randint(90, 135)
            cv2.line(img, (int(mx + md[0] * 0.3), int(my + md[1] * 0.3)),
                     (int(mx + md[0]), int(my + md[1])), (g, g, g), 3)
        along_x = abs(x1 - x0) > abs(y1 - y0)
        for (px, py), (dx, dy) in (((x0, y0), d0), ((x1, y1), d1)):
            _clip(img, px + dx, py + dy, along_x, rng)

    # walls (S and E of each cell + outer boundary), on the k-lattice
    def P(r, c):
        return margin + c * k, margin + r * k

    for c in range(n):
        if not grid.blocked[0, c]:
            draw_wall(*P(0, c), *P(0, c + 1))
        if not grid.blocked[n - 1, c]:
            draw_wall(*P(n, c), *P(n, c + 1))
    for r in range(n):
        if not grid.blocked[r, 0]:
            draw_wall(*P(r, 0), *P(r + 1, 0))
        if not grid.blocked[r, n - 1]:
            draw_wall(*P(r, n), *P(r + 1, n))
    for r, c, d in grid.interior_edges():
        if grid.blocked[r, c] or grid.blocked[r + ml.DR[d], c + ml.DC[d]]:
            continue
        if grid.has_wall(r, c, d):
            if d == ml.S:
                draw_wall(*P(r + 1, c), *P(r + 1, c + 1))
            else:
                draw_wall(*P(r, c + 1), *P(r + 1, c + 1))
    # lone post clips at random crossings (false-positive bait, real arena
    # has these)
    for _ in range(rng.randint(4, 10)):
        r, c = rng.randint(1, n - 1), rng.randint(1, n - 1)
        x, y = P(r, c)
        dx, dy = lean(x, y)
        _clip(img, x + dx * 0.2, y + dy * 0.2, rng.random() < 0.5, rng)

    # extras: 100 mm cylinders for obstacle-course scenes (4.2)
    for extra in (extras or []):
        if extra[0] == "cyl":
            _, ex, ey = extra
            x, y = margin + ex, margin + ey
            rad = int(round(50 / (ml.CELL_MM / k)))     # 100 mm dia
            dx, dy = lean(x, y)
            shade = rng.randint(28, 50)
            for t in np.linspace(0, 1, 12):             # leaning body
                cv2.circle(img, (int(x + dx * t), int(y + dy * t)), rad,
                           (shade, shade, shade + 2), -1)
            g = rng.randint(80, 120)                    # top face + specular
            cv2.circle(img, (int(x + dx), int(y + dy)), rad,
                       (shade + 8, shade + 8, shade + 10), -1)
            cv2.ellipse(img, (int(x + dx), int(y + dy)), (rad - 4, rad - 7),
                        rng.uniform(0, 180), 0, 120, (g, g, g), 2)

    # chamfers: diagonal wall + coloured plate across each corner cell
    plates = [(160, 150, 235), (60, 130, 235), (170, 120, 120), (70, 200, 230)]
    corners_cells = [((0, 1), (1, 0)), ((0, n - 1), (1, n)),
                     ((n - 1, 0), (n, 1)), ((n - 1, n), (n, n - 1))]
    for (a, b), col in zip(corners_cells, plates):
        pa, pb = P(*a), P(*b)
        # plate behind the diagonal (drawn first, slightly outward)
        off = 12
        ca = (pa[0] + (off if pa[0] < S / 2 else -off),
              pa[1] + (off if pa[1] < S / 2 else -off))
        cb = (pb[0] + (off if pb[0] < S / 2 else -off),
              pb[1] + (off if pb[1] < S / 2 else -off))
        cv2.line(img, ca, cb, col, 10)
        draw_wall(*pa, *pb)

    # blur + sensor noise
    img = cv2.GaussianBlur(img, (3, 3), 0.7)
    img = cv2.add(img, rs.randint(0, 7, (S, S, 3)).astype(np.uint8))

    # perspective into 1920x1080
    out_w, out_h = 1920, 1080
    scale = rng.uniform(0.78, 0.90) * out_h / S
    cx, cy = out_w / 2 + rng.uniform(-120, 120), out_h / 2 + rng.uniform(-20, 20)
    half = S * scale / 2
    jit = 0.035 * S * scale
    dstq = np.array([
        [cx - half + rng.uniform(-jit, jit), cy - half + rng.uniform(-jit, jit)],
        [cx + half + rng.uniform(-jit, jit), cy - half + rng.uniform(-jit, jit)],
        [cx + half + rng.uniform(-jit, jit), cy + half + rng.uniform(-jit, jit)],
        [cx - half + rng.uniform(-jit, jit), cy + half + rng.uniform(-jit, jit)],
    ], np.float32)
    srcq = np.array([[0, 0], [S, 0], [S, S], [0, S]], np.float32)
    Hp = cv2.getPerspectiveTransform(srcq, dstq)
    canvas = np.zeros((out_h, out_w, 3), np.uint8)
    canvas[:] = (92, 97, 101)
    canvas = cv2.add(canvas, np.random.RandomState((noise >> 3) & 0x7FFFFFFF)
                     .randint(0, 20, (out_h, out_w, 3)).astype(np.uint8))
    warped = cv2.warpPerspective(img, Hp, (out_w, out_h),
                                 borderMode=cv2.BORDER_TRANSPARENT, dst=canvas)
    maze_quad = np.array([[margin, margin], [margin + size, margin],
                          [margin + size, margin + size], [margin, margin + size]],
                         np.float32)
    img_corners = cv2.perspectiveTransform(maze_quad.reshape(-1, 1, 2), Hp)
    return warped, img_corners.reshape(4, 2)


def obstacle_grid_and_cylinders(n=9, chamfer=1, region=(2, 2), region_cells=5,
                                n_cyl=4, rng=None, k=ml.K):
    """A maze whose obstacle region has no interior walls, plus randomly
    placed 100 mm cylinders inside it (>= 250 mm apart and off the border so
    a 150 mm robot always has a route). Cylinder centres in lattice px."""
    rng = rng or random.Random()
    grid = random_grid(n, chamfer, rng)
    r0, c0 = region
    for r in range(r0, r0 + region_cells):
        for c in range(c0, c0 + region_cells):
            for d in (ml.S, ml.E):
                r2, c2 = r + ml.DR[d], c + ml.DC[d]
                if r0 <= r2 < r0 + region_cells and c0 <= c2 < c0 + region_cells:
                    grid.remove_wall(r, c, d)
    # The course has exactly one entrance and one exit (spec 4.2): carve them
    # in the region boundary, mid-way along the W and E sides.
    mid = r0 + region_cells // 2
    entry = (mid, c0, ml.E)                  # entered travelling east
    exit_cell = (mid, c0 + region_cells - 1)
    grid.remove_wall(mid, c0, ml.W)
    grid.remove_wall(mid, c0 + region_cells - 1, ml.E)
    cyls = []
    mm_px = ml.CELL_MM / k
    margin_px = int(140 / mm_px)                     # keep off region border
    lo_x, hi_x = c0 * k + margin_px, (c0 + region_cells) * k - margin_px
    lo_y, hi_y = r0 * k + margin_px, (r0 + region_cells) * k - margin_px
    min_sep = 250 / mm_px
    # a legal course is traversable: keep the entry/exit cell centres
    # occupiable by the robot (>= 75 + 50 + 15 mm from any cylinder centre)
    keep_out = 140 / mm_px
    gates = [((c0 + 0.5) * k, (r0 + region_cells // 2 + 0.5) * k),
             ((c0 + region_cells - 0.5) * k, (r0 + region_cells // 2 + 0.5) * k)]
    tries = 0
    while len(cyls) < n_cyl and tries < 500:
        tries += 1
        x, y = rng.uniform(lo_x, hi_x), rng.uniform(lo_y, hi_y)
        if any((x - gx) ** 2 + (y - gy) ** 2 < keep_out ** 2 for gx, gy in gates):
            continue
        if all((x - a) ** 2 + (y - b) ** 2 > min_sep ** 2 for a, b in cyls):
            cyls.append((x, y))
    return grid, cyls, entry, exit_cell


def render_obstacles(grid, cyls, seed=0, k=ml.K, **kw):
    """Render like render(), then stamp the cylinders (dark discs with a
    leaning elliptical body and a specular arc) before the perspective step -
    done by monkey-window: we re-render with a callback."""
    # simplest: re-run render() but paint cylinders onto the flat lattice
    # image via the extras hook below.
    return render(grid, seed=seed, extras=[("cyl", x, y) for x, y in cyls], **kw)


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "synth_out")
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 12
    os.makedirs(out_dir, exist_ok=True)
    for i in range(count):
        rng = random.Random(1000 + i)
        grid = random_grid(rng=rng)
        img, corners = render(grid, seed=1000 + i)
        name = f"maze_{i:03d}"
        cv2.imwrite(os.path.join(out_dir, name + ".png"), img)
        json.dump({"n": grid.n, "walls": grid.walls.tolist(),
                   "blocked": grid.blocked.tolist(),
                   "corners": corners.tolist()},
                  open(os.path.join(out_dir, name + ".json"), "w"))
        print(f"{name}: walls rendered, corners {corners.round(0).tolist()}")
    # obstacle-course scenes (4.2)
    for i in range(max(4, count // 3)):
        rng = random.Random(2000 + i)
        grid, cyls, entry, exit_cell = obstacle_grid_and_cylinders(rng=rng)
        img, corners = render_obstacles(grid, cyls, seed=2000 + i)
        name = f"course_{i:03d}"
        cv2.imwrite(os.path.join(out_dir, name + ".png"), img)
        json.dump({"n": grid.n, "walls": grid.walls.tolist(),
                   "blocked": grid.blocked.tolist(),
                   "corners": corners.tolist(),
                   "region": [2, 2], "region_cells": 5,
                   "entry": list(entry), "exit": list(exit_cell),
                   "cylinders_px": cyls},
                  open(os.path.join(out_dir, name + ".json"), "w"))
        print(f"{name}: {len(cyls)} cylinders, entry {entry}, exit {exit_cell}")


if __name__ == "__main__":
    main()
