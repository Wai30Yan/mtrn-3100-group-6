#!/usr/bin/env python3
# =============================================================================
#  Batch path demos: for every real photo in test_images/, pick 5 diverse
#  (start, goal) pairs inside the largest connected region of the DETECTED
#  maze, solve each, verify the emitted Motion array geometrically, and render
#  a labelled path-view image per run into test_images/path_demos/.
#
#  Usage:  python tests/path_demos.py
#
#  AI ASSISTANCE (assignment 5.1): written with the assistance of a generative
#  AI (Anthropic Claude).
# =============================================================================
import glob
import os
import random
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import mazelib as ml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(HERE, "test_images", "path_demos")


def pick_pairs(grid, k=5, seed=42, min_actions=10):
    """k diverse (start, goal) pairs inside the largest connected region:
    seeded rejection sampling, distinct starts and goals, paths of at least
    min_actions actions (relaxed if the region is too small to satisfy it)."""
    best = max(((len(ml.reachable_from(grid, (r, c))), (r, c))
                for r in range(grid.n) for c in range(grid.n)
                if not grid.blocked[r, c]))
    region = sorted(ml.reachable_from(grid, best[1]))
    rng = random.Random(seed)
    pairs, used = [], set()
    need = min_actions
    tries = 0
    while len(pairs) < k and tries < 4000:
        tries += 1
        if tries % 1000 == 0:
            need = max(4, need - 3)          # relax on small regions
        s_cell = rng.choice(region)
        g_cell = rng.choice(region)
        if s_cell == g_cell or s_cell in used or g_cell in used:
            continue
        sd = rng.randrange(4)
        cmds, path = ml.solve(grid, (*s_cell, sd), g_cell)
        if cmds is None or len(cmds) < need:
            continue
        pairs.append(((*s_cell, sd), g_cell, cmds, path))
        used.add(s_cell)
        used.add(g_cell)
    return pairs, len(region)


def label(img, text):
    bar = np.zeros((34, img.shape[1], 3), np.uint8)
    cv2.putText(bar, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack([bar, img])


def main():
    os.makedirs(OUT, exist_ok=True)
    photos = (sorted(glob.glob(os.path.join(HERE, "test_images", "*.jpg")))
              + sorted(glob.glob(os.path.join(HERE, "test_images", "ed279",
                                              "pic*.jpeg"))))
    rows = []
    for photo in photos:
        name = os.path.splitext(os.path.basename(photo))[0]
        img = cv2.imread(photo)
        corners = ml.auto_corners(img)
        if corners is None:
            print(f"{name}: CORNERS FAILED")
            continue
        warp, _ = ml.rectify(img, corners)
        grid, scores = ml.detect_walls(warp)
        # These demos exercise 4.1-style solving; the real 4.1 maze has no
        # cylinders, but some test photos do (4.2 setups). Block any cell
        # holding a cylinder so no demo path drives through one.
        cyls = ml.detect_cylinders(warp, (0, 0), grid.n)
        for c in cyls:
            r, cc = int(c.cy // ml.K), int(c.cx // ml.K)
            if grid.in_bounds(r, cc):
                grid.block(r, cc)
        pairs, region_size = pick_pairs(grid)
        print(f"{name}: largest region {region_size} cells, "
              f"{len(pairs)} runs")
        tiles = []
        for i, (start, goal, cmds, path) in enumerate(pairs, 1):
            motions = ml.path_to_motions(path, anchor=start,
                                         start_heading=start[2])
            ok, clear, msg = ml.check_motions(motions, grid, start, goal)
            vis = ml.render_overlay(warp, grid, None, path, start, goal)
            txt = (f"{name} run {i}: start {start[:2]} {ml.DIR_NAMES[start[2]]}"
                   f" -> goal {goal} | {len(cmds)} actions, "
                   f"{len(motions)} motions, clearance {clear:.0f} mm"
                   f"{'' if ok else '  ** ' + msg}")
            framed = label(vis, txt)
            out = os.path.join(
                OUT, f"{name}_run{i}_S{start[0]}-{start[1]}"
                     f"{ml.DIR_NAMES[start[2]]}_G{goal[0]}-{goal[1]}.png")
            ml.write_image(out, framed)
            tiles.append(cv2.resize(framed, (467, 485)))
            rows.append((name, i, start, goal, len(cmds), len(motions),
                         round(clear), ok))
        if tiles:
            while len(tiles) < 6:
                tiles.append(np.zeros_like(tiles[0]))
            sheet = np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:6])])
            ml.write_image(os.path.join(OUT, f"_sheet_{name}.png"), sheet)

    print(f"\n{'photo':16s} run  start        goal    actions motions clr ok")
    for r in rows:
        print(f"{r[0]:16s}  {r[1]}   {str(r[2]):12s} {str(r[3]):8s}"
              f" {r[4]:3d}     {r[5]:3d}   {r[6]:3d} {'Y' if r[7] else 'N'}")
    bad = [r for r in rows if not r[7]]
    print(f"\n{len(rows)} runs total, {len(bad)} failed verification")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
