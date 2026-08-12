#!/usr/bin/env python3
# =============================================================================
#  Evaluation harness for the vision pipeline.
#
#  1. Solver property test (no images): on random grids, solve() commands
#     replayed by simulate() must reach the goal, for many random start/goal.
#  2. Synthetic photo test: full pipeline (auto corners -> rectify -> detect)
#     against ground truth; reports per-edge FP/FN and, critically, whether a
#     path planned on the DETECTED map is valid on the TRUE map.
#
#  Usage:  python tests/eval.py [synth_dir]
#
#  AI ASSISTANCE (assignment 5.1): written with the assistance of a generative
#  AI (Anthropic Claude).
# =============================================================================
import glob
import json
import math
import os
import random
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import mazelib as ml
from tests.synth import random_grid


def grid_from_json(d):
    g = ml.Grid(d["n"], chamfer=0)
    g.walls = np.array(d["walls"], dtype=np.uint8)
    g.blocked = np.array(d["blocked"], dtype=bool)
    return g


def random_open_cell(grid, rng):
    while True:
        r, c = rng.randint(0, grid.n - 1), rng.randint(0, grid.n - 1)
        if not grid.blocked[r, c]:
            return r, c


def solver_property_test(rounds=60):
    rng = random.Random(7)
    fails = 0
    for i in range(rounds):
        grid = random_grid(rng=rng)
        s = (*random_open_cell(grid, rng), rng.randint(0, 3))
        g = random_open_cell(grid, rng)
        cmds, path = ml.solve(grid, s, g)
        if cmds is None:
            # connected by construction, so this is a solver bug
            fails += 1
            print(f"  round {i}: NO PATH from {s} to {g} (should exist)")
            continue
        end = ml.simulate(grid, s, cmds)
        if end[:2] != g:
            fails += 1
            print(f"  round {i}: simulation ended at {end}, wanted {g}")
    print(f"solver property test: {rounds - fails}/{rounds} passed")
    return fails == 0


def eval_image(png, verbose=True):
    gt = grid_from_json(json.load(open(png[:-4] + ".json")))
    img = cv2.imread(png)
    corners = ml.auto_corners(img)
    if corners is None:
        print(f"{os.path.basename(png)}: auto_corners FAILED")
        return dict(ok=False, fp=99, fn=99)
    warp, _ = ml.rectify(img, corners, n=gt.n)
    det, scores = ml.detect_walls(warp, n=gt.n, chamfer=1)
    fp = fn = 0
    bad = []
    for r, c, d in gt.interior_edges():
        if gt.blocked[r, c] or gt.blocked[r + ml.DR[d], c + ml.DC[d]]:
            continue                      # chamfer cells: by-design walls
        want = gt.has_wall(r, c, d)
        got = det.has_wall(r, c, d)
        if want and not got:
            fn += 1
            bad.append(f"FN({r},{c},{ml.DIR_NAMES[d]})")
        elif got and not want:
            fp += 1
            bad.append(f"FP({r},{c},{ml.DIR_NAMES[d]})")
    # end-to-end: plan on detected, must be valid on truth - both as grid
    # commands AND as the Motion array the robot actually executes
    rng = random.Random(hash(png) & 0xFFFF)
    e2e_ok = e2e_ret = mot_ok = 0
    trials = 12
    for _ in range(trials):
        s = (*random_open_cell(gt, rng), rng.randint(0, 3))
        g = random_open_cell(gt, rng)
        cmds, path = ml.solve(det, s, g)
        if cmds is None:
            continue
        try:
            end = ml.simulate(gt, s, cmds)
            if end[:2] == g:
                e2e_ok += 1
        except ValueError:
            pass
        e2e_ret += 1
        # the emitted Motions must be runnable and clear of the TRUE walls
        try:
            motions = ml.path_to_motions(path, anchor=s, start_heading=s[2])
            ok, _clear, _msg = ml.check_motions(motions, gt, s, g)
            mot_ok += bool(ok)
        except ValueError:
            pass
    if verbose:
        tag = "OK " if (fp == 0 and fn == 0 and mot_ok == e2e_ret) else "BAD"
        print(f"{tag} {os.path.basename(png)}: FP={fp} FN={fn} "
              f"e2e {e2e_ok}/{e2e_ret} motions {mot_ok}/{e2e_ret} "
              f"{' '.join(bad[:8])}")
    return dict(ok=(fp == 0 and fn == 0), fp=fp, fn=fn,
                e2e_ok=e2e_ok, e2e_n=e2e_ret, mot_ok=mot_ok)


def eval_course(png, verbose=True):
    """Obstacle scene: detected cylinders must match GT, and the planned
    trajectory must clear every TRUE cylinder by the robot radius."""
    meta = json.load(open(png[:-4] + ".json"))
    img = cv2.imread(png)
    corners = ml.auto_corners(img)
    if corners is None:
        print(f"{os.path.basename(png)}: auto_corners FAILED")
        return dict(ok=False)
    warp, _ = ml.rectify(img, corners, n=meta["n"])
    region = tuple(meta["region"])
    rc = meta["region_cells"]
    det = ml.detect_cylinders(warp, region, rc)
    gt = meta["cylinders_px"]
    matched = 0
    for gx, gy in gt:
        if any((c.cx - gx) ** 2 + (c.cy - gy) ** 2 < 20 ** 2 for c in det):
            matched += 1
    extra = len(det) - matched
    entry = tuple(meta["entry"])
    exit_cell = tuple(meta["exit"])
    exit_dir = ml.E
    wps, _blocked = ml.plan_course(None, det, region, entry, exit_cell, rc,
                                   exit_dir=exit_dir)
    clear = end_ok = None
    if wps:
        # Validate what the ROBOT actually executes: the emitted Motion list,
        # simulated geometrically, against the TRUE walls and TRUE cylinders.
        motions = ml.course_to_motions(wps, anchor=entry, exit_dir=exit_dir,
                                       entry_cell=entry[:2],
                                       exit_cell=exit_cell)
        true_grid = grid_from_json(meta)
        circles = [(*ml.px_to_world(gx, gy, entry), 0.05) for gx, gy in gt]
        try:
            pts, (fx, fy, fth) = ml.simulate_motions(
                motions, start=(*ml.cell_to_world(entry[0], entry[1]),
                                ml.heading_world(entry[2])))
            clear_m = min(
                ml.min_wall_clearance(pts, ml.wall_segments_world(true_grid, entry)),
                min((math.hypot(px - cx, py - cy) - r
                     for cx, cy, r in circles for px, py in pts), default=9.9))
            clear = clear_m * 1000.0 >= 75.0        # robot radius
            ex, ey = ml.cell_to_world(exit_cell[0], exit_cell[1], entry)
            want_th = ml.heading_world(exit_dir if exit_dir is not None
                                       else entry[2])
            dth = math.degrees(fth - want_th)
            end_ok = (math.hypot(fx - ex, fy - ey) < ml.CELL_M / 2
                      and abs((dth + 180) % 360 - 180) < 1.0)
        except ValueError:
            clear, end_ok = False, False
    ok = (matched == len(gt) and extra == 0 and wps is not None
          and clear and end_ok)
    if verbose:
        state = ('none' if not wps
                 else 'COLLIDES' if not clear
                 else 'BAD-END' if not end_ok else 'clear')
        print(f"{'OK ' if ok else 'BAD'} {os.path.basename(png)}: "
              f"cylinders {matched}/{len(gt)} (+{extra} spurious), "
              f"motions={state}")
    return dict(ok=bool(ok))


def main():
    print("== 1. solver property test ==")
    solver_ok = solver_property_test()

    synth_dir = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "synth_out")
    pngs = sorted(glob.glob(os.path.join(synth_dir, "maze_[0-9][0-9][0-9].png")))
    if not pngs:
        print(f"no synth images in {synth_dir}; run tests/synth.py first")
        sys.exit(0 if solver_ok else 1)
    print(f"== 2. synthetic photo test ({len(pngs)} images) ==")
    results = [eval_image(p) for p in pngs]
    perfect = sum(r["ok"] for r in results)
    fp = sum(r["fp"] for r in results)
    fn = sum(r["fn"] for r in results)
    e2e_ok = sum(r.get("e2e_ok", 0) for r in results)
    e2e_n = sum(r.get("e2e_n", 0) for r in results)
    print(f"summary: {perfect}/{len(results)} images perfect, "
          f"total FP={fp} FN={fn}, end-to-end paths valid {e2e_ok}/{e2e_n}")

    course_pngs = sorted(glob.glob(os.path.join(synth_dir, "course_[0-9][0-9][0-9].png")))
    course_ok = True
    if course_pngs:
        print(f"== 3. obstacle course test ({len(course_pngs)} images) ==")
        cres = [eval_course(p) for p in course_pngs]
        good = sum(r["ok"] for r in cres)
        print(f"summary: {good}/{len(cres)} courses perfect")
        course_ok = good == len(cres)

    sys.exit(0 if solver_ok and perfect == len(results) and e2e_ok == e2e_n
             and course_ok else 1)


if __name__ == "__main__":
    main()
