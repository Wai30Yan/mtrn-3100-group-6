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
    # end-to-end: plan on detected, must be valid on truth
    rng = random.Random(hash(png) & 0xFFFF)
    e2e_ok = e2e_ret = 0
    trials = 12
    for _ in range(trials):
        s = (*random_open_cell(gt, rng), rng.randint(0, 3))
        g = random_open_cell(gt, rng)
        cmds, _p = ml.solve(det, s, g)
        if cmds is None:
            continue
        try:
            end = ml.simulate(gt, s, cmds)
            if end[:2] == g:
                e2e_ok += 1
        except ValueError:
            pass
        e2e_ret += 1
    if verbose:
        tag = "OK " if (fp == 0 and fn == 0) else "BAD"
        print(f"{tag} {os.path.basename(png)}: FP={fp} FN={fn} "
              f"e2e {e2e_ok}/{e2e_ret}  {' '.join(bad[:8])}")
    return dict(ok=(fp == 0 and fn == 0), fp=fp, fn=fn,
                e2e_ok=e2e_ok, e2e_n=e2e_ret)


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
    # plan west-side middle entry -> east-side middle exit
    r0, c0 = region
    entry = (r0 + rc // 2, c0, ml.E)
    exit_cell = (r0 + rc // 2, c0 + rc - 1)
    exit_dir = ml.E
    wps, _blocked = ml.plan_course(None, det, region, entry, exit_cell, rc,
                                   exit_dir=exit_dir)
    clear = end_ok = None
    if wps:
        # Validate what the ROBOT will actually execute: integrate the
        # emitted turn-and-drive segments from the entry cell centre.
        segs = ml.waypoints_to_segments(wps, entry, exit_dir=exit_dir)
        mm_px = ml.CELL_MM / ml.K
        # entry heading E: robot +x = image +x, robot +y (left) = image -y
        px, py = (entry[1] + 0.5) * ml.K, (entry[0] + 0.5) * ml.K
        heading = 0.0
        pts = [(px, py)]
        for turn, dist in segs:
            heading += math.radians(turn)
            step_px = dist * 1000.0 / mm_px
            px += step_px * math.cos(heading)
            py -= step_px * math.sin(heading)
            pts.append((px, py))
        need = (75 + 50) / mm_px            # robot radius + cylinder radius
        clear = True
        for (x0_, y0_), (x1_, y1_) in zip(pts, pts[1:]):
            for t in np.linspace(0, 1, 40):
                qx, qy = x0_ + (x1_ - x0_) * t, y0_ + (y1_ - y0_) * t
                for gx, gy in gt:
                    if (qx - gx) ** 2 + (qy - gy) ** 2 < need ** 2:
                        clear = False
        # must end inside the exit cell, facing exit_dir (E = 0 deg relative)
        exr, exc = exit_cell
        end_ok = (abs(px - (exc + 0.5) * ml.K) < 50
                  and abs(py - (exr + 0.5) * ml.K) < 50
                  and abs((math.degrees(heading) + 180) % 360 - 180) < 1.0)
    ok = (matched == len(gt) and extra == 0 and wps is not None
          and clear and end_ok)
    if verbose:
        state = ('none' if not wps
                 else 'COLLIDES' if not clear
                 else 'BAD-END' if not end_ok else 'clear')
        print(f"{'OK ' if ok else 'BAD'} {os.path.basename(png)}: "
              f"cylinders {matched}/{len(gt)} (+{extra} spurious), "
              f"route={state}")
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
