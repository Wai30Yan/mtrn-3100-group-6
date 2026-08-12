#!/usr/bin/env python3
# =============================================================================
#  MTRN3100 Micromouse - 4.1.1 Path Generation (2%, gates 4.1.2's 8%).
#
#  Photo of the maze (file or live overhead-camera capture) -> wall map ->
#  shortest path -> a Rust `&[Motion]` array for the robot + overlay image
#  proving the solution is image-derived.
#
#  Output pastes in place of the todo!() in:
#      let solution: &[Motion] = todo!();      // micromouse-rs/src/main.rs
#  Absolute world coordinates, metres/radians, in a frame fixed to the maze:
#  origin = the maze's top-left corner, +x = east (image right), +y = north
#  (image up; south is negative). The firmware seeds odometry with the
#  emitted initial_pose before running the array. Turns are emitted
#  as Motion::Arc (faster and more precise than pivoting, per the robot side);
#  straight runs and same-sense arc pairs are combined. The emitted path is
#  simulated and clearance-checked against the detected walls before printing.
#
#  Demo-day usage (Ed #131: plug the laptop into the demo-desk USB camera):
#      python maze_solver.py --capture 0 --start 4,0,E --goal 4,8
#  From a saved photo:
#      python maze_solver.py test_images/maze_fixed_cam.jpg --start 2,0,S --goal 6,8
#  (corner cells like 0,0 are chamfered off on the real arena - blocked by
#   default; pass --chamfer 0 for a maze without chamfers)
#
#  The command string is pasted into the Rust firmware:
#      Task::ChainingMovements("<output>")   in micromouse-rs/src/main.rs
#
#  AI ASSISTANCE (assignment 5.1): written with the assistance of a generative
#  AI (Anthropic Claude), reviewed and tested on real lab-camera photos.
# =============================================================================
import argparse
import copy
import os
import sys

import cv2
import numpy as np

import mazelib as ml


def get_corners(img, image_path, mode, no_ui):
    if mode == "cache" and image_path:
        c = ml.load_cached_corners(image_path, img.shape)
        if c is not None:
            print("# corners: using cached", file=sys.stderr)
            return c
    if mode in ("cache", "auto"):
        c = ml.auto_corners(img)
        if c is not None:
            print("# corners: auto-detected", file=sys.stderr)
            return c
        print("# corners: auto-detect failed", file=sys.stderr)
    if no_ui:
        raise SystemExit("no corners available in --no-ui mode")
    c = ml.click_corners(img)
    return c


def main():
    ap = argparse.ArgumentParser(description="maze photo -> flr command string")
    ap.add_argument("image", nargs="?", help="photo of the maze")
    ap.add_argument("--capture", type=int, metavar="CAM",
                    help="capture from overhead camera index instead of a file")
    ap.add_argument("--start", required=True, help="row,col,dir e.g. 0,0,S")
    ap.add_argument("--goal", required=True, help="row,col e.g. 4,7")
    ap.add_argument("--n", type=int, default=9, help="cells per side (default 9)")
    ap.add_argument("--chamfer", type=int, default=1,
                    help="corner chamfer span in cells (real arena: 1)")
    ap.add_argument("--rotate", type=int, default=0, choices=(0, 90, 180, 270),
                    help="rotate image CW so row 0 = the maze's North")
    ap.add_argument("--corners", default="cache", choices=("cache", "auto", "click"),
                    help="corner source (cache -> auto -> click fallback)")
    ap.add_argument("--turn-cost", type=float, default=1.0)
    ap.add_argument("--margin", type=float, default=5.0,
                    help="safety margin (mm) beyond the 75 mm robot radius "
                         "for the hybrid course crossing (default 5)")
    ap.add_argument("--turn-radius", type=float, default=0.09,
                    help="Arc radius in metres for 90-deg turns (default 0.09 "
                         "= half a cell); the emitted path is clearance-checked")
    ap.add_argument("--flr", action="store_true",
                    help="also print the legacy 'flr' string (week-8 demo "
                         "interface, not used for week 12)")
    ap.add_argument("--no-ui", action="store_true",
                    help="headless: no windows, skip interactive review")
    ap.add_argument("--out", default=None, help="overlay output path")
    ap.add_argument("--save-masks", action="store_true",
                    help="also save the binary colour-mask stages "
                         "(*_mask_binary/walls/obstacles.png)")
    args = ap.parse_args()

    if args.turn_cost < 0:
        ap.error("--turn-cost must be >= 0")

    if args.capture is not None:
        img = ml.capture_frame(args.capture)
        image_path = f"capture_cam{args.capture}.png"
        ml.write_image(image_path, img)       # keep the evidence frame
        print(f"# captured -> {image_path}", file=sys.stderr)
        if args.corners == "cache":
            args.corners = "auto"             # a fresh capture must never
                                              # inherit corners from an older
                                              # frame at the same filename
    elif args.image:
        image_path = args.image
        img = cv2.imread(image_path)
        if img is None:
            raise SystemExit(f"cannot read {image_path}")
    else:
        ap.error("give an image path or --capture CAM")

    start = ml.parse_start(args.start)
    goal = ml.parse_cell(args.goal)

    corners = get_corners(img, image_path, args.corners, args.no_ui)
    ml.save_corners(image_path, img.shape, corners)

    warp, _ = ml.rectify(img, corners, n=args.n)
    if args.rotate:
        warp = np.ascontiguousarray(np.rot90(warp, k=(360 - args.rotate) // 90))

    # The real 4.1 maze has no cylinders, but a photo may (4.2 setups).
    # Detect them FIRST: a dark pillar body on a lattice line reads as a
    # phantom wall unless its disc is excluded from the wall evidence. Then
    # wall off the corridors each disc threatens on a planning copy - the
    # solver cannot route through or beside a pillar, but the map stays
    # honest. Verification runs on the PHYSICAL grid + the measured discs.
    cyls = ml.detect_cylinders(warp, (0, 0), args.n)
    grid, scores = ml.detect_walls(warp, n=args.n, chamfer=args.chamfer,
                                   exclude=ml.cylinder_mask(warp.shape, cyls))

    if args.save_masks:
        base = os.path.splitext(args.out or image_path)[0]
        for p in ml.save_masks(warp, base, cylinders=cyls):
            print(f"# mask: {p}", file=sys.stderr)

    if not args.no_ui:
        grid = ml.review_walls(warp, grid, scores)

    plan = grid
    if cyls:
        print(f"# {len(cyls)} cylinders detected - corridors near them "
              f"walled off for planning", file=sys.stderr)
        plan = ml.wall_off_cylinders(copy.deepcopy(grid), cyls)
    safety = [(r, c, d) for r, c, d in plan.interior_edges()
              if plan.has_wall(r, c, d) and not grid.has_wall(r, c, d)]

    try:
        commands, path = ml.solve(plan, start, goal, turn_cost=args.turn_cost)
    except ValueError as e:
        raise SystemExit(str(e))
    if commands is None and cyls:
        # Normal path finding cannot connect start and goal: if the block is
        # the obstacle course, switch to hybrid mode - lattice legs up to the
        # course, continuous (occupancy-grid) planning through the pillars.
        motions, info = ml.solve_hybrid(grid, cyls, start, goal,
                                        r_turn=args.turn_radius,
                                        margin_mm=args.margin,
                                        turn_cost=args.turn_cost)
        if motions is not None:
            rg, (er, ec, es), (xr, xc, xs) = (info["region"], info["entry"],
                                              info["exit"])
            print(f"# lattice route blocked - hybrid crossing of the course "
                  f"at {rg}: enter {(er, ec)} from {ml.DIR_NAMES[es]}, exit "
                  f"{(xr, xc)} to {ml.DIR_NAMES[xs]}", file=sys.stderr)
            vis = ml.render_overlay(warp, grid, scores, None, start, goal,
                                    cylinders=cyls, extra_walls=safety)
            x0, y0 = rg[1] * ml.K, rg[0] * ml.K
            cv2.rectangle(vis, (x0, y0), (x0 + 5 * ml.K, y0 + 5 * ml.K),
                          (255, 0, 255), 2)
            for pth in (list(info["path1"]) + [(er, ec)],
                        [(xr, xc)] + list(info["path2"])):
                p = [(c * ml.K + ml.K // 2, r * ml.K + ml.K // 2)
                     for r, c in pth]
                for a, b in zip(p, p[1:]):
                    cv2.line(vis, a, b, (0, 200, 0), 4)
            wp = [(int(x), int(y)) for x, y in info["wps"]]
            for a, b in zip(wp, wp[1:]):
                cv2.line(vis, a, b, (0, 200, 0), 4)
            out = args.out or (os.path.splitext(image_path)[0]
                               + "_overlay.png")
            ml.write_image(out, vis)
            print(f"# overlay: {out}", file=sys.stderr)
            print("// paste in place of the todo!()s in "
                  "micromouse-rs/src/main.rs")
            print(f"// absolute world coords (m, rad), maze frame: origin = "
                  f"maze top-left corner, +x = east/right, +y = north/up "
                  f"(down is negative); start = cell {start[:2]} facing "
                  f"{ml.DIR_NAMES[start[2]]}")
            print(ml.format_initial_pose(start))
            print(f"// hybrid: maze -> course {rg} -> maze; {len(motions)} "
                  f"motions; min clearance {info['clearance']:.0f} mm")
            print("let solution: &[Motion] = "
                  + ml.format_motions(motions) + ";")
            return
        print(f"# hybrid crossing also failed: {info}", file=sys.stderr)
    if commands is None:
        overlay = ml.render_overlay(warp, grid, scores, None, start, goal,
                                    cylinders=cyls, extra_walls=safety)
        fail = args.out or "overlay_FAILED.png"
        ml.write_image(fail, overlay)
        # Say WHY: usually one of the two cells sits in a pocket the detected
        # walls seal off, which the overlay then makes obvious.
        reach = ml.reachable_from(plan, start[:2])
        where = ("the start and goal are in separate regions of the detected "
                 "maze" if goal not in reach else "unknown")
        hint = (f"  Note {len(cyls)} detected cylinders also wall off nearby "
                f"corridors (magenta discs / orange closures in the "
                f"overlay).\n" if cyls else "")
        raise SystemExit(
            f"NO PATH FOUND - {where}.\n"
            f"  start {start[:2]} can reach {len(reach)} cells; "
            f"goal {goal} is {'NOT ' if goal not in reach else ''}among them.\n"
            + hint +
            f"  Check the walls around both cells in {fail} - if one is wrong, "
            f"rerun without --no-ui and click that edge to toggle it.")

    # Self-check 1: replay the grid commands on the planning wall map.
    end = ml.simulate(plan, start, commands)
    assert end[:2] == goal, f"simulation ended at {end}, expected {goal}"

    # Build the robot motions; world frame is anchored on the start pose.
    try:
        motions = ml.path_to_motions(path, anchor=start, start_heading=start[2],
                                     r_turn=args.turn_radius)
    except ValueError as e:
        raise SystemExit(str(e))

    # Self-check 2: the motions themselves must be runnable - every Line
    # reachable by the firmware, the swept path clear of the detected walls
    # AND the measured cylinder discs, and the final pose inside the goal
    # cell. Checked against the PHYSICAL grid, not the planning copy.
    ok, clearance, msg = ml.check_motions(motions, grid, start, goal,
                                          circles=ml.cylinders_to_circles(cyls))

    overlay = ml.render_overlay(warp, grid, scores, path, start, goal,
                                cylinders=cyls, extra_walls=safety)
    out = args.out or (os.path.splitext(image_path)[0] + "_overlay.png")
    ml.write_image(out, overlay)
    # Abort BEFORE printing: a rejected array must never end up on the
    # clipboard just because the failure was on the line after it.
    if not ok:
        print(f"# overlay: {out}", file=sys.stderr)
        raise SystemExit(f"UNRUNNABLE PATH: {msg}")

    print("// paste in place of the todo!()s in micromouse-rs/src/main.rs")
    print(f"// absolute world coords (m, rad), maze frame: origin = maze "
          f"top-left corner, +x = east/right, +y = north/up (down is "
          f"negative); start = cell {start[:2]} facing "
          f"{ml.DIR_NAMES[start[2]]}")
    print(ml.format_initial_pose(start))
    print(f"// {len(motions)} motions, {commands.count('f')} cells; "
          f"min wall clearance {clearance:.0f} mm")
    print("let solution: &[Motion] = " + ml.format_motions(motions) + ";")
    if args.flr:
        print(f'// legacy week-8 string: "{commands}"')

    print(f"# overlay: {out}", file=sys.stderr)

    if not args.no_ui:
        cv2.imshow("result - any key to close", overlay)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
