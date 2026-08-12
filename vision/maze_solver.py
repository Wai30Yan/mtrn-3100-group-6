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

    grid, scores = ml.detect_walls(warp, n=args.n, chamfer=args.chamfer)

    if args.save_masks:
        base = os.path.splitext(args.out or image_path)[0]
        for p in ml.save_masks(warp, base):
            print(f"# mask: {p}", file=sys.stderr)

    if not args.no_ui:
        grid = ml.review_walls(warp, grid, scores)

    try:
        commands, path = ml.solve(grid, start, goal, turn_cost=args.turn_cost)
    except ValueError as e:
        raise SystemExit(str(e))
    if commands is None:
        overlay = ml.render_overlay(warp, grid, scores, None, start, goal)
        fail = args.out or "overlay_FAILED.png"
        ml.write_image(fail, overlay)
        # Say WHY: usually one of the two cells sits in a pocket the detected
        # walls seal off, which the overlay then makes obvious.
        reach = ml.reachable_from(grid, start[:2])
        where = ("the start and goal are in separate regions of the detected "
                 "maze" if goal not in reach else "unknown")
        raise SystemExit(
            f"NO PATH FOUND - {where}.\n"
            f"  start {start[:2]} can reach {len(reach)} cells; "
            f"goal {goal} is {'NOT ' if goal not in reach else ''}among them.\n"
            f"  Check the walls around both cells in {fail} - if one is wrong, "
            f"rerun without --no-ui and click that edge to toggle it.")

    # Self-check 1: replay the grid commands on the detected wall map.
    end = ml.simulate(grid, start, commands)
    assert end[:2] == goal, f"simulation ended at {end}, expected {goal}"

    # Build the robot motions; world frame is anchored on the start pose.
    try:
        motions = ml.path_to_motions(path, anchor=start, start_heading=start[2],
                                     r_turn=args.turn_radius)
    except ValueError as e:
        raise SystemExit(str(e))

    # Self-check 2: the motions themselves must be runnable - every Line
    # reachable by the firmware, the swept path clear of the detected walls,
    # and the final pose inside the goal cell.
    ok, clearance, msg = ml.check_motions(motions, grid, start, goal)

    overlay = ml.render_overlay(warp, grid, scores, path, start, goal)
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
