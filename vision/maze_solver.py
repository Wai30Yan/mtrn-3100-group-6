#!/usr/bin/env python3
# =============================================================================
#  MTRN3100 Micromouse - 4.1.1 Path Generation (2%, gates 4.1.2's 8%).
#
#  Photo of the maze (file or live overhead-camera capture) -> wall map ->
#  shortest path -> 'flr' command string for the robot + overlay image proving
#  the solution is image-derived.
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
    ap.add_argument("--no-ui", action="store_true",
                    help="headless: no windows, skip interactive review")
    ap.add_argument("--out", default=None, help="overlay output path")
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
        raise SystemExit(f"NO PATH FOUND - check walls in {fail}")

    # Self-check: replay the commands on the detected wall map.
    end = ml.simulate(grid, start, commands)
    assert end[:2] == goal, f"simulation ended at {end}, expected {goal}"

    overlay = ml.render_overlay(warp, grid, scores, path, start, goal)
    out = args.out or (os.path.splitext(image_path)[0] + "_overlay.png")
    ml.write_image(out, overlay)

    print(commands)
    print(f"# {len(commands)} actions ({commands.count('f')} cells), "
          f"validated by simulation", file=sys.stderr)
    print(f"# overlay: {out}", file=sys.stderr)
    print(f'# paste into micromouse-rs/src/main.rs:\n'
          f'#   let task = Task::ChainingMovements("{commands}");', file=sys.stderr)

    if not args.no_ui:
        cv2.imshow("result - any key to close", overlay)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
