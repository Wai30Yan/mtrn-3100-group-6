#!/usr/bin/env python3
# =============================================================================
#  MTRN3100 Micromouse - 4.2 Continuous Planning (5%).
#
#  A 5x5-cell area of the maze is an obstacle course of random 100 mm
#  cylinders. This tool: photo -> occupancy map -> collision-free trajectory
#  through the course + the flr strings for the normal-maze legs either side.
#
#  Usage (region location may be hardcoded per spec - pass its NW cell):
#      python obstacle_planner.py photo.jpg --region 2,2 --start 0,0,S \
#             --goal 8,8 --entry 2,2,E --exit 4,6
#  entry = row,col,heading-of-travel INTO the course; exit = row,col of the
#  course's last cell. Course interior walls are assumed removed (obstacles
#  replace them); its outer boundary keeps walls except the entry/exit gaps.
#
#  Output: trajectory overlay (the 1-mark demonstrator evidence), a
#  paste-ready Rust literal of TURN-AND-DRIVE segments (relative pivot in
#  degrees CCW-positive then drive metres - executable with the firmware's
#  existing Motion::Pivot + Motion::Line, first pivot relative to the entry
#  heading, final zero-distance pivot aligns with the exit direction), and
#  the flr commands for start->entry and exit->goal. The exit gap side is
#  read from the DETECTED walls, not guessed.
#
#  AI ASSISTANCE (assignment 5.1): written with the assistance of a generative
#  AI (Anthropic Claude), reviewed and tested on real and synthetic photos.
# =============================================================================
import argparse
import os
import sys

import cv2
import numpy as np

import mazelib as ml
from maze_solver import get_corners


def main():
    ap = argparse.ArgumentParser(description="obstacle course photo -> waypoints")
    ap.add_argument("image", nargs="?")
    ap.add_argument("--capture", type=int, metavar="CAM")
    ap.add_argument("--region", required=True, help="NW cell of the 5x5 course, row,col")
    ap.add_argument("--region-cells", type=int, default=5)
    ap.add_argument("--entry", required=True,
                    help="row,col,heading-of-travel-into-course e.g. 2,2,E")
    ap.add_argument("--exit", required=True, help="row,col of course exit cell")
    ap.add_argument("--start", help="maze start row,col,dir (for the full run)")
    ap.add_argument("--goal", help="maze goal row,col (for the full run)")
    ap.add_argument("--n", type=int, default=9)
    ap.add_argument("--chamfer", type=int, default=1)
    ap.add_argument("--rotate", type=int, default=0, choices=(0, 90, 180, 270))
    ap.add_argument("--corners", default="cache", choices=("cache", "auto", "click"))
    ap.add_argument("--no-ui", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    if args.capture is not None:
        img = ml.capture_frame(args.capture)
        image_path = f"capture_cam{args.capture}.png"
        cv2.imwrite(image_path, img)
    elif args.image:
        image_path = args.image
        img = cv2.imread(image_path)
        if img is None:
            raise SystemExit(f"cannot read {image_path}")
    else:
        ap.error("give an image path or --capture CAM")

    region = ml.parse_cell(args.region)
    entry = ml.parse_start(args.entry)
    exit_cell = ml.parse_cell(args.exit)

    # ---- validate the geometry BEFORE any output ---------------------------
    r0, c0 = region
    rc = args.region_cells
    er, ec, ed = entry
    xr, xc = exit_cell

    def on_border(r, c):
        return (r0 <= r < r0 + rc and c0 <= c < c0 + rc
                and (r in (r0, r0 + rc - 1) or c in (c0, c0 + rc - 1)))

    if not on_border(er, ec):
        raise SystemExit(f"--entry cell {(er, ec)} is not on the border of the "
                         f"{rc}x{rc} region at {region}")
    if not on_border(xr, xc):
        raise SystemExit(f"--exit cell {(xr, xc)} is not on the border of the "
                         f"{rc}x{rc} region at {region}")
    pre_r, pre_c = er - ml.DR[ed], ec - ml.DC[ed]      # cell before the course
    if r0 <= pre_r < r0 + rc and c0 <= pre_c < c0 + rc:
        raise SystemExit(f"--entry heading {ml.DIR_NAMES[ed]} does not cross "
                         f"the region boundary at cell {(er, ec)} - the robot "
                         f"must enter from outside the course")
    if not (0 <= pre_r < args.n and 0 <= pre_c < args.n):
        raise SystemExit(f"--entry heading {ml.DIR_NAMES[ed]} enters from "
                         f"outside the maze")

    corners = get_corners(img, image_path, args.corners, args.no_ui)
    ml.save_corners(image_path, img.shape, corners)
    warp, _ = ml.rectify(img, corners, n=args.n)
    if args.rotate:
        warp = np.ascontiguousarray(np.rot90(warp, k=(360 - args.rotate) // 90))

    # ---- detect walls FIRST: the exit gap side is read from the image ------
    grid, scores = ml.detect_walls(warp, n=args.n, chamfer=args.chamfer)
    exit_sides = []
    for d in range(4):
        r2, c2 = xr + ml.DR[d], xc + ml.DC[d]
        outside = not (r0 <= r2 < r0 + rc and c0 <= c2 < c0 + rc)
        if outside and grid.in_bounds(r2, c2) and not grid.blocked[r2, c2] \
                and not grid.has_wall(xr, xc, d):
            exit_sides.append(d)
    if not exit_sides:
        raise SystemExit(f"no open boundary side detected at exit cell "
                         f"{(xr, xc)} - check the wall overlay / --exit")
    if len(exit_sides) > 1:
        print(f"# warning: multiple open exit sides detected "
              f"({''.join(ml.DIR_NAMES[d] for d in exit_sides)}), "
              f"using {ml.DIR_NAMES[exit_sides[0]]}", file=sys.stderr)
    exit_dir = exit_sides[0]
    if grid.has_wall(er, ec, ml.OPP[ed]):
        print(f"# warning: detected a wall across the entry side "
              f"{ml.DIR_NAMES[ml.OPP[ed]]} of {(er, ec)} - check --entry",
              file=sys.stderr)

    cylinders = ml.detect_cylinders(warp, region, args.region_cells)
    print(f"# {len(cylinders)} cylinders detected", file=sys.stderr)
    for c in cylinders:
        print(f"#   at ({c.cx:.0f},{c.cy:.0f}) px, r={c.r:.0f} px "
              f"(~{2 * c.r * ml.CELL_MM / ml.K:.0f} mm dia)", file=sys.stderr)

    wps_px, blocked = ml.plan_course(None, cylinders, region, entry, exit_cell,
                                     args.region_cells, exit_dir=exit_dir)
    if wps_px is None:
        raise SystemExit("NO ROUTE through the obstacle course - check detection")

    segments = ml.waypoints_to_segments(wps_px, entry, exit_dir=exit_dir)

    # ---- overlay: occupancy + trajectory (the 1-mark evidence) -------------
    vis = warp.copy()
    r0, c0 = region
    x0, y0 = c0 * ml.K, r0 * ml.K
    size = args.region_cells * ml.K
    tint = vis[y0:y0 + size, x0:x0 + size]
    tint[blocked] = (0.55 * tint[blocked] + np.array([0, 0, 110])).astype(np.uint8)
    for c in cylinders:
        cv2.circle(vis, (int(c.cx), int(c.cy)), int(c.r), (0, 255, 255), 2)
    pts = [(int(x), int(y)) for x, y in wps_px]
    for a, b in zip(pts, pts[1:]):
        cv2.line(vis, a, b, (0, 220, 0), 4)
    for p in pts:
        cv2.circle(vis, p, 6, (255, 120, 0), -1)
    cv2.rectangle(vis, (x0, y0), (x0 + size, y0 + size), (255, 0, 255), 2)

    # ---- optional: full-maze legs around the course ------------------------
    legs = None
    if args.start and args.goal:
        start = ml.parse_start(args.start)
        goal = ml.parse_cell(args.goal)
        # The course interior is obstacles, not walls: block those cells for
        # the flr legs so the solver routes to the entry / from the exit.
        # (exit_dir was read from the DETECTED walls above, before blocking.)
        for r in range(r0, r0 + args.region_cells):
            for c in range(c0, c0 + args.region_cells):
                grid.block(r, c)
        try:
            leg1, path1 = ml.solve(grid, start, (pre_r, pre_c))
        except ValueError as e:
            raise SystemExit(f"leg 1: {e}")
        if leg1 is not None:
            # face the course entry heading at the end of leg 1
            end = ml.simulate(grid, start, leg1)
            turns = {0: "", 1: "r", 2: "rr", 3: "l"}[(ed - end[2]) % 4]
            leg1 += turns + "f"                          # step into the course
        out_r, out_c = xr + ml.DR[exit_dir], xc + ml.DC[exit_dir]
        try:
            leg2, path2 = ml.solve(grid, (out_r, out_c, exit_dir), goal)
        except ValueError as e:
            raise SystemExit(f"leg 2: {e}")
        if leg2 is not None:
            leg2 = "f" + leg2                            # step out of the course
        legs = (leg1, leg2)
        for path in (path1, path2 if leg2 else None):
            if path:
                p = [(c * ml.K + ml.K // 2, r * ml.K + ml.K // 2) for r, c in path]
                for a, b in zip(p, p[1:]):
                    cv2.line(vis, a, b, (200, 160, 0), 3)

    out = args.out or (os.path.splitext(image_path)[0] + "_course_overlay.png")
    ml.write_image(out, vis)
    print(f"# overlay: {out}", file=sys.stderr)

    # ---- outputs -----------------------------------------------------------
    print("// obstacle course, turn-and-drive segments for the robot:")
    print("// each entry = (pivot degrees, CCW/left positive - matches "
          "Rotation2), then drive metres.")
    print("// executes with the existing Motion::Pivot + Motion::Line; first "
          "pivot is relative to")
    print(f"// the heading entering the course ({ml.DIR_NAMES[ed]}); the final "
          f"0.0 m pivot leaves the robot")
    print(f"// facing {ml.DIR_NAMES[exit_dir]}, ready for the exit->goal "
          f"commands below.")
    body = ", ".join(f"({t:.1f}, {d:.3f})" for t, d in segments)
    print(f"const COURSE: &[(f32, f32)] = &[{body}];")
    wps_robot = ml.waypoints_to_robot_frame(wps_px, entry)
    print("// (waypoints in the entry frame, for reference: "
          + " ".join(f"({x:.2f},{y:.2f})" for x, y in wps_robot) + ")")
    failed = []
    if legs:
        leg1, leg2 = legs
        print(f'// start -> course entry (ends inside entry cell, facing '
              f'{ml.DIR_NAMES[ed]}):')
        print(f'//   "{leg1}"' if leg1 else "//   NO PATH")
        print(f'// course exit -> goal (the robot faces '
              f'{ml.DIR_NAMES[exit_dir]} after the final COURSE pivot):')
        print(f'//   "{leg2}"' if leg2 else "//   NO PATH")
        if leg1 is None:
            failed.append("leg 1 (start -> entry)")
        if leg2 is None:
            failed.append("leg 2 (exit -> goal)")

    if not args.no_ui:
        cv2.imshow("course - any key to close", vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    if failed:
        raise SystemExit("NO PATH for " + " and ".join(failed)
                         + " - check the wall overlay")


if __name__ == "__main__":
    main()
