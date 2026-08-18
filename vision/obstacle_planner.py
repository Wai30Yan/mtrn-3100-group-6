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
#  Output: trajectory overlay (the 1-mark demonstrator evidence) and ONE
#  Rust `&[Motion]` array for the whole run - start -> course entry (Arcs,
#  as in normal maze navigation) -> through the obstacles (Pivot + Line, per
#  the robot side) -> exit -> goal. Pastes in place of the todo!() in
#      let mut solution: Vec<Motion> = todo!();      // micromouse-rs/src/main.rs
#  Absolute world coordinates, metres/radians, in a frame fixed to the maze:
#  origin = the maze's top-left corner, +x = east (image right), +y = north
#  (image up; south is negative); firmware seeds odometry from the emitted
#  initial_pose. The exit gap side is read from the DETECTED walls, not
#  guessed.
#
#  AI ASSISTANCE (assignment 5.1): written with the assistance of a generative
#  AI (Anthropic Claude), reviewed and tested on real and synthetic photos.
# =============================================================================
import argparse
import copy
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
    ap.add_argument("--start", required=True,
                    help="maze start row,col,dir - also the world-frame origin")
    ap.add_argument("--goal", required=True, help="maze goal row,col")
    ap.add_argument("--margin", type=float, default=5.0,
                    help="safety margin (mm) beyond the 75 mm robot radius, "
                         "for both planning and the final clearance check; "
                         "lower it only if the built course is too tight "
                         "(default 5)")
    ap.add_argument("--turn-radius", type=float, default=0.09,
                    help="Arc radius (m) for turns in the normal-maze legs")
    ap.add_argument("--n", type=int, default=9)
    ap.add_argument("--chamfer", type=int, default=1)
    ap.add_argument("--rotate", type=int, default=0, choices=(0, 90, 180, 270))
    ap.add_argument("--corners", default="auto", choices=("auto", "click"))
    ap.add_argument("--no-ui", action="store_true")
    ap.add_argument("--out", default=None)
    ap.add_argument("--save-masks", action="store_true",
                    help="also save the binary colour-mask stages "
                         "(*_mask_binary/walls/obstacles.png)")
    ap.add_argument("--force", action="store_true",
                    help="emit even when wall detection looks unreliable")
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

    corners = get_corners(img, args.corners, args.no_ui)
    warp, _ = ml.rectify(img, corners, n=args.n)
    if args.rotate:
        warp = np.ascontiguousarray(np.rot90(warp, k=(360 - args.rotate) // 90))

    # ---- cylinders first (their discs are excluded from wall evidence so a
    # pillar on a lattice line can't read as a phantom wall), then walls: the
    # exit gap side is read from the image ----------------------------------
    cylinders = ml.detect_cylinders(warp, region, args.region_cells)
    grid, scores = ml.detect_walls(warp, n=args.n, chamfer=args.chamfer,
                                   exclude=ml.cylinder_mask(warp.shape,
                                                            cylinders))
    ambiguous = sum(1 for e in scores if 0.25 <= e.score < 0.75)
    if ambiguous > 20 and not args.force:
        raise SystemExit(
            f"RECTIFICATION LOOKS WRONG: {ambiguous} of {len(scores)} edges "
            f"scored ambiguously (good captures stay under ~15). The warp "
            f"probably failed - retry with --corners click, or pass --force "
            f"to emit anyway at your own risk.")
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

    print(f"# {len(cylinders)} cylinders detected", file=sys.stderr)
    for c in cylinders:
        print(f"#   at ({c.cx:.0f},{c.cy:.0f}) px, r={c.r:.0f} px "
              f"(~{2 * c.r * ml.CELL_MM / ml.K:.0f} mm dia)", file=sys.stderr)

    if args.save_masks:
        base = os.path.splitext(args.out or image_path)[0]
        for p in ml.save_masks(warp, base, cylinders=cylinders):
            print(f"# mask: {p}", file=sys.stderr)

    wps_px, blocked, worigin = ml.plan_course(
        grid, cylinders, region, entry, exit_cell, args.region_cells,
        exit_dir=exit_dir, margin_floor_mm=args.margin)
    if wps_px is None:
        raise SystemExit(
            "NO ROUTE through the obstacle course within the safety margin - "
            "check detection, or (if the built course is genuinely that "
            "tight) rerun with a smaller --margin")



    # ---- overlay: occupancy + trajectory (the 1-mark evidence) -------------
    vis = warp.copy()
    r0, c0 = region
    x0, y0 = c0 * ml.K, r0 * ml.K
    size = args.region_cells * ml.K
    wx0, wy0 = worigin
    tint = vis[wy0:wy0 + blocked.shape[0], wx0:wx0 + blocked.shape[1]]
    tint[blocked] = (0.55 * tint[blocked] + np.array([0, 0, 110])).astype(np.uint8)
    for c in cylinders:
        cv2.circle(vis, (int(c.cx), int(c.cy)), int(c.r), (0, 255, 255), 2)
    pts = [(int(x), int(y)) for x, y in wps_px]
    for a, b in zip(pts, pts[1:]):
        cv2.line(vis, a, b, (0, 220, 0), 4)
    for p in pts:
        cv2.circle(vis, p, 6, (255, 120, 0), -1)
    cv2.rectangle(vis, (x0, y0), (x0 + size, y0 + size), (255, 0, 255), 2)

    # ---- full-maze legs either side of the course --------------------------
    start = ml.parse_start(args.start)
    goal = ml.parse_cell(args.goal)
    # Snapshot the PHYSICAL wall map before blocking: block() synthesises
    # walls around the course cells that do not exist in reality, and the
    # clearance check must run against what is really there.
    phys_grid = copy.deepcopy(grid)
    # Inside the course the obstacles REPLACE the interior walls (spec 4.2),
    # so any interior wall the detector reported there is a cylinder or a
    # shadow misread as a wall - it is already modelled as a circle. Keeping
    # it would fail the clearance check against a wall that isn't there.
    for r in range(r0, r0 + rc):
        for c in range(c0, c0 + rc):
            for d in (ml.S, ml.E):
                r2, c2 = r + ml.DR[d], c + ml.DC[d]
                if r0 <= r2 < r0 + rc and c0 <= c2 < c0 + rc:
                    phys_grid.remove_wall(r, c, d)
    # ... and the entry/exit gaps are open in reality too.
    phys_grid.remove_wall(er, ec, ml.OPP[ed])
    phys_grid.remove_wall(xr, xc, exit_dir)
    # The course interior is obstacles, not walls: block those cells so the
    # grid solver routes to the entry / from the exit. (exit_dir was read
    # from the DETECTED walls above, before this blocking.)
    for r in range(r0, r0 + args.region_cells):
        for c in range(c0, c0 + args.region_cells):
            grid.block(r, c)
    try:
        leg1, path1 = ml.solve(grid, start, (pre_r, pre_c))
        out_r, out_c = xr + ml.DR[exit_dir], xc + ml.DC[exit_dir]
        leg2, path2 = ml.solve(grid, (out_r, out_c, exit_dir), goal)
    except ValueError as e:
        raise SystemExit(str(e))
    failed = ([] if leg1 is not None else ["leg 1 (start -> course entry)"]) + \
             ([] if leg2 is not None else ["leg 2 (course exit -> goal)"])
    if failed:
        ml.write_image(args.out or "course_FAILED.png", vis)
        raise SystemExit("NO PATH for " + " and ".join(failed)
                         + " - check the wall overlay")
    for path in (path1, path2):
        p = [(c * ml.K + ml.K // 2, r * ml.K + ml.K // 2) for r, c in path]
        for a, b in zip(p, p[1:]):
            cv2.line(vis, a, b, (200, 160, 0), 3)

    out = args.out or (os.path.splitext(image_path)[0] + "_course_overlay.png")
    ml.write_image(out, vis)
    print(f"# overlay: {out}", file=sys.stderr)

    # ---- one Motion array for the whole run --------------------------------
    try:
        # leg 1 ends at the pre-gate cell centre; the course polyline owns
        # the whole crossing (through both gates); leg 2 continues from the
        # post-gate cell centre facing exit_dir.
        align = ml.leg_first_dir(path2, exit_dir)
        motions = ml.path_to_motions(path1, anchor=start,
                                     start_heading=start[2],
                                     r_turn=args.turn_radius)
        # LIDAR off one cell before the zone, on again after exiting
        motions += [("lidar_off",)]
        motions += ml.course_to_motions(wps_px, anchor=start,
                                        exit_dir=align)
        motions += [("lidar_on",)]
        motions += ml.path_to_motions(path2, anchor=start,
                                      start_heading=align,
                                      r_turn=args.turn_radius)
        motions = ml.collapse_pivots(motions)
    except ValueError as e:
        raise SystemExit(str(e))

    ok, clearance, msg = ml.check_motions(
        motions, phys_grid, start, goal,
        circles=ml.cylinders_to_circles(cylinders), margin_mm=args.margin)
    # Abort BEFORE printing: a rejected array must never end up on the
    # clipboard just because the failure was on the line after it.
    if not ok:
        raise SystemExit(f"UNRUNNABLE PATH: {msg}")

    print("// paste in place of the todo!()s in micromouse-rs/src/main.rs")
    print(f"// absolute world coords (m, rad), maze frame: origin = maze "
          f"top-left corner, +x = east/right, +y = north/up (down is "
          f"negative); start = cell {start[:2]} facing "
          f"{ml.DIR_NAMES[start[2]]}")
    print(ml.format_initial_pose(start))
    print(f"// start -> course entry {(er, ec)} (Arcs) -> {len(cylinders)} "
          f"obstacles (Pivot+Line) -> exit {(xr, xc)} facing "
          f"{ml.DIR_NAMES[exit_dir]} -> goal {goal}")
    print(f"// {len(motions)} motions; min wall clearance {clearance:.0f} mm")
    print("let mut solution: Vec<Motion> = " + ml.format_motions(motions) + ";")

    if not args.no_ui:
        cv2.imshow("course - any key to close", vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
