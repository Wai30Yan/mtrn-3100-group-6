# Vision Module Spec — MTRN3100 Micromouse (Week 12)

Owner: David (Python). Robot side: Rust firmware on `rust-lidar` branch (teammate).
Sources of truth: `MTRN3100_Micromouse.pdf` §4 (Week 12, 20% of course mark) and
staff answers on EdStem (thread numbers cited as #N below).

---

## 1. What the vision part covers (from the assignment PDF)

| Spec section | Task | Marks | Vision's role |
|---|---|---|---|
| §4.1.1 | Path Generation | **2%** | Entirely yours: photo of maze → command string. |
| §4.1.2 | Maze Completion | 8% | Robot-side, but **gated on 4.1.1** — "if your program fails to execute or output a path, you will not be able to attempt 4.1.2". |
| §4.2 | Continuous Planning | **5%** | Yours: occupancy map from image + solved trajectory image (1 mark). Robot following it (2+2 marks) is joint — your waypoints, their follower. |
| §4.3 | Autonomous Mapping | 5% | **Not mine.** Onboard exploration + display, owned by Waiyan (`micromouse-rs/src/map.rs` on `rust-lidar`: `Direction`, `MazeMap`, BFS). The robot is not tethered to the laptop. |

Your code directly earns ~3 marks and gates ~12 of the 20.

Hard rules from the PDF that shape the design:

- §1.1: off-board processing is only allowed for "computer vision for path generation" — your Python runs on the laptop; everything else runs on the robot.
- §4.1.1: demonstrated **live**. You must show the image input and the output ("to verify that your solution is not hard-coded"). **No tuning values after taking the image** — you photograph, immediately execute, and paste the output into the robot source in front of the demonstrator.
- §1.2: maze representation is free choice but must be autonomously generated from your CV solution. Start is `(row, col, direction)`, goal is `(row, col)`, zero-indexed, row 0 = north (top). Cells are 180 mm × 180 mm, walls 150 mm tall.
- §3.4's `flr` command format (`f` = forward one cell, `l` = 90° CCW, `r` = 90° CW) was the **week-8 demo interface only**. Week 12 uses the `&[Motion]` array in §4 below; the solver still works in `flr` internally because it is the natural grid representation, and `--flr` prints it for eyeballing.
- §4.2: obstacles are 100 mm ⌀ cylinders, wall height, randomly placed in a 5×5-cell region; region location **may be hardcoded**; only one entrance and one exit. Transfer format to the robot is free — spec suggests "waypoints relative to each other".
- §5.1: AI-assisted code must be labelled (file header + inline comments). Staff confirmed AI use is fine — "go nuts, just make sure you actually do irl testing" (#93).

## 2. Ground truth from EdStem (changes the naive design)

- **The image comes from a fixed overhead camera, not your phone.** Two top-down
  cameras are mounted on the rafters above the maze; you plug your laptop into a
  USB connection at the demo desk and capture the image yourself (#131, #121).
  "Ideally you can handle either camera; if you can't, make sure you get a
  certain one each time" (#131). The camera outputs **1920×1080** (measured from
  the posted photo, #140). → the tool has a `--capture` mode (cv2.VideoCapture).
- **Camera position is now fixed** and **manually clicking the 4 maze corners is
  officially fine** (#156). → interactive corner picker is the *primary* flow,
  with cached corners reused between runs on the same camera; auto-detection is
  a convenience, not a requirement.
- **You can capture debug images any time from now** (#131) and week 11 will
  likely have all-day lab sessions (#166); until week 10 you may sit in on any
  lab session (#166).
- **Maze appearance** (from the posted camera photo, #140): white/light floor,
  **dark (near-black) acrylic walls** — reflective on one side (#111), so expect
  specular highlights — with **cyan clips** at wall tops/joints (and lone clip
  pairs on some wall-less posts — not wall evidence on their own). The arena is
  a **9×9 grid** (square, matching the PDF text; the PDF's 5×9 figures are
  outdated) with **chamfered corners**: each of the four corner CELLS (exactly
  one cell per corner, measured on the rectified photo) is cut off by a 45°
  diagonal wall faced with a coloured plate (pink/orange/purple/yellow).
  Corner cells are unreachable → blocked; start/goal won't be there.
- **The aluminium frame reads as part of the floor** in any brightness-based
  segmentation (silver ≈ white), and the frame's top surface is 30–40 mm above
  the floor plane — so a "maze outline" found from the bright region is really
  the FRAME outline: up to ~20% outside the walls and projectively wrong.
  The 150 mm walls also parallax-lean away from the camera axis by up to
  ~1/3 cell at the arena edges; only wall BASES sit on the true lattice.
  Both facts drove the final rectification design (§5).
- **A seam/lip runs between the two floor plates** across the middle of the maze
  (#118) — visible as a faint line, possibly white-taped on the day. The wall
  detector must not read it as a wall (it's thin and low-contrast vs. thick dark
  walls; threshold on darkness *and* thickness).
- **"Shortest path" may be fewest actions or fewest cells** — either accepted if
  consistent and justified (#144). → solve with turn-penalised Dijkstra over
  `(cell, heading)` = "fewest actions", which is also what's fastest to execute.
- **Consecutive forwards can be chained and turns curved** on the robot (#161)
  — the command string stays `flr`; chaining is the robot's optimisation.
- **Speed marks rank only teams that complete the maze** (#129).
- **Libraries are unrestricted** provided they're in `requirements.txt` (#147).
- Walls are "infinitely high" — no crossing them (#101).

### Confirmed in the second EdStem sweep (as of 9 Aug 2026)

- **§4.2 obstacle course has NO interior walls** (#270, 8 Aug): "no walls in
  it, just the 5x5 space with obstacles inside." The run is
  `start → standard maze → obstacle course → standard maze → goal`, exactly
  what `obstacle_planner.py` emits. Staff also confirmed the region location
  and similar constants **may be edited in code on the day** — the
  no-tuning rule is specifically about CV parameters, "as it takes too long
  while marking".
- **Obstacles are placed at random, not on the posts** (#204): "you will not
  be able to do a standard maze Cartesian path through it" — continuous
  planning genuinely required, which is what the A* + shortcut does.
- **The floor gap runs down the CENTRE of a cell** (#249, 6 Aug). Another
  team hit exactly the failure I designed against — their processing reads
  it as a wall. Staff: "Because the line is in the centre of a cell, your
  method shouldn't be picking it up." Verified on our rectified photo: the
  seam sits at x=449 px, i.e. 49 px from the nearest cell boundary, while
  wall sampling only reads boundaries (x=100,200,…) with ±6–33 px strips —
  structurally immune, no threshold tuning involved. Tape may still be added.
- **§4.3 needs the robot's OLED** (#224): no screen output → no marks, since
  the marker can't tell mapping from random motion. Confirms 4.3 is fully
  onboard (Waiyan) and that a laptop viewer would have earned nothing.
- **Speed marks use the ceiling** (#213): with 11 finishers, the top 2 get
  full marks.
- **Extra lab time**: demonstrators will stay until 6pm during week-11 open
  labs *if* the myExperience response rate hits 60% (#237). Worth filling in.
- Maze wall sections and posts have gone missing from the lab (#219) — the
  arena may be short of parts when you test.

### Real §4.2 captures (Ed #279 "Pics", 12 Aug) — 8 photos, now in the repo

The first genuine obstacle-course captures: same fixed rig, **3680×2452**
(and 1440×960 for two of them), with 4–5 cylinders standing in an open area.
Stored in `vision/test_images/ed279/`. What they proved:

- **The obstacle region is the 5×5 at NW cell (0,3)** — identical across all
  eight, and it has no interior walls, matching #270.
- **Cylinders measure 94–97 mm** against the 100 mm spec — detection is
  accurate on real hardware, not just synthetics.
- **They broke, and then fixed, the rectifier.** These frames are far more
  overhead than the earlier photo, so a boundary wall shows only its 1–2 px
  top edge at grey ≈90 instead of a 15 px face at grey ≈25. The old
  refinement (which required a ≥6 px run darker than 60) skipped the real
  boundary and locked onto an interior wall 1.5 cells in, shifting the whole
  grid. Fixed by adding a **lattice-comb refinement**: instead of trying to
  isolate the outer boundary wall, fit a comb of n+1 equally spaced lines to
  *all* the maze's walls, which all sit on the same lattice. Both refinements
  now run and the pipeline keeps whichever the wall detector reads more
  decisively.
- Ambiguous-edge counts are higher on these (10–23 vs 3) but the flags sit
  **around the cylinders**, which partially darken a sampling strip. Since
  §4.2 says that region has no interior walls, `obstacle_planner` already
  ignores them.

**Still unanswered** (asked by another student under #270, no staff reply
yet): the maximum number of obstacles, and the minimum gap between them.
Our planner already degrades gracefully — it retries with a smaller safety
margin and reports when it had to — but if staff answer, check the tightest
gap is still ≥ ~190 mm centre-to-centre or the robot physically cannot fit.

## 3. Deliverables

One folder in the repo:

```
vision/
  maze_solver.py       §4.1.1  photo → &[Motion] array + overlay image
  obstacle_planner.py  §4.2    photo → occupancy map, &[Motion] array + overlay
  mazelib.py           shared: rectify, corner picker, wall/obstacle detection,
                       grid+solver, Motion emission, geometric verifier
  requirements.txt     numpy, opencv-python
  test_images/         real photos from the lab cameras
  tests/               synthetic-photo generator + ground-truth eval harness
```

**Reviewing this without reading it all** (fair complaint — it is ~1700 lines):
the contract that matters is the emitted `&[Motion]` array, and
`tests/eval.py` is the thing that checks it. Run
`./.venv/bin/python tests/eval.py`: it generates mazes with known walls,
runs the whole pipeline, and — the part worth trusting — *geometrically
simulates every emitted Motion array* and asserts each `Line` is reachable
from its start heading, the swept path clears every real wall and cylinder by
the robot radius, and the final pose lands in the goal cell. If that passes,
the arrays are runnable regardless of how the pixels got there. The detection
internals only need review if the overlay ever looks wrong.

(The old scaffold in OneDrive is superseded by this — it assumed the wrong wall
polarity and a handheld camera.)

## 4. Interface contracts with the robot (agree with teammate)

**The interface (agreed with the robot side).** Both tools print a Rust
`&[Motion]` literal that pastes in place of the `todo!()` in
`let solution: &[Motion] = todo!();`, using the revised `Motion` enum:

```rust
enum Motion {
  Line  { final_position: Translation2<f32>, final_speed: f32 },  // straight
  Arc   { final_position: Translation2<f32>, final_speed: f32 },  // circular
  Pivot { rotation: Rotation2<f32> },                             // in place
}
```

The `Task` string interface was **demo-only** and is no longer the week-12
path (`maze_solver.py --flr` still prints it, for eyeballing only).

Conventions the emitters follow:

- **Absolute coordinates**, metres and radians, in the robot's world frame:
  origin = the power-on pose = the **maze start cell centre**, +x = the start
  heading, +y = left, angles CCW-positive (matching `Rotation2`). This
  matches `StateObserver`, which zero-initialises at power-on.
  *If the robot's world frame is anchored differently, this is the one
  assumption to correct — everything else follows from it.*
- **Rotation is implicit** along `Line`/`Arc` (robot ends tangent to motion);
  `Pivot` is emitted only where the robot must turn on the spot.
- **`Arc` for normal maze navigation** (faster and more precise): each 90°
  turn is a quarter-circle of radius `--turn-radius` (default 0.09 m = half a
  cell), entered and exited that far from the turn cell's centre.
- **`Line` + `Pivot` only inside the obstacle course.**
- **Runs are combined**: consecutive straights become one `Line`, and
  consecutive same-sense arcs with no straight between become one `Arc`
  (a U-turn is a single 180° `Arc`).
- **`final_speed`** is `0.0` on the last motion and on any motion immediately
  before a `Pivot`, else `TRAVEL_SPEED`.

Every emitted array is **geometrically simulated before printing**: each
`Line` must be reachable along the heading it starts from, the swept path
must clear all detected walls and cylinders by ≥ 80 mm (75 mm robot radius +
margin), and the final pose must land in the goal cell. The tool exits
non-zero and says why if not — so a pasted array has already been checked
against the photo it came from.

Two things I need from you to finish this:

1. **`TRAVEL_SPEED`** — the emitted arrays reference it by name; tell me the
   value/const you settle on (or I'll inline a number).
2. **Confirm the world-frame origin** above, and whether `Motion::Arc`'s
   implicit geometry is "circle tangent to the current heading through
   `final_position`" — that's what I assumed, and it's what makes the arc
   unique given the enum has no radius/centre field.

## 5. Pipeline design

### 5.1 maze_solver.py (§4.1.1)

```
python maze_solver.py photo.jpg --start 0,0,S --goal 4,7          # from file
python maze_solver.py --capture 0 --start 0,0,S --goal 4,7       # from lab camera
```

1. **Corners.** Default flow: cached corners → automatic detection → manual
   click (TL, TR, BR, BL; staff-sanctioned #156). Auto-detection line-fits the
   sides of the bright floor/frame region — deliberately allowed to overshoot,
   because step 2 re-anchors precisely.
2. **Rectify (two-stage).** Coarse homography from the corner quad, then find
   each outer boundary wall's BASE line inside it: scan inward past frame
   seams/shadows collecting every thick dark run per sample column, group the
   runs into straight-line candidates (1-D RANSAC), and pick the
   opposite-side line PAIR whose implied 9-cell pitch best explains the
   interior wall projections (global lattice consistency — this is what
   rejects the frame seam, which locally looks exactly like a wall). The four
   chosen lines intersect into the true maze corners; the original image is
   re-warped through them in one step to 900×900 (K=100 px/cell). Verified to
   a few px of registration everywhere on the real photo.
3. **Detect walls.** Per interior edge, an asymmetric strip: a few px on the
   inner side of the lattice line, parallax-scaled reach on the outer side
   (wall faces project outward, up to ~33 px near the edges). A strip
   position counts as wall if its darkest transverse pixel is (a) absolutely
   dark and well below the brightest pixel of the slice, or (b) a localized
   dip below the slice's own median — catches thin near-axis walls and walls
   washed out by reflections while broad soft shadows (uniform, min≈median)
   stay rejected — or (c) crossed by a cyan clip. Wall present at ≥50%
   coverage of the central 55% of the edge; scores in 0.25–0.75 are flagged
   on the overlay for the eye.
4. **Review overlay.** Before solving, detected walls render over the
   rectified photo with ambiguous edges flagged. Click an edge to toggle if
   visibly wrong — input correction shown to the demonstrator, same category
   as corner clicking; the *thresholds* stay frozen.
5. **Solve.** Dijkstra over `(cell, heading)`: cost 1 per `f`, cost 1 per turn
   (90°). Minimises actions (#144-sanctioned) which minimises run time. The
   commands are replayed on the detected map (`simulate`) before printing —
   the tool cannot output a path that crosses its own detected walls.
6. **Emit.** Convert the cell path to `Line`/`Arc` motions (see §4), simulate
   and clearance-check them, print the Rust array; save + display
   `*_overlay.png` with walls + path + start/goal → demonstrator evidence.

### 5.2 obstacle_planner.py (§4.2)

```
python obstacle_planner.py photo.jpg --region 2,2 --entry 4,2,E --exit 4,6 \
       --start 4,0,E --goal 4,8
```

1. Same corner/rectify flow (full maze); `--region row,col` = NW cell of the
   5×5 course (hardcoded per spec allowance).
2. **Detect cylinders** by distance transform: a 100 mm cylinder (~28 px
   radius) contains points ≥18 px from any background while wall bands are
   thin — so cylinder cores pop out regardless of where they sit relative to
   the wall lattice, with centres corrected for body lean. Measured accuracy
   on synthetic scenes: ±2 mm diameter.
3. **Occupancy grid** at 20 mm resolution: cylinders (detected radius +10 mm)
   and region boundary walls (gaps at entry/exit only), inflated by robot
   radius (75 mm) + a **graduated margin (25→15→8→2 mm)** — random cylinders
   can leave gaps barely wider than the robot, so the planner prefers the
   safest route that exists and reports when it had to shrink.
4. **Plan** entry→exit with A* (8-connected, corner-cutting forbidden),
   snapping a blocked entry/exit centre to the nearest free node in its cell,
   then line-of-sight shortcut to sparse waypoints.
5. **Emit.** Trajectory overlay (the 1-mark evidence) + **one** `&[Motion]`
   array for the whole run: start → course entry (Arcs) → through the
   obstacles (Pivot + Line) → exit → goal (Arcs). The course polyline is
   pinned to the entry and exit cell centres so the legs join at known
   poses, and the exit-gap side is read from the **detected** walls.

*(§4.3 autonomous mapping is Waiyan's onboard `map.rs` — no laptop tool.)*

## 6. Demo-day flow (rehearse end-to-end in week 11)

1. Demonstrator writes start/goal (and the string goes on the whiteboard for
   3.4-style tasks; expect the same style).
2. Plug laptop into the demo-desk USB camera; `--capture 0`.
3. Click 4 corners (or reuse cache), eyeball the wall overlay, approve.
4. Command string prints; paste into `main.rs`, `./deploy`, place robot, run.

Time the paste-and-flash step with the teammate — minutes, not tens.

## 7. Test plan and current results

Run everything with:

```
cd vision && ./.venv/bin/python tests/synth.py && ./.venv/bin/python tests/eval.py
```

Current results (all green):

- **Solver property test**: 60/60 random mazes — emitted commands replayed on
  the wall map reach the goal without crossing walls.
- **Motion-array test**: 360/360 emitted `&[Motion]` arrays geometrically
  simulated — every `Line` reachable from its start heading, every swept path
  ≥ 80 mm clear of the *true* walls, every final pose in the goal cell.
- **Synthetic photo test**: 30/30 rendered 9×9 "photos" (white mottled floor,
  dark leaning walls, specular streaks, cyan clips, lone post clips, floor
  seam, frame + frame seam, chamfers with coloured plates, random perspective
  into 1920×1080, sensor noise) detected with **0 wall errors** across all
  interior edges; **360/360** end-to-end paths planned on the detected map
  are valid on the true map.
- **Obstacle course test**: 10/10 synthetic courses — every cylinder found
  (±2 mm diameter, no spurious detections) and every planned trajectory
  clears the true cylinders by the robot radius.
- **Real fixed-camera photo (#140)**: registration and full wall map verified
  by eye at zoom; solved path threads the corridors correctly.
- **Operating envelope**: the strongly oblique handheld iPhone shot (#121)
  breaks the top-down lean model (walls lean ACROSS cells; face/base
  confusion) — by design out of scope, since the assessed input is the fixed
  overhead camera. If the rig ever changes, the manual corner click plus the
  review UI still allow a correct (slower) run.
- **Still to do in the lab**: capture 10+ real frames from BOTH cameras
  (possible now, #131) — synthetics calibrate structure, only real frames
  calibrate exposure/glare; then a full demo-day dress rehearsal (§6).

## 8. Risks

- **Glare on reflective acrylic** → score by dark-fraction (median-robust), not
  mean; cyan-clip evidence is glare-immune; real-frame calibration in the lab.
- **Seam/tape between floor plates** → thickness+darkness gating; white tape is
  bright so it biases *away* from wall.
- **Camera moved between calibration and demo** → corners re-clickable in
  seconds; cache keyed per camera.
- **Wrong camera on the day** → handle either: nothing in the pipeline is
  camera-specific beyond the clicked corners.
- **Waypoint frame mismatch with robot odometry** → the §4 contract fixes the
  frame at course entry; teammate resets/reads pose there.
