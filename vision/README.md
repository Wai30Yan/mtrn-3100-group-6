# MTRN3100 Micromouse — vision module

Off-board (laptop) computer vision for the **week-12** assessment. Takes a
photo of the maze from the overhead lab camera and produces the Rust
`&[Motion]` array the robot executes.

Scope: **§4.1.1 path generation** and **§4.2 continuous planning**.
§4.3 autonomous mapping runs onboard (`micromouse-rs/src/map.rs`) — nothing
here.

> AI assistance (assignment §5.1): written with the assistance of a
> generative AI (Anthropic Claude). Every source file carries an
> `AI ASSISTANCE` header.

---

## 1. Setup (once)

From the repo root:

```bash
cd vision && python3 -m venv .venv && ./.venv/bin/pip install -r requirements.txt
```

Everything below is run from the `vision/` directory. Substitute
`./.venv/bin/python` with plain `python` if you activate the venv instead.

## 2. Check it works — run the test suite

```bash
./.venv/bin/python tests/synth.py && ./.venv/bin/python tests/eval.py
```

`synth.py` renders fake lab photos with known walls; `eval.py` runs the whole
pipeline against them. **This is the part worth trusting**: for every maze it
geometrically simulates the emitted `&[Motion]` array and asserts that each
`Line` is reachable from the heading it starts at, that the swept path clears
every real wall and cylinder by the robot radius, and that the final pose
lands in the goal cell. Expect:

```
solver property test: 60/60 passed
summary: 30/30 images perfect, total FP=0 FN=0, end-to-end paths valid 360/360
summary: 10/10 courses perfect
```

## 3. §4.1.1 — maze photo to robot commands

On a saved photo:

```bash
./.venv/bin/python maze_solver.py test_images/maze_fixed_cam.jpg --start 2,0,S --goal 6,8
```

On the day, straight from the overhead camera (Ed #131 — plug the laptop into
the demo-desk USB):

```bash
./.venv/bin/python maze_solver.py --capture 0 --start 2,0,S --goal 6,8
```

`--start row,col,dir` (dir = N/E/S/W) and `--goal row,col` are given by the
demonstrator. Row 0 is the top of the image, zero-indexed.

**What happens:** corners are found automatically (falling back to a
4-corner click), the image is rectified, walls are detected, then a review
window opens showing the detected walls over the photo — **click any edge to
toggle it** if it is visibly wrong, then press Enter. The command array
prints and an overlay `*_overlay.png` is saved as the demonstrator evidence.

Useful flags:

| flag | what it does |
|---|---|
| `--no-ui` | headless: skip the review window (used by scripts/tests) |
| `--corners click` | force the manual 4-corner picker (staff-approved, Ed #156) |
| `--corners auto` | ignore any cached corners and re-detect |
| `--flr` | also print the old week-8 `flr` string, for eyeballing |
| `--turn-radius 0.09` | arc radius in metres for 90° turns (max = 0.09) |
| `--turn-cost 0` | optimise for fewest cells instead of fewest actions |
| `--chamfer 0` | maze has no chamfered corners |
| `--n 9` | cells per side |

Output (real example, from `test_images/maze_fixed_cam.jpg`):

```rust
// absolute world coords (m, rad); origin = power-on pose at start cell (2, 0)
// facing S, +x = that heading, +y = left
// 10 motions, 14 cells; min wall clearance 87 mm
&[
    Motion::Line { final_position: Translation2::new(0.0900, 0.0000), final_speed: TRAVEL_SPEED },
    Motion::Arc  { final_position: Translation2::new(0.1800, 0.0900), final_speed: TRAVEL_SPEED },
    ...
    Motion::Line { final_position: Translation2::new(0.7200, 1.4400), final_speed: 0.0 },
]
```

Paste it in place of the `todo!()` in
`let solution: &[Motion] = todo!();` in `micromouse-rs/src/main.rs`, then
flash.

## 4. §4.2 — obstacle course

```bash
./.venv/bin/python obstacle_planner.py photo.jpg \
    --region 2,2 --entry 4,2,E --exit 4,6 --start 4,0,E --goal 4,8
```

- `--region row,col` — NW cell of the 5×5 obstacle area (the spec lets you
  hardcode this, and staff confirmed these values may be edited on the day —
  Ed #270).
- `--entry row,col,dir` — the course cell you enter and the heading you enter
  it with. `--exit row,col` — the course cell you leave from; **which side
  the gap is on is read from the detected walls**, not guessed.
- `--start` / `--goal` — the maze start and goal, as in §4.1.1.

Emits **one** `&[Motion]` array for the whole run — start → course entry
(Arcs) → through the obstacles (Pivot + Line) → exit → goal — plus a
trajectory overlay, which is the 1-mark evidence for the occupancy map.

## 5. When it says NO PATH or UNRUNNABLE

Both tools **refuse to print an array they cannot verify**, and say why:

- *"the start and goal are in separate regions of the detected maze"* — one
  of the cells is walled off. It also prints how many cells the start can
  reach. Open the saved `overlay_FAILED.png`: if a wall is wrong, rerun
  without `--no-ui` and click that edge to toggle it. (Note the demo maze in
  `test_images/maze_fixed_cam.jpg` genuinely has a few sealed pockets, e.g.
  cell `(7,1)` — that is the maze, not a bug.)
- *"UNRUNNABLE PATH: …"* — the geometry would not execute on the robot (a
  `Line` the firmware could never complete, or the swept path clipping a
  wall). Reduce `--turn-radius` if it mentions clearance.
- *"start cell (0,0) is a blocked (chamfered corner) cell"* — the four corner
  cells are cut off by the chamfer plates; pass `--chamfer 0` if the maze you
  are looking at has none.

## 6. Before demo day

1. Go to the lab and capture **10+ frames from both cameras** (allowed since
   Ed #131) — the synthetic tests calibrate structure, only real frames
   calibrate exposure and glare.
2. Rehearse the full loop: capture → click/confirm corners → eyeball the wall
   overlay → paste into `main.rs` → flash → run. Time it.
3. Confirm `TRAVEL_SPEED` and the world-frame origin with the robot side (see
   `../VISION_SPEC.md` §4).
