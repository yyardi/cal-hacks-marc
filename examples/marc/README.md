# MARC — Marker-Actuated Robotic Controller

Prompt → PNG (Diffusers) → SVG (Potrace) → stroke plan → SO101 draw pass.

Everything inside `examples/marc/` is wired for the Lerobot SO-100/SO-101 follower with low-speed
IK, matching the joint ordering used by `teleoperate.py`. The defaults focus on a single marker so we
can get reliable drawings first before layering colour swaps.

> **Workspace limit:** The SO101 follower comfortably reaches a 173 mm × 150 mm rectangle when the
> page origin is at the lower-left of the drawing. All calibration and planning commands below use
> this area by default to avoid poses the hardware cannot hit reliably.

## Repo layout

```
vectorize/   # diffusion prompt->PNG + Potrace wrapper + SVG simplifier
planner/     # sample SVGs into ordered polylines and export JSON plans
calib/       # AprilTag/square detection, homographies, and jog-to-robot alignment
executor/    # driver API, IK streaming adapter for the SO101
run_svg.py   # prompt -> PNG + SVG helper
run_draw_lerobot_ik.py  # execute a JSON plan on the arm
out/         # artefacts (PNG, SVG, plans, calibration files) - gitignored
```

## 1. Environment setup (run once per machine)

1. **Create a virtual environment and upgrade pip**
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install --upgrade pip
   ```

2. **Install lerobot and project requirements**
   ```bash
   pip install -e .
   pip install -r examples/marc/requirements.txt
   ```
   The defaults limit travel to 2 cm/s and drawing moves to 1 cm/s to keep the IK solve rock solid.
   Start with a single colour in your plan; multi-colour grouping comes later.

3. **Install PyTorch** — follow the command suggested at <https://pytorch.org/get-started/locally/>
   for your OS, Python version, and GPU/CPU.

4. **Install Potrace**
   - macOS: `brew install potrace`
   - Windows: download the official ZIP, extract (e.g. `C:\Potrace`), add that folder to `PATH`,
     open a new PowerShell, and run `potrace --version` to confirm.

5. **Authenticate with Hugging Face** so Diffusers can download SDXL/SD 1.5 weights.
   ```bash
   export HF_TOKEN=hf_xxx_read_token  # or run `huggingface-cli login`
   ```

6. **Grab the SO101 URDF + meshes.** Every SO-ARM100/SO101 follower ships with a USB stick that contains
   the full `Simulation/SO101/` folder (URDF plus STL meshes). If you cannot find it locally, download the
   exact same payload from the manufacturer’s GitHub mirror by running:

   ```bash
   python -m examples.marc.fetch_so101_urdf
   ```

   The helper extracts `examples/marc/SO101/` with the URDF, STL meshes, textures, and verifies the URDF’s
   SHA256 hash so you know it matches the official release. Track whichever serial port appears when you
   plug the follower in (`/dev/tty.usbmodem*` on macOS and Linux). You will pass the port on the CLI; no
   source edits are required when it changes.

## 2. One-shot command sequence (prompt → draw)

Run these commands sequentially every time you want a new drawing. Replace `<slug>` with the basename
printed by `run_svg` and fill in the port/paths that match your setup. Prefer an
all-in-one helper? Skip ahead to the note after step 6 for the
`examples.marc.run_prompt_to_robot` workflow.

1. **Generate raster + SVG**
   ```bash
   python -m examples.marc.run_svg \
     "minimal line-art cat, black outline" \
     --output-dir examples/marc/out \
     --width 768 --height 768
   ```
   Outputs `examples/marc/out/<slug>.png`, `<slug>.svg`, and `<slug>-simplified.svg`.

2. **Convert SVG to a single-colour stroke plan** (dimensions in millimetres)
   ```bash
   python -m examples.marc.planner.make_plan \
     examples/marc/out/<slug>-simplified.svg \
     --output examples/marc/out/<slug>_plan.json \
     --page-width 173 --page-height 150 --unit mm --margin 5
   ```
   The planner now recentres and uniformly scales the SVG so it lands in the
   middle of the safe workspace. The CLI prints the normalised bounds and the
   percentage of sampled points that remain inside the page after scaling.
   Keep the default palette alone for now so the executor streams everything
   with one marker.

3. **Capture an overhead photo for the camera homography**
   *If you do not have a fixed camera yet, skip this step and leave out `--camera-homography` when
   drawing.*
   ```bash
   python -m examples.marc.calib.estimate_homography \
  examples/marc/paper.png \
  --mode squares \
  --square-min-area-fraction 0.00005 \
  --overlay examples/marc/out/homography_debug.png \
  --output examples/marc/out/page_homography.npz
   ```
   Tips:
   - Tape four high-contrast black squares tight to the paper corners (AprilTags stay available via
     `--mode aruco`). The detector labels the ordered corners `P0…P3` and draws the page box as a thick
     red outline so you can immediately confirm the drawing region.
   - The sample photo you shared—phone pointed straight down with four taped squares—is ideal. Keep the
     camera height fixed between capture and drawing; that geometry is baked into the homography.
   - Replace `/path/to/overhead_photo.png` with your actual capture (for example, a local
     `examples/marc/paper.png` that you do **not** commit). The script never ships a sample image
     because each rig has a different camera mounting.
   - If the CLI reports fewer than four squares, lower `--square-min-area-fraction` (e.g. `5e-5`) or
     increase lighting/contrast and rerun. The script now auto-relaxes thresholds before failing and,
     as a last resort, fits a rectangle to the largest contour so you can still inspect the overlay.

4. **Jog three calibration points so the robot knows where the page sits**
   ```bash
   python -m examples.marc.calib.page_to_robot \
     --output examples/marc/out/calib_page_to_robot.npy
   ```
   What to do in practice:
   1. Launch your teleoperation tool (the keyboard helper in `examples/lekiwi/teleoperate.py` works
      well—edit the `SO100FollowerConfig` port and URDF path before running `python -m examples.lekiwi.teleoperate`).
   2. Tape three pencil dots on the page that mark the 173 mm × 150 mm drawing rectangle: origin at the
      lower-left corner, +X along the long edge, and +Y along the short edge. Jog the pen tip onto each dot and
      note the follower’s XY readout in metres (the teleop helper prints three decimals). If you are using the Cal Hacks demo rig, you can skip
      manual entry entirely with `--use-stage-default` to reuse the baked coordinates. The stage setup maps
      `(0, 0) → (0.195, -0.084)` m, `(page_width, 0) → (0.368, -0.073)` m, and `(0, page_height) → (0.185, 0.066)` m.
      Running with `--use-stage-default` still writes `examples/marc/out/calib_page_to_robot.npy` so every
      downstream CLI can pick up the same transform automatically.
   3. Enter those three XY pairs when the calibration script prompts for origin, +X, and +Y (or rely on the
      baked defaults). The stored `.npy` file is the rigid transform from page metres into robot
      metres, solved via a two-point Procrustes fit (rotation + translation only).

5. **(Optional but recommended) Run the hardware test square**
   Before streaming a prompt-driven plan, draw a centred square to confirm the calibration envelope. The
   CLI automatically looks for `examples/marc/out/calib_page_to_robot.npy`, so you only need extra flags if
   you saved your transform elsewhere:

   ```bash
   python -m examples.marc.run_draw_test_square \
     --port /dev/tty.usbmodem12345601 \
     --urdf /absolute/path/to/so101_new_calib.urdf \
     --square-size-mm 110 --margin-mm 12 --calibrate
   ```

   On the Cal Hacks rig you can also skip file IO entirely by passing `--use-stage-default`, which injects the
   baked coordinates gathered from the stage photo:

   ```bash
   python -m examples.marc.run_draw_test_square \
     --port /dev/tty.usbmodem12345601 \
     --urdf /absolute/path/to/so101_new_calib.urdf \
     --use-stage-default \
     --square-size-mm 110 --margin-mm 12 --calibrate
   ```
   The script moves gently and leaves a square centred on the page bounds the homography identified. If
   the square is skewed or off-centre, redo the jog calibration or camera photo before attempting a full
   drawing.

   The square helper (and the full drawing CLI below) now default to storing the follower calibration under
   `~/.cache/huggingface/lerobot/calibration/robots/so100_follower/marc_so101.json`. The first time you
   connect without a saved JSON the driver will automatically run the interactive calibration routine so the
   motors know their offsets. To keep multiple rigs separate, pass `--follower-id <name>`; add
   `--follower-calibration-dir <path>` if you prefer to keep the calibration JSON alongside the repo instead of
   the global cache.

6. **Stream the plan to the arm at gentle speeds**
   ```bash
   python -m examples.marc.run_draw_lerobot_ik \
     --plan examples/marc/out/<slug>_plan.json \
     --port /dev/tty.usbmodem12345601 \
     --urdf /absolute/path/to/so101_new_calib.urdf \
     --page-width 0.173 --page-height 0.150 \
     --z-contact -0.028 --z-safe 0.05 \
     --pitch -90 --roll 0 --yaw 180 \
     --calibrate
   ```

   To collapse the entire prompt → draw workflow into one command, run:

   ```bash
   python -m examples.marc.run_prompt_to_robot \
     "minimal line-art cat, black outline" \
     --output-dir examples/marc/out \
     --port /dev/tty.usbmodem12345601 \
     --urdf /absolute/path/to/so101_new_calib.urdf \
     --page-width 173 --page-height 150 --unit mm --margin 5
  ```

  Already generated the PNG/SVG/plan trio for that slug? Add `--reuse-intermediates`
  to reuse the artefacts in `examples/marc/out` and jump straight to validation and
  streaming.

  The CLI shares the same calibration defaults as `run_draw_lerobot_ik`. If the follower offsets are missing it
  will prompt for calibration once and then reuse the saved JSON on future runs. Use `--follower-id` and
  `--follower-calibration-dir` here too if you need to manage multiple hardware setups or relocate the
  calibration files.

  Add `--skip-draw` (or omit `--port`/`--urdf`) to stop after generating the
  plan. The helper shares the same auto-scaling diagnostics as
  `examples.marc.planner.make_plan` and then streams the validated plan to the
  arm.

   The executor falls back to `examples/marc/out/calib_page_to_robot.npy` when `--homography` is omitted.
   If you need to point at a different calibration file, pass `--homography /path/to/transform.npy`. To
   reuse the baked stage measurements, add `--use-stage-default` instead of a file path.
   Swap the port for whatever your OS reports. The driver mirrors the IK routine from `teleoperate.py`
   and streams poses at 15 Hz with 2 cm/s travel and 1 cm/s draw speeds to stay well within the SO101’s
   comfortable region.

7. **Optional correction pass** — once the camera flow is solid, add
   `--correct --camera-homography examples/marc/out/page_homography.npz --target-image examples/marc/out/<slug>.png`
   to re-run a quick touch-up cycle.

## 3. Calibrating the physical paper setup

- Tape the sheet down before steps 3 and 4 so both the overhead photo and jogged points reference the
  same pose. If the page moves later, redo both calibrations.
- Mark the origin/+X/+Y dots in pencil on the sheet border. After jogging, leave them for reuse or
  erase gently.
- Keep cables clear of the camera so the homography stays valid between the photo and the draw pass.
- Inspect `homography_debug.png` and confirm the red rectangle and `P0…P3` markers align with the
  sheet before trusting the calibration.
- If you are skipping the camera entirely, you only need the three-point jog file
  (`calib_page_to_robot.npy`).

## 4. Module overview

- `vectorize.generate`: loads SDXL (falls back to SD 1.5) and renders the prompt to a PNG on the best
  available device (CUDA → MPS → CPU). Seeded generators are supported.
- `vectorize.potrace_wrap`: thresholds the PNG with OpenCV, writes a temporary PBM, and shells out to
  the Potrace binary so the output SVG respects page dimensions.
- `vectorize.simplify_svg`: removes tiny path segments and simplifies curves before planning.
- `planner.vector_planning`: samples SVG paths into polylines, orders them with a nearest-neighbour
  heuristic, and groups by colour. `planner.make_plan` converts those strokes into metre-based JSON.
- `calib.estimate_homography`: detects four high-contrast squares (or optional AprilTags) in an
  overhead image and stores the camera→page homography (with an optional debug overlay).
- `calib.page_to_robot`: fits a rigid 2D transform from three jogged page points to robot XY space.
- `executor.so100_driver`: wraps `SO100Follower`, loads the same URDF used by teleoperation, solves
  planar IK via `RobotKinematics`, and streams low-speed pose interpolations.
- `run_draw_lerobot_ik`: ties everything together, optionally docking markers (if configured) and
  running a correction pass.
- `run_draw_test_square.py`: issues a single square stroke pattern using the jog calibration to verify
  page bounds and motion limits before running a full prompt plan.

## 5. Troubleshooting

- `potrace: command not found` → ensure it is installed and on `PATH` (`potrace --version`).
- IK failure logs usually point to a bad calibration. Re-run the overhead photo and three-point jog,
  making sure the paper never moved between the two steps.
- If the follower does not move, double-check `--port` and that the URDF path is correct. The driver
  logs the initial joint snapshot on connect—verify it matches expectations.
- Speeds are conservative; only raise `--travel-speed`/`--draw-speed` once the basic flow is rock
  solid.

## 6. How the transforms work (math)

The calibration pipeline assumes the sheet of paper is planar. The overhead photo gives us four
corner pixels \((u_i, v_i)\) which we map to canonical page coordinates \((x_i, y_i)\) measured in
millimetres (e.g. \((0, 0), (215.9, 0), (215.9, 279.4), (0, 279.4))\). OpenCV solves for a
homography matrix \(H\) such that, in homogeneous coordinates,

\[
\lambda
\begin{bmatrix}
 x \\
 y \\
 1
\end{bmatrix}
 =
 H
\begin{bmatrix}
 u \\
 v \\
 1
\end{bmatrix},
\]

with \(\lambda\) being an arbitrary projective scale. Because the page is flat, the camera height is
implicitly encoded in \(H\); if the camera moves or the paper bends, the original homography no
longer matches reality and you need a new overhead photo.

The jog calibration produces a 2D rigid transform that aligns page metres with robot XY
metres. Given three non-collinear page points \(P_i\) and their measured robot coordinates
\(R_i\), we solve the classic Procrustes problem: find rotation matrix \(R\) and translation vector
\(t\) minimising \(\sum_i \|R P_i + t - R_i\|^2\). The SVD-based solution implemented in
`calib.page_to_robot.fit_rigid_transform` computes

\[
R = V U^T,\qquad t = \bar{R} - R\,\bar{P},
\]

where \(U\Sigma V^T\) is the singular value decomposition of the covariance matrix between centred
point sets and the bars denote means. The resulting homogeneous matrix is

\[
T =
\begin{bmatrix}
 R & t \\
 0 & 1
\end{bmatrix},
\]

which maps page metres into robot metres. During execution we convert a stroke point from
camera pixels → page millimetres (via \(H\)) → page metres → robot metres (via \(T\)) and then feed those XY
targets into the SO101 planar IK solver together with the requested Z heights.

The example photo above—with four taped black squares tight to the paper corners—is a good input for
the square detector. Ensure all four squares are visible, avoid glare, and keep the camera fixed at
the same height during homography capture and drawing.

