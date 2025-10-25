# MARC — Marker-Actuated Robotic Controller

Prompt → PNG (Diffusers) → SVG (Potrace) → stroke plan → SO101 draw pass.

Everything inside `examples/marc/` is wired for the Lerobot SO-100/SO-101 follower with low-speed
IK, matching the joint ordering used by `teleoperate.py`. The defaults focus on a single marker so we
can get reliable drawings first before layering colour swaps.

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
   The defaults limit travel to 4 cm/s and drawing moves to 2 cm/s to keep the IK solve stable.
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

6. **Grab the SO101 URDF** that ships with the arm (`Simulation/SO101/so101_new_calib.urdf`) and keep
   track of whichever serial port appears when you plug the follower in (`/dev/tty.usbmodem*` on macOS
   and Linux). You will pass the port on the CLI; no source edits are required when it changes.

## 2. One-shot command sequence (prompt → draw)

Run these commands sequentially every time you want a new drawing. Replace `<slug>` with the basename
printed by `run_svg` and fill in the port/paths that match your setup.

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
     --page-width 215.9 --page-height 279.4 --unit mm
   ```
   The resulting JSON stores the page size in metres and an ordered list of strokes. Keep the default
   palette alone for now so the executor streams everything with one marker.

3. **(Optional) Capture an overhead photo for the camera homography**
   *If you do not have a fixed camera yet, skip this step and leave out `--camera-homography` when
   drawing.*
   ```bash
   python -m examples.marc.calib.estimate_homography \
     /path/to/overhead_photo.png \
     --output examples/marc/out/page_homography.npz \
     --overlay examples/marc/out/homography_debug.png
   ```
   Tips:
   - Tape four high-contrast black squares tight to the paper corners (AprilTags stay available via
     `--mode aruco`). The detector labels the ordered corners `P0…P3` in the overlay so you can see the
     box that will become the drawing canvas.
   - The sample photo you shared—phone pointed straight down with four taped squares—is ideal. Keep the
     camera height fixed between capture and drawing; that geometry is baked into the homography.
   - If the CLI reports fewer than four squares, lower `--square-min-area-fraction` (e.g. `5e-5`) or
     increase lighting/contrast and rerun. The script now auto-relaxes thresholds before failing.

4. **Jog three calibration points so the robot knows where the page sits**
   ```bash
   python -m examples.marc.calib.page_to_robot \
     --output examples/marc/out/calib_page_to_robot.npy
   ```
   What to do in practice:
   1. Launch your teleoperation tool (for example `python -m examples.phone_to_so100.teleoperate`) after
      editing the script’s `SO100FollowerConfig` port and URDF path to match your setup.
   2. Tape three pencil dots on the page: origin (lower-left), +X (to the right), and +Y (toward the top).
      Jog the pen tip until it just touches each dot, noting the live follower XY readout (millimetres) in
      the teleop console.
   3. Enter those three XY pairs when the calibration script prompts for origin, +X, and +Y. The stored
      `.npy` file is the rigid transform from page millimetres into robot millimetres.

5. **(Optional but recommended) Run the hardware test square**
   Before streaming a prompt-driven plan, draw a centred square to confirm the calibration envelope:
   ```bash
   python -m examples.marc.run_draw_test_square \
     --port /dev/tty.usbmodem12345601 \
     --urdf /absolute/path/to/so101_new_calib.urdf \
     --page-to-robot examples/marc/out/calib_page_to_robot.npy \
     --page-width-mm 215.9 --page-height-mm 279.4 \
     --square-size-mm 120 --margin-mm 25 --calibrate
   ```
   The script moves gently and leaves a square centred on the page bounds the homography identified. If
   the square is skewed or off-centre, redo the jog calibration or camera photo before attempting a full
   drawing.

6. **Stream the plan to the arm at gentle speeds**
   ```bash
   python -m examples.marc.run_draw_lerobot_ik \
     --plan examples/marc/out/<slug>_plan.json \
     --port /dev/tty.usbmodem12345601 \
     --urdf /absolute/path/to/so101_new_calib.urdf \
     --homography examples/marc/out/calib_page_to_robot.npy \
     --page-width 0.2159 --page-height 0.2794 \
     --z-contact -0.012 --z-safe 0.08 \
     --pitch -90 --roll 0 --yaw 180 \
     --calibrate
   ```
   Swap the port for whatever your OS reports. The driver mirrors the IK routine from `teleoperate.py`
   and streams poses at 15 Hz with 4 cm/s travel and 2 cm/s draw speeds to stay well within the SO101’s
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
- Inspect `homography_debug.png` and confirm the blue rectangle and `P0…P3` markers align with the
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

The jog calibration produces a 2D rigid transform that aligns page millimetres with robot XY
millimetres. Given three non-collinear page points \(P_i\) and their measured robot coordinates
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

which maps page millimetres into robot millimetres. During execution we convert a stroke point from
camera pixels → page millimetres (via \(H\)) → robot millimetres (via \(T\)) and then feed those XY
targets into the SO101 planar IK solver together with the requested Z heights.

The example photo above—with four taped black squares tight to the paper corners—is a good input for
the square detector. Ensure all four squares are visible, avoid glare, and keep the camera fixed at
the same height during homography capture and drawing.

