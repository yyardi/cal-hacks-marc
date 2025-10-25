# MARC drawing executor

This directory contains the full "prompt → SVG → stroke plan → SO101 draw" toolchain used by the
MARC (Marker-Actuated Robotic Controller) demos. Everything here targets the Lerobot SO-100/SO-101
follower running in degrees mode and streams low-speed IK trajectories to the arm.

## 1. Environment and assets

Run these once on the machine that will drive the robot:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .[kinematics]
pip install opencv-python pillow diffusers[torch]
export HF_TOKEN=...  # or run `huggingface-cli login`
```

Download the SO101 URDF that ships with the arm (`Simulation/SO101/so101_new_calib.urdf`) and note
the serial port that appears when you plug in the follower (for example
`/dev/tty.usbmodem12345601`). The port is passed at runtime, so you do **not** hard-code it into the
codebase.

## 2. End-to-end workflow

Follow these steps every time you want to draw a new prompt. Replace `<slug>` with the slug printed
by the SVG generator and update the port/paths to match your machine.

1. **Generate raster + vector art**
   ```bash
   python -m examples.marc.run_svg \
     "minimal line-art cat, black outline" \
     --output-dir examples/marc/out \
     --page-width 215.9 --page-height 279.4
   ```
   This produces `examples/marc/out/<slug>.png`, `<slug>.svg`, and `<slug>-simplified.svg`.

2. **Convert the SVG into a robot plan (meters in JSON)**
   ```bash
   python -m examples.marc.planner.make_plan \
     examples/marc/out/<slug>-simplified.svg \
     --output examples/marc/out/<slug>_plan.json \
     --page-width 215.9 --page-height 279.4 --unit mm
   ```

3. **Capture the page homography from the overhead camera**
   - Print four high-contrast AprilTag 36h11 markers (IDs 0–3 work well) and tape them to the page
     corners.
   - Lock exposure/white-balance on the camera and take a top-down photo with the paper flattened.
   - Run:
     ```bash
     python -m examples.marc.calib.estimate_homography \
       /path/to/photo.png \
       --output examples/marc/out/page_homography.npz \
       --overlay examples/marc/out/homography_debug.png
     ```
     Inspect the overlay to confirm the corner order is correct.

4. **Jog the robot to align page space with robot XY**
   ```bash
   python -m examples.marc.calib.page_to_robot \
     --output examples/marc/out/calib_page_to_robot.npy
   ```
   The script prompts for three points: the page origin, the +X point (along the page width), and the
   +Y point (along the page height). Jog the SO101 using Lerobot teleop, read the follower display for
   the XY coordinates, and type them in.

5. **Stream the drawing at safe speeds**
   ```bash
   python -m examples.marc.run_draw_lerobot_ik \
     --plan examples/marc/out/<slug>_plan.json \
     --port /dev/tty.usbmodem12345601 \
     --urdf /absolute/path/to/so101_new_calib.urdf \
     --homography examples/marc/out/calib_page_to_robot.npy \
     --camera-homography examples/marc/out/page_homography.npz \
     --page-width 0.2159 --page-height 0.2794 \
     --z-contact -0.012 --z-safe 0.08 --pitch -88 \
     --calibrate
   ```
   The defaults limit travel to 4 cm/s and drawing moves to 2 cm/s to keep the IK solve stable.
   Start with a single colour in your plan; multi-colour grouping comes later.

6. **Optional: closed-loop correction after a pass**
   Add `--correct --target-image examples/marc/out/<slug>.png` to step 5 once the camera feed is
   confirmed. The driver warps the captured page with the stored homography and issues small residual
   strokes where the edges differ.

## 3. Understanding the calibration artefacts

- `examples/marc/out/page_homography.npz` stores the camera→page homography computed from the photo.
  It keeps the page size in millimetres, so re-run the command if you switch to A4.
- `examples/marc/out/calib_page_to_robot.npy` contains the 3×3 homogeneous transform that maps
  page millimetres into robot XY millimetres. This is loaded by the driver before each move.
- If you move the page relative to the arm, repeat steps 3 and 4 before streaming a new plan.

## 4. Marker rack presets (optional)

If you have a fixed rack, create `examples/marc/out/marker_config.json`:

```json
{
  "black": {"pick": [0.18, -0.12], "return": [0.18, -0.12], "z": -0.015},
  "blue": {"pick": [0.24, -0.12]}
}
```

Pass the file with `--marker-config` so the driver knows where to dock the pen between colour passes.

## 5. Troubleshooting

- If the robot refuses to move, double-check the serial port and that the URDF path points to the
  SO101 calibration file.
- IK failures usually indicate an inaccurate homography or page-to-robot transform. Re-take the photo,
  confirm the overlay looks correct, and rerun the three-point jog routine.
- Speeds are intentionally conservative. Increase `--travel-speed` and `--draw-speed` gradually (still
  in metres per second) once everything is reliable.
