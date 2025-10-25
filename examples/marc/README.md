# MARC drawing executor

This directory contains utilities to reproduce the MARC drawing demos with a
[Lerobot SO-100 follower](https://github.com/huggingface/lerobot). The workflow
converts page-space strokes into end-effector poses solved through inverse
kinematics and streams them to the physical arm.

## 1. Prerequisites

1. Install the Python dependencies for the project and the optional kinematics
   extras (placo + OpenCV):
   ```bash
   pip install -e .[kinematics]
   pip install opencv-python pillow
   ```
2. Download the SO-100 URDF from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
   (recommended file: `Simulation/SO101/so101_new_calib.urdf`).
3. Ensure the SO-100 follower firmware is on the latest release and that you
   know the serial port that exposes the motor bus (for example `/dev/tty.usbmodemXXXX`).
4. Prepare a JSON stroke plan and a page-to-robot homography (see below).

## 2. Calibration and homography

1. Calibrate the follower using the utilities that ship with Lerobot, or allow
   the executor to run `--calibrate` on the first connection (this prompts for
   manual alignment).
2. Create a homography that maps points in page coordinates *(x, y in meters)*
   to the robot workspace. Store the 3×3 matrix as either `homography.npy`,
   an `.npz` file with a `homography` entry, or a JSON file with a
   `{"homography": [[...]]}` payload. The same file is consumed both for
   execution and for the optional correction stage.
3. (Optional) Measure the marker rack slots and store them in
   `marker_config.json`. Example:
   ```json
   {
     "black": {"pick": [0.18, -0.12], "return": [0.18, -0.12], "z": -0.015},
     "magenta": {"pick": [0.24, -0.12]}
   }
   ```

## 3. Stroke plan format

The executor consumes a JSON file that declares the page size and the strokes
for each colour. A minimal example:

```json
{
  "page": {"width": 0.210, "height": 0.297},
  "colors": [
    {
      "name": "black",
      "strokes": [
        [[0.02, 0.02], [0.05, 0.02], [0.05, 0.05]],
        [[0.06, 0.04], [0.10, 0.04]]
      ],
      "raster": "artifacts/black.png"
    }
  ]
}
```

Each stroke is an ordered list of page coordinates in metres. The optional
`raster` points to a reference image used during correction.

## 4. Running a drawing session

1. Place the page, align the camera, and insert a marker that matches the first
   colour in the plan into the gripper.
2. Execute the CLI:
   ```bash
   python -m examples.marc.run_draw_lerobot_ik \
     --plan plans/lerobot_plan.json \
     --port /dev/tty.usbmodemXXXX \
     --urdf /path/to/so101_new_calib.urdf \
     --homography calibration/homography.json \
     --page-width 0.210 --page-height 0.297 \
     --marker-config calibration/marker_config.json \
     --z-contact -0.012 --z-safe 0.08 --pitch -88
   ```
3. To run the optional closed-loop correction after each colour, add
   `--correct`. The executor will capture the page via the follower camera,
   warp it with the supplied homography, and draw residual strokes for pixels
   that differ from the provided raster.
4. Use `--calibrate` on the first run to push calibration constants to the
   motors.

## 5. Advanced options

- Tune the command rate with `--command-rate` (Hz) to match the latency of your
  serial link.
- Override travel/draw speeds or end-effector orientation with
  `--travel-speed`, `--draw-speed`, `--pick-speed`, `--pitch`, `--roll`, and
  `--yaw`.
- Supply a global raster for correction via `--target-image` if the colour
  sections do not provide their own.

## 6. Troubleshooting

- If you see "OpenCV not installed" warnings, install `opencv-python` to enable
  correction.
- IK failures typically stem from an inaccurate homography or invalid URDF
  path; double-check both and reduce `--travel-speed` to command smaller steps.
