# Board Calibration

Board calibration can be done either on the Jetson Nano or offline on a laptop
from a copied overhead image. The offline path is useful when xRDP or VNC is too
laggy for precise OpenCV clicks.

## Offline Laptop Workflow

From the laptop, copy the latest undistorted overhead image from the Nano:

```bash
scp nanochess:/data/chess_robot/data/snapshots/latest_undistorted.png .
```

Run the calibration tool locally from a checkout of this repository:

```bash
python tools/calibrate_board.py \
  --image latest_undistorted.png \
  --output board_profile.yaml \
  --debug-dir board_calibration_debug \
  --mode manual-9x9 \
  --display-scale 1.5
```

Use `--mode four-corner` for a faster approximate grid, or `--mode manual-9x9`
for the most precise grid. Increase `--display-scale` when the image appears too
small for accurate clicking, or use a value below `1.0` if the image is larger
than the laptop display.

Inspect these generated overlays locally before copying the result back:

```text
board_calibration_debug/board_grid_overlay.png
board_calibration_debug/board_labels_overlay.png
board_calibration_debug/occupancy_crop_overlay.png
```

Copy the board profile and overlays back to the Nano:

```bash
scp board_profile.yaml nanochess:/data/chess_robot/data/calibration/board/board_profile.yaml
scp board_calibration_debug/*.png nanochess:/data/chess_robot/data/debug/
```

## OpenCV Controls

During point collection:

```text
left click        add the expected point
u or Backspace    undo the last point
r                 reset all points for the current step
q or Esc          quit without writing calibration output
```

For optional seam selection, press `Enter` after at least 3 points or press `s`
to skip seam selection.

## Point Order

Four-corner mode expects:

```text
1. top-left
2. top-right
3. bottom-right
4. bottom-left
```

Manual 9x9 mode expects all 81 grid intersections in row-major order:

```text
row 0 col 0, row 0 col 1, ... row 0 col 8,
row 1 col 0, row 1 col 1, ... row 8 col 8
```

The board orientation remains robot-black: top-left is `h1`, top-right is `a1`,
bottom-left is `h8`, and bottom-right is `a8`.

## Overhead Camera Runtime Settings

The selected overhead camera V4L2 settings are runtime device settings. They may
reset after a reboot, USB reconnect, or camera re-enumeration.

The project applies the configured overhead camera controls before capture. The
current validated setting uses auto exposure, auto white balance, and
`backlight_compensation=32` on the stable overhead device path. If these camera
settings are changed, recapture `data/snapshots/empty_board_undistorted.png`
before using empty-reference occupancy detection again.
