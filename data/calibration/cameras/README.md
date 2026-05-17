# Camera Calibration Files

This folder contains camera calibration artifacts copied into the Jetson repo.

- `overhead_calibration.npz` and `overhead_calibration.json` are for the fixed overhead camera used by board detection.
- `wrist_calibration.npz` and `wrist_calibration.json` are stored for future wrist-camera work only.

The board detection camera path auto-discovers only overhead-named calibration files unless an explicit `--calib` path is provided.
Do not hardcode laptop paths such as `/home/precious/lens_calib` in runtime code.
