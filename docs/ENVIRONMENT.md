# Environment

## Jetson Nano runtime

The Jetson Nano runtime uses Python 3.6 under JetPack/Ubuntu. Do not modernise this environment unless the target hardware and JetPack version change.

Create the project virtual environment on the Nano with system site packages enabled:

```bash
python3 -m venv --system-site-packages .venv
```

This is intentional. Jetson OpenCV is provided by the system Python packages, and the virtual environment must be able to import those packages. In particular, `cv2` should come from the Jetson system package, not from PyPI.

Do not install `opencv-python` with pip on the Nano. Pip OpenCV wheels are not the correct runtime dependency for this JetPack 4.x target and can break or bypass the system OpenCV build that is already matched to the device.

Use `requirements-nano.txt` for the Nano runtime:

```bash
source .venv/bin/activate
python -m pip install -r requirements-nano.txt
```

The generic `requirements.txt` may become useful for development or non-Nano environments later, but the Nano runtime should use `requirements-nano.txt`.

`python-chess==0.31.2` is pinned because current `chess` / `python-chess` package resolution does not resolve cleanly on Python 3.6. The package name in `requirements-nano.txt` is `python-chess`, but the import name remains:

```python
import chess
```

System packages made visible through `--system-site-packages` include the verified Nano imports for `cv2`, `numpy`, and `matplotlib`. Do not add `numpy`, `matplotlib`, or OpenCV to `requirements-nano.txt` unless there is a measured Nano-specific reason.

## Verification

After activating the virtual environment, verify the runtime without touching hardware:

```bash
source .venv/bin/activate
python tools/verify_environment.py
```

The verification script imports the required Python modules, prints their versions, creates a `chess.Board()`, and checks that `e2e4` is legal from the starting position.
