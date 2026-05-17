#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [ ! -d ".venv" ]; then
    echo "Creating Jetson Nano venv with system site packages..."
    python3 -m venv --system-site-packages .venv
fi

source .venv/bin/activate

python -m pip install --upgrade "pip<22" "setuptools<60" wheel
python -m pip install -r requirements-nano.txt

python tools/verify_environment.py
