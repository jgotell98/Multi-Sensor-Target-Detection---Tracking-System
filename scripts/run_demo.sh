#!/usr/bin/env bash
set -euo pipefail

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

./build/mst_run --duration 20 --objects 3 --seed 1 --tracker ekf --out output
python3 python/visualize.py --in output

