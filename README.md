# Multi-Sensor Target Detection & Tracking System

End-to-end simulated pipeline:

Sensor Simulator → Signal Processing → Detection → Tracking → Visualization

## Quick start

### Build (C++)

Prereqs: a C++17 compiler + CMake (and optionally Ninja).

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### Run a simulation (writes CSV + JSON)

```bash
./build/mst_run --duration 20 --objects 3 --seed 1 --tracker ekf --out output
```

On Windows you’ll typically run `build\\Release\\mst_run.exe` (depending on generator).

Outputs:
- `output/truth.csv`
- `output/detections.csv`
- `output/tracks.csv`
- `output/bench.json`

### Visualize (Python)

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r python/requirements.txt
python python/visualize.py --in output
```

Windows PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r python\requirements.txt
python python\visualize.py --in output
```

## Trackers implemented

- Kalman Filter (KF): constant-velocity model + Cartesian position measurements
- Extended Kalman Filter (EKF): constant-velocity model + radar (range/bearing) measurements
- Particle Filter (PF): constant-velocity particles + bearing-only measurements (acoustic)

Notes:
- In `ekf` mode the tracker also fuses acoustic bearing-only detections (simple gating + EKF bearing update).

## Debugging (Linux)

- gdb:
  - Build debug: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug`
  - Run: `gdb --args ./build/mst_run --duration 10 --tracker ekf --out output`
- Valgrind:
  - `valgrind --leak-check=full --track-origins=yes ./build/mst_run --duration 5 --tracker kf --out output`

## Notes

- The default simulation is 2D (x/y), with objects following constant-velocity motion plus mild process noise.
- The signal processing stage generates clustered detections from noisy sensor hits and clutter.
