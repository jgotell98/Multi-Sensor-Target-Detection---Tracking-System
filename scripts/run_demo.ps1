$ErrorActionPreference = "Stop"

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release

.\build\Release\mst_run.exe --duration 20 --objects 3 --seed 1 --tracker ekf --out output
python .\python\visualize.py --in output

