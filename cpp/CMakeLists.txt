add_library(mst_core
  src/benchmark/metrics.cpp
  src/processing/signal_processing.cpp
  src/sim/sensors.cpp
  src/sim/world.cpp
  src/tracking/ekf.cpp
  src/tracking/kalman.cpp
  src/tracking/multi_tracker.cpp
  src/tracking/particle_filter.cpp
)

target_include_directories(mst_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(MSVC)
  target_compile_options(mst_core PRIVATE /W4)
else()
  target_compile_options(mst_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(mst_run app/main.cpp)
target_link_libraries(mst_run PRIVATE mst_core)

if(UNIX AND NOT APPLE)
  target_link_libraries(mst_run PRIVATE pthread)
endif()

