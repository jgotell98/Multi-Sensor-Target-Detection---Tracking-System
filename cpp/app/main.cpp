#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "mst/benchmark/metrics.hpp"
#include "mst/processing/signal_processing.hpp"
#include "mst/sim/sensors.hpp"
#include "mst/sim/world.hpp"
#include "mst/tracking/multi_tracker.hpp"
#include "mst/types.hpp"
#include "mst/util/csv_writer.hpp"
#include "mst/util/thread_queue.hpp"

namespace mst {
namespace {

struct Args final {
  double duration_s = 20.0;
  double dt_s = 0.05;
  int objects = 3;
  std::uint32_t seed = 1;
  std::string out_dir = "output";
  std::string tracker = "ekf";  // kf|ekf|pf
};

static void print_help() {
  std::cout << "mst_run options:\n"
               "  --duration <sec>\n"
               "  --dt <sec>\n"
               "  --objects <n>\n"
               "  --seed <n>\n"
               "  --tracker <kf|ekf|pf>\n"
               "  --out <dir>\n";
}

static Args parse(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto require = [&](const char* name) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + name);
      return argv[++i];
    };
    if (k == "--help" || k == "-h") {
      print_help();
      std::exit(0);
    } else if (k == "--duration") {
      a.duration_s = std::stod(require("--duration"));
    } else if (k == "--dt") {
      a.dt_s = std::stod(require("--dt"));
    } else if (k == "--objects") {
      a.objects = std::stoi(require("--objects"));
    } else if (k == "--seed") {
      a.seed = static_cast<std::uint32_t>(std::stoul(require("--seed")));
    } else if (k == "--out") {
      a.out_dir = require("--out");
    } else if (k == "--tracker") {
      a.tracker = require("--tracker");
    } else {
      throw std::runtime_error("unknown arg: " + k);
    }
  }
  return a;
}

static TrackerKind tracker_kind_from(const std::string& s) {
  if (s == "kf") return TrackerKind::KF;
  if (s == "ekf") return TrackerKind::EKF;
  if (s == "pf") return TrackerKind::PF;
  throw std::runtime_error("unknown tracker: " + s);
}

}  // namespace
}  // namespace mst

int main(int argc, char** argv) try {
  using namespace mst;

  const Args args = parse(argc, argv);
  std::filesystem::create_directories(args.out_dir);

  WorldConfig wcfg;
  wcfg.objects = args.objects;
  World world(wcfg, args.seed);

  RadarConfig rcfg;
  AcousticConfig acfg;
  SensorSimulator sensors(rcfg, acfg, args.seed + 7);

  SignalProcessingConfig pcfg;
  SignalProcessor processor(pcfg);

  TrackerConfig tcfg;
  tcfg.kind = tracker_kind_from(args.tracker);
  tcfg.dt_s = args.dt_s;
  MultiTracker tracker(tcfg, args.seed + 99);

  CsvWriter truth_csv(std::filesystem::path(args.out_dir) / "truth.csv");
  CsvWriter det_csv(std::filesystem::path(args.out_dir) / "detections.csv");
  CsvWriter tracks_csv(std::filesystem::path(args.out_dir) / "tracks.csv");

  truth_csv.write_line("t_s,object_id,x,y,vx,vy");
  det_csv.write_line("t_s,sensor,kind,x,y,range_m,bearing_rad,snr");
  tracks_csv.write_line("t_s,track_id,x,y,vx,vy,class,age_s,last_update_s");

  ThreadQueue<Frame> q;
  std::vector<double> frame_ms;
  std::vector<std::vector<TruthObject>> truth_by_frame;
  std::vector<std::vector<TrackEstimate>> tracks_by_frame;

  const int steps = static_cast<int>(args.duration_s / args.dt_s);

  std::thread producer([&] {
    double t = 0.0;
    for (int k = 0; k < steps; ++k) {
      Frame f;
      f.t_s = t;
      f.truth = world.objects();
      f.radar_hits = sensors.simulate_radar(f.truth);
      f.acoustic_hits = sensors.simulate_acoustic(f.truth);
      q.push(std::move(f));

      world.step(args.dt_s);
      t += args.dt_s;

      std::this_thread::sleep_for(std::chrono::duration<double>(args.dt_s));
    }
    q.stop();
  });

  std::thread consumer([&] {
    while (true) {
      auto item = q.pop_blocking();
      if (!item.has_value()) break;
      const Frame& f = *item;

      const auto t0 = std::chrono::steady_clock::now();

      const auto radar_det = processor.detect_radar(f.radar_hits);
      const auto acoustic_det = processor.detect_acoustic(f.acoustic_hits);
      tracker.step(f.t_s, radar_det, acoustic_det);
      const auto tr = tracker.tracks();

      const auto t1 = std::chrono::steady_clock::now();
      frame_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
      truth_by_frame.push_back(f.truth);
      tracks_by_frame.push_back(tr);

      for (const auto& o : f.truth) {
        truth_csv.write_line(std::to_string(f.t_s) + "," + std::to_string(o.id) + "," + std::to_string(o.pos.x) +
                             "," + std::to_string(o.pos.y) + "," + std::to_string(o.vel.x) + "," +
                             std::to_string(o.vel.y));
      }

      for (const auto& d : radar_det) {
        const double x = d.range_m * std::cos(d.bearing_rad);
        const double y = d.range_m * std::sin(d.bearing_rad);
        det_csv.write_line(std::to_string(f.t_s) + ",radar,cluster," + std::to_string(x) + "," + std::to_string(y) +
                           "," + std::to_string(d.range_m) + "," + std::to_string(d.bearing_rad) + "," +
                           std::to_string(d.snr));
      }
      for (const auto& d : acoustic_det) {
        det_csv.write_line(std::to_string(f.t_s) + ",acoustic,cluster,0,0,0," + std::to_string(d.bearing_rad) + "," +
                           std::to_string(d.snr));
      }

      for (const auto& e : tr) {
        tracks_csv.write_line(std::to_string(f.t_s) + "," + std::to_string(e.id) + "," + std::to_string(e.x) + "," +
                              std::to_string(e.y) + "," + std::to_string(e.vx) + "," + std::to_string(e.vy) + "," +
                              to_string(e.cls) + "," + std::to_string(e.age_s) + "," +
                              std::to_string(e.last_update_s));
      }
    }
  });

  producer.join();
  consumer.join();

  const BenchStats bs = compute_bench(frame_ms, truth_by_frame, tracks_by_frame);
  CsvWriter bench_json(std::filesystem::path(args.out_dir) / "bench.json");
  bench_json.write_line("{");
  bench_json.write_line("  \"frames\": " + std::to_string(bs.frames) + ",");
  bench_json.write_line("  \"avg_frame_ms\": " + std::to_string(bs.avg_frame_ms) + ",");
  bench_json.write_line("  \"p95_frame_ms\": " + std::to_string(bs.p95_frame_ms) + ",");
  bench_json.write_line("  \"rmse_pos_m\": " + std::to_string(bs.rmse_pos_m));
  bench_json.write_line("}");

  std::cout << "Wrote output to: " << args.out_dir << "\n";
  std::cout << "Avg frame (ms): " << bs.avg_frame_ms << " | P95 (ms): " << bs.p95_frame_ms
            << " | RMSE pos (m): " << bs.rmse_pos_m << "\n";
  return 0;
} catch (const std::exception& e) {
  std::cerr << "Error: " << e.what() << "\n";
  return 1;
}

