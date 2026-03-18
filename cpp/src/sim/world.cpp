#include "mst/sim/world.hpp"

#include <algorithm>
#include <cmath>

#include "mst/math/util.hpp"

namespace mst {
namespace {

static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }

}  // namespace

World::World(WorldConfig cfg, std::uint32_t seed) : cfg_(cfg), rng_(seed) {
  std::uniform_real_distribution<double> pos(-cfg_.arena_half_extent_m, cfg_.arena_half_extent_m);
  std::uniform_real_distribution<double> ang(-kPi, kPi);
  std::uniform_real_distribution<double> spd(cfg_.min_speed_mps, cfg_.max_speed_mps);

  objects_.reserve(static_cast<std::size_t>(cfg_.objects));
  for (int i = 0; i < cfg_.objects; ++i) {
    const double a = ang(rng_);
    const double s = spd(rng_);
    TruthObject o;
    o.id = i + 1;
    o.pos = {pos(rng_), pos(rng_)};
    o.vel = {s * std::cos(a), s * std::sin(a)};
    objects_.push_back(o);
  }
}

void World::step(double dt_s) {
  for (auto& o : objects_) {
    const double ax = cfg_.process_noise_accel_std_mps2 * accel_noise_(rng_);
    const double ay = cfg_.process_noise_accel_std_mps2 * accel_noise_(rng_);

    o.vel.x += ax * dt_s;
    o.vel.y += ay * dt_s;
    o.vel.x = clamp(o.vel.x, -cfg_.max_speed_mps, cfg_.max_speed_mps);
    o.vel.y = clamp(o.vel.y, -cfg_.max_speed_mps, cfg_.max_speed_mps);

    o.pos.x += o.vel.x * dt_s;
    o.pos.y += o.vel.y * dt_s;
    bounce_if_needed(o);
  }
}

void World::bounce_if_needed(TruthObject& o) {
  const double e = cfg_.arena_half_extent_m;
  if (o.pos.x > e) {
    o.pos.x = e;
    o.vel.x *= -1.0;
  } else if (o.pos.x < -e) {
    o.pos.x = -e;
    o.vel.x *= -1.0;
  }
  if (o.pos.y > e) {
    o.pos.y = e;
    o.vel.y *= -1.0;
  } else if (o.pos.y < -e) {
    o.pos.y = -e;
    o.vel.y *= -1.0;
  }
}

}  // namespace mst
