#include "mst/tracking/particle_filter.hpp"

#include <cmath>

namespace mst {
namespace {

static double gaussian_pdf(double x, double std) {
  const double v = std * std;
  const double a = 1.0 / std::sqrt(2.0 * kPi * v);
  return a * std::exp(-0.5 * x * x / v);
}

}  // namespace

ParticleFilterCV::ParticleFilterCV(ParticleConfig cfg, std::uint32_t seed) : cfg_(cfg), rng_(seed) {
  p_.resize(cfg_.particles);
}

void ParticleFilterCV::init() {
  const double w = 1.0 / static_cast<double>(p_.size());
  for (auto& p : p_) {
    p.x = cfg_.init_pos_std_m * norm_(rng_);
    p.y = cfg_.init_pos_std_m * norm_(rng_);
    p.vx = cfg_.init_vel_std_mps * norm_(rng_);
    p.vy = cfg_.init_vel_std_mps * norm_(rng_);
    p.w = w;
  }
}

void ParticleFilterCV::predict(double dt_s) {
  const double astd = cfg_.process_accel_std_mps2;
  for (auto& p : p_) {
    const double ax = astd * norm_(rng_);
    const double ay = astd * norm_(rng_);
    p.vx += ax * dt_s;
    p.vy += ay * dt_s;
    p.x += p.vx * dt_s;
    p.y += p.vy * dt_s;
  }
}

void ParticleFilterCV::update_bearing(double bearing_rad) {
  double sumw = 0.0;
  for (auto& p : p_) {
    const double pred = std::atan2(p.y, p.x);
    const double err = wrap_angle_rad(bearing_rad - pred);
    const double like = gaussian_pdf(err, cfg_.meas_bearing_std_rad);
    p.w *= like + 1e-12;
    sumw += p.w;
  }

  if (sumw <= 1e-30) {
    const double w = 1.0 / static_cast<double>(p_.size());
    for (auto& p : p_) p.w = w;
    return;
  }
  for (auto& p : p_) p.w /= sumw;
}

double ParticleFilterCV::effective_n() const {
  double s = 0.0;
  for (const auto& p : p_) s += p.w * p.w;
  if (s <= 1e-30) return 0.0;
  return 1.0 / s;
}

void ParticleFilterCV::resample_if_needed() {
  if (effective_n() < 0.6 * static_cast<double>(p_.size())) systematic_resample();
}

void ParticleFilterCV::systematic_resample() {
  std::vector<Particle> out;
  out.resize(p_.size());

  const double step = 1.0 / static_cast<double>(p_.size());
  std::uniform_real_distribution<double> uni(0.0, step);
  double u = uni(rng_);

  double cdf = p_[0].w;
  std::size_t i = 0;
  for (std::size_t m = 0; m < p_.size(); ++m) {
    const double th = u + m * step;
    while (th > cdf && i + 1 < p_.size()) {
      ++i;
      cdf += p_[i].w;
    }
    out[m] = p_[i];
    out[m].w = step;
  }
  p_.swap(out);
}

TrackEstimate ParticleFilterCV::estimate(std::int32_t id, double /*t_s*/, double age_s, double last_update_s) const {
  double mx = 0.0, my = 0.0, mvx = 0.0, mvy = 0.0;
  for (const auto& p : p_) {
    mx += p.w * p.x;
    my += p.w * p.y;
    mvx += p.w * p.vx;
    mvy += p.w * p.vy;
  }

  TrackEstimate e;
  e.id = id;
  e.x = mx;
  e.y = my;
  e.vx = mvx;
  e.vy = mvy;
  e.age_s = age_s;
  e.last_update_s = last_update_s;

  const double v = std::sqrt(mvx * mvx + mvy * mvy);
  if (v < 0.6) e.cls = TrackClass::Stationary;
  else if (v < 4.0) e.cls = TrackClass::Slow;
  else e.cls = TrackClass::Fast;
  return e;
}

}  // namespace mst
