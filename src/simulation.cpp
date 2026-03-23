#include "simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>

namespace {

inline int cell_coord(float v, float cell) {
  return static_cast<int>(std::floor(v / cell));
}

}  // namespace

void Simulation::rebuild_ball_grid() {
  const int nb = static_cast<int>(balls_.size());
  if (nb <= 0) {
    return;
  }
  const int nx = std::max(4, 2 + static_cast<int>(std::ceil(world_w_ / cell_size_)));
  const int ny = std::max(4, 2 + static_cast<int>(std::ceil(world_h_ / cell_size_)));
  const int nc = nx * ny;
  grid_head_.resize(static_cast<size_t>(nc));
  grid_next_.resize(static_cast<size_t>(nb));
  grid_nx_ = nx;
  grid_ny_ = ny;
  std::fill(grid_head_.begin(), grid_head_.end(), -1);

  for (int i = 0; i < nb; ++i) {
    const auto& b = balls_[i];
    int cx = cell_coord(b.p.x, cell_size_);
    int cy = cell_coord(b.p.y, cell_size_);
    cx = std::clamp(cx, 0, nx - 1);
    cy = std::clamp(cy, 0, ny - 1);
    const int c = cy * nx + cx;
    grid_next_[static_cast<size_t>(i)] = grid_head_[static_cast<size_t>(c)];
    grid_head_[static_cast<size_t>(c)] = i;
  }
}

Simulation::Simulation(SimConfig cfg) : cfg_(cfg), rng_(cfg.rng_seed ? cfg.rng_seed : 1u) {
  build_walls();
  spawn_balls();
  for (int k = 0; k < 14; ++k) {
    positional_ball_wall();
    positional_ball_ball(2);
  }
}

uint32_t Simulation::rnd_u32() {
  uint32_t x = rng_;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  rng_ = x;
  return x;
}

float Simulation::rnd_range(float lo, float hi) {
  const float t = (rnd_u32() & 0xFFFFFF) / float(0x1000000);
  return lo + (hi - lo) * t;
}

void Simulation::build_walls() {
  walls_.clear();
  tank_center_ = {world_w_ * 0.5f, world_h_ * 0.5f};
  const float half = std::min(world_w_, world_h_) * 0.5f;
  tank_radius_ = half - wall_margin_;

  const int nseg = 64;
  const float two_pi = 6.2831853f;
  for (int i = 0; i < nseg; ++i) {
    float t0 = two_pi * static_cast<float>(i) / static_cast<float>(nseg);
    float t1 = two_pi * static_cast<float>(i + 1) / static_cast<float>(nseg);
    Vec2 a{tank_center_.x + std::cos(t0) * tank_radius_, tank_center_.y - std::sin(t0) * tank_radius_};
    Vec2 b{tank_center_.x + std::cos(t1) * tank_radius_, tank_center_.y - std::sin(t1) * tank_radius_};
    walls_.push_back({a, b});
  }
}

void Simulation::spawn_balls() {
  balls_.clear();
  rng_ = cfg_.rng_seed ? cfg_.rng_seed : 1u;

  const int n = std::max(1, cfg_.ball_count);
  const float r_lo = 6.4f;
  const float r_hi = 9.2f;
  const float pad = 4.f;
  const float r_spawn_max = tank_radius_ - r_hi - pad;
  if (r_spawn_max < r_hi + 2.f) {
    return;
  }
  const float two_pi = 6.2831853f;

  auto spawn_velocity = [&](Vec2 p) {
    Vec2 off = p - tank_center_;
    float rd = len(off);
    Vec2 e_t = rd > 1e-4f ? Vec2{-off.y, off.x} * (1.f / rd) : Vec2{1.f, 0.f};
    Vec2 e_r = rd > 1e-4f ? off * (1.f / rd) : Vec2{0.f, -1.f};
    float vt = rnd_range(-2050.f, 2050.f);
    float vr = rnd_range(-880.f, 880.f);
    return e_t * vt + e_r * vr + Vec2{rnd_range(-620.f, 620.f), rnd_range(-1050.f, -140.f)};
  };

  auto overlaps_existing = [&](Vec2 p, float rad) {
    for (const auto& e : balls_) {
      if (len_sq(p - e.p) < (rad + e.r + 2.5f) * (rad + e.r + 2.5f)) {
        return true;
      }
    }
    return false;
  };

  int placed = 0;
  int rings = 52;
  for (int ring = 0; ring < rings && placed < n; ++ring) {
    float rr = (static_cast<float>(ring + 1) / static_cast<float>(rings)) * r_spawn_max * 0.98f;
    int count = std::max(3, static_cast<int>(two_pi * rr / (2.f * r_hi + 3.f)));
    for (int k = 0; k < count && placed < n; ++k) {
      float ang = two_pi * static_cast<float>(k) / static_cast<float>(count) + rnd_range(-0.08f, 0.08f);
      Vec2 p{tank_center_.x + std::cos(ang) * rr, tank_center_.y - std::sin(ang) * rr};
      float rad = rnd_range(r_lo, r_hi);
      if (len(p - tank_center_) + rad > tank_radius_ - pad) {
        continue;
      }
      if (overlaps_existing(p, rad)) {
        continue;
      }
      Ball b{};
      b.r = rad;
      b.p = p;
      b.v = spawn_velocity(p);
      const float area = 3.14159265f * b.r * b.r;
      b.inv_mass = 1.f / std::max(0.0001f, area);
      balls_.push_back(b);
      ++placed;
    }
  }

  int tries = 0;
  while (placed < n && tries < n * 900) {
    ++tries;
    float t = rnd_range(0.f, 6.2831853f);
    float u = std::sqrt(rnd_range(0.04f, 1.f));
    float rr = u * r_spawn_max * 0.97f;
    Vec2 p{tank_center_.x + std::cos(t) * rr, tank_center_.y - std::sin(t) * rr};
    float rad = rnd_range(r_lo, r_hi);
    if (len(p - tank_center_) + rad > tank_radius_ - pad) {
      continue;
    }
    if (overlaps_existing(p, rad)) {
      continue;
    }
    Ball b{};
    b.r = rad;
    b.p = p;
    b.v = spawn_velocity(p);
    const float area = 3.14159265f * b.r * b.r;
    b.inv_mass = 1.f / std::max(0.0001f, area);
    balls_.push_back(b);
    ++placed;
  }

  float max_r = r_hi;
  for (const auto& b : balls_) {
    max_r = std::max(max_r, b.r);
  }
  cell_size_ = std::max(max_r * 2.15f, 8.f);
}

void Simulation::integrate(float h) {
  const Vec2 g{0.f, cfg_.gravity};
  const float drag = std::exp(-cfg_.linear_drag_k * h);
  for (auto& b : balls_) {
    b.v = b.v * drag;
    b.v += g * h;
    b.p += b.v * h;
  }
}

void Simulation::positional_ball_wall() {
  const float pos_percent = 1.0f;
  for (auto& b : balls_) {
    Vec2 d = b.p - tank_center_;
    float dist = len(d);
    if (dist < 1e-6f) {
      b.p.x += 0.02f;
      d = b.p - tank_center_;
      dist = len(d);
    }
    Vec2 n_out = d * (1.f / dist);
    float overlap = dist + b.r - tank_radius_;
    float slop = 0.015f * b.r;
    if (overlap <= slop) {
      continue;
    }
    float corr = (overlap - slop) * pos_percent;
    b.p -= n_out * corr;
  }
}

void Simulation::positional_ball_ball(int sweeps) {
  const float pos_percent = 1.0f;
  const int nb = static_cast<int>(balls_.size());
  if (nb <= 0 || sweeps <= 0) {
    return;
  }

  auto resolve_pair = [&](int i, int j) {
    if (j <= i) {
      return;
    }
    Ball& a = balls_[i];
    Ball& b = balls_[j];
    Vec2 delta = b.p - a.p;
    float dist2 = len_sq(delta);
    float rsum = a.r + b.r;
    if (dist2 < 1e-10f || dist2 >= rsum * rsum) {
      return;
    }
    float dist = std::sqrt(dist2);
    Vec2 n = delta * (1.f / dist);
    float overlap = rsum - dist;
    float slop = 0.002f * std::min(a.r, b.r);
    if (overlap <= slop) {
      return;
    }
    float corr = (overlap - slop) * pos_percent;
    float w = a.inv_mass + b.inv_mass;
    if (w < 1e-12f) {
      return;
    }
    float wa = b.inv_mass / w;
    float wb = a.inv_mass / w;
    a.p -= n * (corr * wa);
    b.p += n * (corr * wb);
  };

  for (int s = 0; s < sweeps; ++s) {
    rebuild_ball_grid();
    const int nx = grid_nx_;
    const int ny = grid_ny_;
    const int nc = nx * ny;
    for (int ci = 0; ci < nc; ++ci) {
      const int cx = ci % nx;
      const int cy = ci / nx;
      for (int i = grid_head_[static_cast<size_t>(ci)]; i != -1; i = grid_next_[static_cast<size_t>(i)]) {
        for (int ox = -1; ox <= 1; ++ox) {
          for (int oy = -1; oy <= 1; ++oy) {
            const int cx2 = cx + ox;
            const int cy2 = cy + oy;
            if (cx2 < 0 || cy2 < 0 || cx2 >= nx || cy2 >= ny) {
              continue;
            }
            const int cj = cy2 * nx + cx2;
            for (int j = grid_head_[static_cast<size_t>(cj)]; j != -1; j = grid_next_[static_cast<size_t>(j)]) {
              resolve_pair(i, j);
            }
          }
        }
      }
    }
  }
}

void Simulation::velocity_ball_wall(float /*h*/) {
  const float e_base = std::clamp(cfg_.restitution, 0.f, 1.f);
  for (auto& b : balls_) {
    Vec2 d = b.p - tank_center_;
    float dist = len(d);
    if (dist < 1e-6f) {
      continue;
    }
    Vec2 n_out = d * (1.f / dist);
    Vec2 closest = tank_center_ + n_out * tank_radius_;
    Vec2 dvec = b.p - closest;
    float nd = len(dvec);
    if (nd < 1e-6f) {
      continue;
    }
    Vec2 n = dvec * (1.f / nd);
    float pen = b.r - nd;
    if (pen < -0.02f * b.r) {
      continue;
    }
    float vn = dot(b.v, n);
    if (vn >= 0.f) {
      continue;
    }
    float e = e_base;
    if (vn > -cfg_.bounce_threshold) {
      e = 0.f;
    }
    if (std::abs(vn) < cfg_.rest_velocity_slop) {
      e = 0.f;
    }
    float dv = -(1.f + e) * vn;
    dv = std::clamp(dv, -6200.f, 6200.f);
    b.v += n * dv;
  }
}

void Simulation::velocity_ball_ball(float /*h*/) {
  const float e_base = std::clamp(cfg_.restitution, 0.f, 1.f);
  const int nb = static_cast<int>(balls_.size());
  if (nb <= 0) {
    return;
  }

  auto impulse_pair = [&](int i, int jidx) {
    if (jidx <= i) {
      return;
    }
    Ball& a = balls_[i];
    Ball& b = balls_[jidx];
    Vec2 delta = b.p - a.p;
    float dist2 = len_sq(delta);
    float rsum = a.r + b.r;
    if (dist2 < 1e-10f) {
      return;
    }
    float dist = std::sqrt(dist2);
    if (dist >= rsum + 0.02f * std::min(a.r, b.r)) {
      return;
    }
    Vec2 n = delta * (1.f / dist);
    float vn = dot(b.v - a.v, n);
    if (vn >= 0.f) {
      return;
    }
    float e = e_base;
    if (vn > -cfg_.bounce_threshold) {
      e = 0.f;
    }
    if (std::abs(vn) < cfg_.rest_velocity_slop) {
      e = 0.f;
    }
    float w = a.inv_mass + b.inv_mass;
    if (w < 1e-12f) {
      return;
    }
    float impulse = -(1.f + e) * vn / w;
    float max_imp = 36000.f * std::min(a.r, b.r);
    impulse = std::clamp(impulse, -max_imp, max_imp);
    a.v -= n * (impulse * a.inv_mass);
    b.v += n * (impulse * b.inv_mass);
  };

  rebuild_ball_grid();
  const int nx = grid_nx_;
  const int ny = grid_ny_;
  const int nc = nx * ny;
  for (int ci = 0; ci < nc; ++ci) {
    const int cx = ci % nx;
    const int cy = ci / nx;
    for (int i = grid_head_[static_cast<size_t>(ci)]; i != -1; i = grid_next_[static_cast<size_t>(i)]) {
      for (int ox = -1; ox <= 1; ++ox) {
        for (int oy = -1; oy <= 1; ++oy) {
          const int cx2 = cx + ox;
          const int cy2 = cy + oy;
          if (cx2 < 0 || cy2 < 0 || cx2 >= nx || cy2 >= ny) {
            continue;
          }
          const int cj = cy2 * nx + cx2;
          for (int j = grid_head_[static_cast<size_t>(cj)]; j != -1; j = grid_next_[static_cast<size_t>(j)]) {
            impulse_pair(i, j);
          }
        }
      }
    }
  }
}

void Simulation::clamp_velocities() {
  const float max2 = cfg_.max_velocity * cfg_.max_velocity;
  for (auto& b : balls_) {
    float s2 = len_sq(b.v);
    if (s2 > max2) {
      b.v = normalize(b.v) * cfg_.max_velocity;
    }
  }
}

void Simulation::clamp_to_tank() {
  for (auto& b : balls_) {
    Vec2 d = b.p - tank_center_;
    float dist = len(d);
    if (dist > tank_radius_ + b.r + 220.f) {
      if (dist > 1e-5f) {
        b.p = tank_center_ + normalize(d) * std::max(0.f, tank_radius_ - b.r - 2.f);
      }
      b.v = b.v * 0.35f;
    }
  }
}

void Simulation::solve_contacts(float h) {
  (void)h;
  for (int it = 0; it < cfg_.solver_iterations; ++it) {
    positional_ball_wall();
    positional_ball_ball(3);
    clamp_to_tank();
  }
  velocity_ball_wall(h);
  velocity_ball_ball(h);
  for (int it = 0; it < 6; ++it) {
    positional_ball_wall();
    positional_ball_ball(3);
    clamp_to_tank();
  }
}

void Simulation::step(float dt_seconds) {
  float acc = 0.f;
  acc += dt_seconds;
  int steps = 0;
  const float h = cfg_.fixed_dt;
  while (acc >= h && steps < cfg_.max_substeps) {
    integrate(h);
    solve_contacts(h);
    clamp_velocities();
    acc -= h;
    ++steps;
  }
  if (steps > 0) {
    positional_ball_wall();
    positional_ball_ball(3);
    positional_ball_wall();
    clamp_to_tank();
  }
}

std::string Simulation::validate_report(float max_penetration, float max_speed) const {
  std::ostringstream issues;
  const float max_sp2 = max_speed * max_speed;
  const float oob_slack = 22.f;

  for (size_t i = 0; i < balls_.size(); ++i) {
    const Ball& b = balls_[i];
    if (len_sq(b.v) > max_sp2) {
      issues << "speed idx=" << i << " |v|^2=" << len_sq(b.v) << "; ";
    }
    const float d_out = len(b.p - tank_center_) + b.r - tank_radius_;
    if (d_out > oob_slack) {
      issues << "oob idx=" << i << " p=(" << b.p.x << "," << b.p.y << "); ";
    }
    const float dist_c = len(b.p - tank_center_);
    const float wall_pen = dist_c + b.r - tank_radius_;
    if (wall_pen > max_penetration) {
      issues << "wall_pen idx=" << i << " pen=" << wall_pen << "; ";
    }
  }

  const int nb = static_cast<int>(balls_.size());
  for (int i = 0; i < nb; ++i) {
    for (int j = i + 1; j < nb; ++j) {
      const Ball& a = balls_[i];
      const Ball& b = balls_[j];
      float dist = len(a.p - b.p);
      float pen = (a.r + b.r) - dist;
      if (pen > max_penetration) {
        issues << "ball_pen i=" << i << " j=" << j << " pen=" << pen << "; ";
        if (issues.tellp() > 4000) {
          issues << "...";
          return issues.str();
        }
      }
    }
  }

  return issues.str();
}
