#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

struct Vec2 {
  float x = 0.f;
  float y = 0.f;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, float s) { return {a.x * s, a.y * s}; }
inline Vec2 operator*(float s, Vec2 a) { return {a.x * s, a.y * s}; }
inline Vec2& operator+=(Vec2& a, Vec2 b) {
  a.x += b.x;
  a.y += b.y;
  return a;
}
inline Vec2& operator-=(Vec2& a, Vec2 b) {
  a.x -= b.x;
  a.y -= b.y;
  return a;
}
inline float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline float len_sq(Vec2 a) { return dot(a, a); }
inline float len(Vec2 a) { return std::sqrt(len_sq(a)); }
inline Vec2 normalize(Vec2 a) {
  float L = len(a);
  return L > 1e-8f ? Vec2{a.x / L, a.y / L} : Vec2{1.f, 0.f};
}

struct WallSeg {
  Vec2 a{};
  Vec2 b{};
};

struct Ball {
  Vec2 p{};
  Vec2 v{};
  float r = 5.f;
  float inv_mass = 1.f;
};

struct SimConfig {
  float restitution = 0.90f;
  float gravity = 640.f;
  /// Tiny air-like damping (1/s); collisions do most energy loss—keep this low for natural long bounce.
  float linear_drag_k = 0.011f;
  float fixed_dt = 1.f / 480.f;
  int solver_iterations = 6;
  int max_substeps = 32;
  float max_velocity = 5400.f;
  float bounce_threshold = 9.f;
  float rest_velocity_slop = 1.15f;
  uint32_t rng_seed = 1u;
  int ball_count = 700;
};

class Simulation {
 public:
  explicit Simulation(SimConfig cfg);

  void set_restitution(float e) { cfg_.restitution = e; }
  float restitution() const { return cfg_.restitution; }

  void step(float dt_seconds);

  const std::vector<Ball>& balls() const { return balls_; }
  const std::vector<WallSeg>& walls() const { return walls_; }
  float world_width() const { return world_w_; }
  float world_height() const { return world_h_; }

  std::string validate_report(float max_penetration, float max_speed) const;
  bool validate_ok(float max_penetration, float max_speed) const {
    return validate_report(max_penetration, max_speed).empty();
  }

 private:
  SimConfig cfg_;
  float world_w_ = 960.f;
  float world_h_ = 960.f;
  float wall_margin_ = 30.f;
  Vec2 tank_center_{};
  float tank_radius_ = 0.f;
  float cell_size_ = 22.f;

  std::vector<Ball> balls_;
  std::vector<WallSeg> walls_;

  std::vector<int> grid_head_;
  std::vector<int> grid_next_;
  int grid_nx_ = 0;
  int grid_ny_ = 0;

  uint32_t rng_;

  uint32_t rnd_u32();
  float rnd_range(float lo, float hi);

  void build_walls();
  void spawn_balls();
  void integrate(float h);
  void solve_contacts(float h);
  void positional_ball_wall();
  void positional_ball_ball(int sweeps);
  void velocity_ball_wall(float h);
  void velocity_ball_ball(float h);
  void clamp_velocities();
  void clamp_to_tank();

  void rebuild_ball_grid();
};
