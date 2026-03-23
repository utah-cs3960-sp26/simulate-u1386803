#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "simulation.hpp"

namespace {

struct Args {
  bool headless = false;
  int frames = 600;
  uint32_t seed = 1u;
  float restitution = 0.90f;
  int ball_count = 700;
  bool help = false;
};

float parse_float(const char* s, float def) {
  if (!s || !*s) {
    return def;
  }
  char* end = nullptr;
  float v = std::strtof(s, &end);
  if (end == s) {
    return def;
  }
  return v;
}

int parse_int(const char* s, int def) {
  if (!s || !*s) {
    return def;
  }
  char* end = nullptr;
  long v = std::strtol(s, &end, 10);
  if (end == s) {
    return def;
  }
  return static_cast<int>(v);
}

Args parse_args(int argc, char** argv) {
  Args a;
  if (const char* env = std::getenv("RESTITUTION")) {
    a.restitution = parse_float(env, a.restitution);
  }
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--headless")) {
      a.headless = true;
    } else if (!std::strcmp(argv[i], "--help") || !std::strcmp(argv[i], "-h")) {
      a.help = true;
    } else if (!std::strcmp(argv[i], "--frames") && i + 1 < argc) {
      a.frames = parse_int(argv[++i], a.frames);
    } else if (!std::strcmp(argv[i], "--seed") && i + 1 < argc) {
      a.seed = static_cast<uint32_t>(parse_int(argv[++i], static_cast<int>(a.seed)));
    } else if ((!std::strcmp(argv[i], "-e") || !std::strcmp(argv[i], "--restitution")) && i + 1 < argc) {
      a.restitution = parse_float(argv[++i], a.restitution);
    } else if ((!std::strcmp(argv[i], "-n") || !std::strcmp(argv[i], "--balls")) && i + 1 < argc) {
      a.ball_count = parse_int(argv[++i], a.ball_count);
    }
  }
  return a;
}

void print_help() {
  SDL_Log(
      "simulate [--headless] [--frames N] [--seed S] [-e|--restitution E] [-n|--balls N]\n"
      "  RESTITUTION env overrides default if flag not passed.\n");
}

void draw_walls(SDL_Renderer* r, const Simulation& sim) {
  SDL_SetRenderDrawColor(r, 200, 200, 210, 255);
  for (const auto& w : sim.walls()) {
    SDL_RenderLine(r, w.a.x, w.a.y, w.b.x, w.b.y);
  }
}

void draw_ball_fan(SDL_Renderer* r, const Ball& b, SDL_Color fill) {
  constexpr int segs = 8;
  SDL_Vertex vx[static_cast<size_t>(segs) + 2];
  const SDL_FColor fc{fill.r / 255.f, fill.g / 255.f, fill.b / 255.f, fill.a / 255.f};
  vx[0].position.x = b.p.x;
  vx[0].position.y = b.p.y;
  vx[0].color = fc;
  vx[0].tex_coord = {0.f, 0.f};

  for (int k = 0; k <= segs; ++k) {
    float t = (static_cast<float>(k) / static_cast<float>(segs)) * 6.2831853f;
    vx[static_cast<size_t>(k) + 1].position.x = b.p.x + std::cos(t) * b.r;
    vx[static_cast<size_t>(k) + 1].position.y = b.p.y + std::sin(t) * b.r;
    vx[static_cast<size_t>(k) + 1].color = fc;
    vx[static_cast<size_t>(k) + 1].tex_coord = {0.f, 0.f};
  }

  static const int idx[segs * 3] = {0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5,
                                      0, 5, 6, 0, 6, 7, 0, 7, 8, 0, 8, 9};
  SDL_RenderGeometry(r, nullptr, vx, segs + 2, idx, segs * 3);
}

int run_headless(const Args& a) {
  SimConfig cfg;
  cfg.rng_seed = a.seed;
  cfg.restitution = a.restitution;
  cfg.ball_count = std::max(1, a.ball_count);
  Simulation sim(cfg);
  sim.set_restitution(a.restitution);

  if (a.frames <= 0) {
    const std::string rep = sim.validate_report(0.08f, 1.e9f);
    if (!rep.empty()) {
      SDL_Log("SPAWN VALIDATION: %s", rep.c_str());
      return 1;
    }
    return 0;
  }

  const float dt = 1.f / 60.f;
  for (int f = 0; f < a.frames; ++f) {
    sim.step(dt);
  }

  const std::string rep = sim.validate_report(1.25f, 6000.f);
  if (!rep.empty()) {
    SDL_Log("VALIDATION FAIL: %s", rep.c_str());
    return 1;
  }
  return 0;
}

int run_visual(const Args& a) {
  SimConfig cfg;
  cfg.rng_seed = a.seed;
  cfg.restitution = a.restitution;
  cfg.ball_count = std::max(1, a.ball_count);
  Simulation sim(cfg);
  sim.set_restitution(a.restitution);

  SDL_Window* window = nullptr;
  SDL_Renderer* renderer = nullptr;
  SDL_SetHint(SDL_HINT_RENDER_VSYNC, "0");
  if (!SDL_CreateWindowAndRenderer("simulate-u1386803", static_cast<int>(sim.world_width()),
                                    static_cast<int>(sim.world_height()), 0, &window, &renderer)) {
    SDL_Log("CreateWindowAndRenderer failed: %s", SDL_GetError());
    return 1;
  }
  SDL_SetRenderVSync(renderer, 0);

  bool running = true;
  Uint64 prev = SDL_GetPerformanceCounter();
  Uint64 freq = SDL_GetPerformanceFrequency();
  float fps_smooth = 0.f;

  while (running) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_EVENT_QUIT) {
        running = false;
      }
      if (e.type == SDL_EVENT_KEY_DOWN && e.key.key == SDLK_ESCAPE) {
        running = false;
      }
    }

    Uint64 now = SDL_GetPerformanceCounter();
    float dt = static_cast<float>(now - prev) / static_cast<float>(freq);
    prev = now;
    dt = std::min(dt, 0.05f);

    sim.step(dt);

    SDL_SetRenderDrawColor(renderer, 24, 26, 32, 255);
    SDL_RenderClear(renderer);
    draw_walls(renderer, sim);

    const SDL_Color ball_green{40, 190, 95, 255};
    for (const auto& b : sim.balls()) {
      draw_ball_fan(renderer, b, ball_green);
    }

    SDL_RenderPresent(renderer);

    float inst_fps = dt > 1e-6f ? 1.f / dt : 0.f;
    fps_smooth = fps_smooth * 0.9f + inst_fps * 0.1f;
    char title[128];
    std::snprintf(title, sizeof(title), "simulate-u1386803 | balls=%zu | fps=%.0f | e=%.2f",
                  sim.balls().size(), static_cast<double>(fps_smooth),
                  static_cast<double>(sim.restitution()));
    SDL_SetWindowTitle(window, title);
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Args a = parse_args(argc, argv);
  if (a.help) {
    print_help();
    return 0;
  }

  if (a.headless) {
    if (!SDL_Init(0)) {
      SDL_Log("SDL_Init failed: %s", SDL_GetError());
      return 1;
    }
    int code = run_headless(a);
    SDL_Quit();
    return code;
  }

  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("SDL_Init failed: %s", SDL_GetError());
    return 1;
  }
  int code = run_visual(a);
  SDL_Quit();
  return code;
}
