cmake_minimum_required(VERSION 3.20)
project(simulate LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SIMULATE_FETCH_SDL3 "Force FetchContent for SDL3 even if a system package exists" OFF)

if(NOT SIMULATE_FETCH_SDL3)
  find_package(SDL3 3 CONFIG QUIET)
endif()

if(NOT SDL3_FOUND)
  include(FetchContent)
  set(SDL_TESTS OFF CACHE BOOL "" FORCE)
  set(SDL_SHARED ON CACHE BOOL "" FORCE)
  FetchContent_Declare(
    SDL
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-3.2.8
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(SDL)
endif()

if(NOT TARGET SDL3::SDL3)
  message(FATAL_ERROR "SDL3 not found. Install SDL3 (e.g. brew install sdl3) or configure with -DSIMULATE_FETCH_SDL3=ON.")
endif()

add_executable(simulate
  src/simulation.cpp
  src/main.cpp
)
target_include_directories(simulate PRIVATE src)
target_link_libraries(simulate PRIVATE SDL3::SDL3)
target_compile_definitions(simulate PRIVATE $<$<CONFIG:Debug>:SIMULATE_DEBUG=1>)

enable_testing()
add_test(NAME headless_smoke COMMAND simulate --headless --frames 600 --seed 1 -e 0.25 -n 700)
set_tests_properties(headless_smoke PROPERTIES TIMEOUT 180)
