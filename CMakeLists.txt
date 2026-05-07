# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com

# moteus does not use cmake to build, but does expose some client
# libraries for use in cmake projects.

cmake_minimum_required(VERSION 3.10)
project(moteus VERSION 0.1.20230821)

set(MAIN_PROJECT OFF)

# The "cpp" target exposes the C++ client side header files.
add_library(cpp INTERFACE)
add_library(moteus::cpp ALIAS cpp)

target_include_directories(cpp
  INTERFACE lib/cpp/mjbots/moteus)


# A client project can use this as follows:
#
# include(FetchContent)
# FetchContent_Declare(
#   moteus
#   GIT_REPOSITORY https://github.com/mjbots/moteus.git
#   # Pin to a specific commit for a reproducible build.  The
#   # trailing comment names the corresponding cpp/v* release tag.
#   GIT_TAG        a1b2c3d4e5f6789012345678901234567890abcd  # cpp/v1.0.0
# )
#
# FetchContent_MakeAvailable(moteus)
#
# add_executable(myproject myproject.cc)
# target_link_libraries(myproject moteus::cpp)
#
# Hashes (rather than tag names alone) are what guarantee a
# reproducible fetch — tags are mutable in principle.  See
# docs/integration/cpp.md for more.
