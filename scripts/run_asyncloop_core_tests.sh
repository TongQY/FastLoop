#!/usr/bin/env bash
set -euo pipefail

compiler="${CXX:-g++}"
flags=(-std=c++14 -Wall -Wextra -Wpedantic -Werror -pthread -Iinclude)
sources=(
  src/AsyncLoop/TransactionEngine.cc
  src/AsyncLoop/AsyncController.cc
  src/AsyncLoop/Pose3.cc
)
build_dir="${ASYNCLOOP_BUILD_DIR:-build-local}"

mkdir -p "${build_dir}"

"${compiler}" "${flags[@]}" \
  src/AsyncLoop/TransactionEngine.cc tests/asyncloop/TransactionEngineTests.cc \
  -o "${build_dir}/transaction_tests"
"${compiler}" "${flags[@]}" \
  "${sources[@]}" tests/asyncloop/AsyncControllerTests.cc \
  -o "${build_dir}/controller_tests"
"${compiler}" "${flags[@]}" \
  "${sources[@]}" tests/asyncloop/Pose3Tests.cc \
  -o "${build_dir}/pose_tests"

"${build_dir}/transaction_tests"
"${build_dir}/controller_tests"
"${build_dir}/pose_tests"

if [[ "${ASYNCLOOP_SANITIZE:-0}" == "1" ]]; then
  "${compiler}" "${flags[@]}" -fsanitize=address,undefined \
    -fno-omit-frame-pointer "${sources[@]}" tests/asyncloop/AsyncControllerTests.cc \
    -o "${build_dir}/controller_tests_sanitized"
  # LeakSanitizer may be unavailable in restricted/ptrace containers.
  ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}" \
    "${build_dir}/controller_tests_sanitized"
fi
