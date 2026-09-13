#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_build_dir="$(mktemp -d)"
trap 'rm -f "$test_build_dir/test_parser"; rmdir "$test_build_dir"' EXIT
"${CXX:-g++}" -std=c++17 -O1 -g -Wall -Wextra -Werror \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -Itests/stubs -I. \
  esphome/components/seplos_parser/seplos_parser.cpp tests/test_parser.cpp \
  -o "$test_build_dir/test_parser"
"$test_build_dir/test_parser"
