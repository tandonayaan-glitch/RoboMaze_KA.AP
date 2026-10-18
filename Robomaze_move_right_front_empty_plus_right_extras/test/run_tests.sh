#!/usr/bin/env bash
# Builds and runs the host unit tests. Needs any g++ (C++11+) on PATH.
set -e
cd "$(dirname "$0")"
g++ -std=c++11 -O1 -Wall -Wextra -Werror -o test_nav.exe test_nav.cpp
./test_nav.exe
