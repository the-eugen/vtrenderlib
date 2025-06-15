#!/usr/bin/env bash

set -euo pipefail

POSIX_BUILD_DIR="build-posix"
WIN32_BUILD_DIR="build-win32"

echo "[+] Configuring native POSIX build in $POSIX_BUILD_DIR..."
rm -rf "$POSIX_BUILD_DIR"
mkdir -p "$POSIX_BUILD_DIR"
cmake -G "Unix Makefiles" -S . -B "$POSIX_BUILD_DIR"

echo "[+] Configuring Windows build in $WIN32_BUILD_DIR..."
rm -rf "$WIN32_BUILD_DIR"
mkdir -p "$WIN32_BUILD_DIR"
cmake -G "Unix Makefiles" -S . -B "$WIN32_BUILD_DIR" -DCMAKE_TOOLCHAIN_FILE="mingw-gcc.cmake"
