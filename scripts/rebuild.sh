#!/usr/bin/env bash

set -e  # stop on first error

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"

echo "========================================"
echo " PX-Swarm full rebuild"
echo "========================================"
echo "Project root: $PROJECT_ROOT"
echo "Build dir:    $BUILD_DIR"
echo

# Remove old build directory
if [ -d "$BUILD_DIR" ]; then
    echo "[1/5] Removing old build directory..."
    rm -rf "$BUILD_DIR"
else
    echo "[1/5] No previous build directory found."
fi

# Create fresh build directory
echo "[2/5] Creating new build directory..."
mkdir -p "$BUILD_DIR"

cd "$BUILD_DIR"

# Run CMake
echo "[3/5] Running CMake..."
cmake ..

# Compile
echo "[4/5] Building project..."
make -j$(nproc)

# Verify binary
echo "[5/5] Checking binary..."
if [ -f "$BUILD_DIR/px_swarm_node" ]; then
    echo
    echo "Build successful!"
    echo "Binary: $BUILD_DIR/px_swarm_node"
else
    echo
    echo "Build failed: px_swarm_node not found."
    exit 1
fi

echo "========================================"
echo " Done."
echo "========================================"
