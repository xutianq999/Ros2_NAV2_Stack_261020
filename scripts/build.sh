#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# Some packages (notably rtabmap_sync) have memory-intensive C++ translation
# units. colcon limits package-level parallelism, while Make/Ninja controls
# source-file parallelism within a package, so constrain both levels here.
# Use BUILD_JOBS=2 ./scripts/build.sh to opt into a higher limit.
BUILD_JOBS="${BUILD_JOBS:-1}"
if ! [[ "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]]; then
  echo "BUILD_JOBS must be a positive integer (got: $BUILD_JOBS)" >&2
  exit 2
fi

export CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS"
export MAKEFLAGS="-j$BUILD_JOBS"

colcon build --symlink-install \
  --executor sequential \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
