#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "${repo_root}"
mkdir -p build

emcc_build() {
  local ojph_disable_simd=$1
  local build_type=$2
  cd "${repo_root}/build"
  emcmake cmake .. --fresh \
    -DOJPH_DISABLE_SIMD="${ojph_disable_simd}" \
    -DCMAKE_BUILD_TYPE="${build_type}"
  cmake --build . --config "${build_type}" --clean-first
  cd "${repo_root}"
}

if ! command -v emcmake >/dev/null 2>&1; then
  echo "emcmake not found. Activate emsdk (e.g. source emsdk_env.sh) first." >&2
  exit 1
fi

if [[ "${1:-}" != "" ]]; then
  if [[ "${2:-}" == "" ]]; then
    echo "usage: $0 [<OJPH_DISABLE_SIMD ON|OFF> <CMAKE_BUILD_TYPE>]" >&2
    exit 1
  fi
  emcc_build "$1" "$2"
  exit 0
fi

emcc_build ON Debug
emcc_build ON Release
emcc_build OFF Debug
emcc_build OFF Release
