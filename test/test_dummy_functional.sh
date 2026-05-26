#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/control/grasp/${SROBOTIS_TEST_NAME:-grasp-dummy-functional}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/grasp_dummy_functional.log"
build_dir="$artifact_dir/build"

mkdir -p "$log_dir" "$build_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] build_dir=$build_dir"

    cmake -S "$module_root" -B "$build_dir" \
        -DGRASP_BUILD_TESTS=ON \
        -DGRASP_BUILD_HW_TEST=OFF

    cmake --build "$build_dir" -j"$(nproc)"

    "$build_dir/test_grasp"
} | tee "$log_file"

grep -q "All grasp tests PASSED" "$log_file"
