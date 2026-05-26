#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/control/grasp/${SROBOTIS_TEST_NAME:-grasp-dummy-performance}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/grasp_dummy_performance.log"
build_dir="$artifact_dir/build"
bench_src="$script_dir/benchmark_dummy_grasp.c"
bench_bin="$build_dir/benchmark_dummy_grasp"
bench_log="$log_dir/benchmark_dummy_grasp_build.log"
max_avg_us="${GRASP_DUMMY_MAX_AVG_US:-200}"
iters="${GRASP_DUMMY_ITERS:-20000}"

mkdir -p "$log_dir" "$build_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] build_dir=$build_dir"
    echo "[info] GRASP_DUMMY_ITERS=$iters"
    echo "[info] GRASP_DUMMY_MAX_AVG_US=$max_avg_us"

    cmake -S "$module_root" -B "$build_dir" \
        -DGRASP_BUILD_TESTS=OFF \
        -DGRASP_BUILD_HW_TEST=OFF

    cmake --build "$build_dir" -j"$(nproc)"

    c++ -x c -std=c99 -O2 "$bench_src" -I"$module_root/include" -L"$build_dir" \
        -Wl,-rpath,"$build_dir" -lgrasp -lpthread -lm -o "$bench_bin" \
        >"$bench_log" 2>&1

    echo "[info] benchmark build log: $bench_log"
    cat "$bench_log"

    "$bench_bin" "$iters" "$max_avg_us"
} | tee "$log_file"

grep -q "PERF_OK" "$log_file"
