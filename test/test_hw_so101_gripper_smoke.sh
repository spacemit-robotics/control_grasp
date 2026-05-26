#!/usr/bin/env bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/control/grasp/${SROBOTIS_TEST_NAME:-grasp-so101-hardware-smoke}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/grasp_so101_hardware_smoke.log"
build_dir="$artifact_dir/build"
motor_include_dir="$module_root/../../peripherals/motor/include"
port="${GRASP_SO101_PORT:-/dev/ttyACM0}"
timeout_s="${GRASP_SO101_SMOKE_TIMEOUT_S:-15}"

mkdir -p "$log_dir" "$build_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] build_dir=$build_dir"
    echo "[info] motor_include_dir=$motor_include_dir"
    echo "[info] port=$port"
    echo "[info] timeout_s=$timeout_s"

    test -e "$port"
    test -f "$motor_include_dir/motor.h"

    cmake -S "$module_root" -B "$build_dir" \
        -DMOTOR_INCLUDE_PATH="$motor_include_dir" \
        -DGRASP_BUILD_TESTS=OFF \
        -DGRASP_BUILD_HW_TEST=ON

    cmake --build "$build_dir" -j"$(nproc)"

    # The hardware test is menu-driven today. Send the explicit exit option and
    # bound runtime so CI/manual automation cannot hang indefinitely.
    printf '0\n' | timeout "$timeout_s" "$build_dir/test_hw_so101_gripper" --port "$port"
} | tee "$log_file"

grep -q "test_hw_so101_gripper" "$log_file"
