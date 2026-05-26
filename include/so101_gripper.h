/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file so101_gripper.h
 * @brief SO-101 夹爪专用配置接口
 */
#ifndef SO101_GRIPPER_H
#define SO101_GRIPPER_H

#include <stdint.h>

#include "grasp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SO-101 夹爪配置参数
 *
 * 通过 grasp_alloc("so101_gripper", &cfg) 的 args 参数传入。
 * 如果 args 为 NULL，驱动会使用默认值。
 */
struct so101_gripper_config {
    const char *uart_path;
    uint32_t baud;
    uint8_t id;
    grasp_config_t grasp_cfg;
};

#ifdef __cplusplus
}
#endif

#endif  // SO101_GRIPPER_H