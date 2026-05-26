/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp.h
 * @brief 通用末端执行器控制接口 (公共 API)
 *
 * 支持电动夹爪、吸盘等各种末端执行器。
 * 具体驱动通过注册机制挂载。
 */
#ifndef GRASP_H
#define GRASP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 1. Constants & Error Codes
 * ========================================================================== */

#define GRASP_OK           0
#define GRASP_ERR_ALLOC   -1
#define GRASP_ERR_CONNECT -2
#define GRASP_ERR_TIMEOUT -3
#define GRASP_ERR_CONFIG  -4
#define GRASP_ERR_PARAM   -5
#define GRASP_ERR_NOSYS   -6  /* 功能未实现 */

/* ==========================================================================
 * 2. Data Structures
 * ========================================================================== */

/** 抓取指令类型 */
typedef enum {
    GRASP_CMD_RELEASE = 0, /* 松开 / 张开 / 停止吸气 */
    GRASP_CMD_GRAB,        /* 抓紧 / 闭合 / 开启吸气 */
    GRASP_CMD_RELAX,       /* 放松 / 掉电 (省电或教学) */
} grasp_cmd_type_t;

/** 抓取器反馈状态 (核心状态机) */
typedef enum {
    GRASP_STATE_IDLE = 0, /* 空闲 (已到位或已放松) */
    GRASP_STATE_MOVING,   /* 正在动作中 */
    GRASP_STATE_HOLDING,  /* 抓住了 (电流升高 / 气压达标) */
    GRASP_STATE_EMPTY,    /* 抓空了 (完全闭合无负载) */
    GRASP_STATE_ERROR,    /* 故障 (过热、断线) */
} grasp_state_t;

/** 抓取参数配置 */
typedef struct {
    float max_effort;      /* 归一化力矩/吸力 [0.0 ~ 1.0] */
    float hold_threshold;  /* 判定 Holding 的电流/气压阈值 */
    uint32_t timeout_ms;   /* 动作超时 (ms) */
} grasp_config_t;

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

/* ==========================================================================
 * 3. Opaque Handle
 * ========================================================================== */

struct grasp_dev;

/* ==========================================================================
 * 4. API Functions
 * ========================================================================== */

/**
 * @brief 创建末端执行器实例
 * @param driver_name  驱动名 (如 "drv_uart_so101_gripper", "dummy")
 * @param args         驱动特定配置参数 (透传给驱动 factory)
 * @return 成功返回设备句柄，失败返回 NULL
 */
struct grasp_dev *grasp_alloc(const char *driver_name, void *args);

/**
 * @brief 释放末端执行器实例
 */
void grasp_free(struct grasp_dev *dev);

/* --- 核心控制 --- */

/**
 * @brief 发送简单指令 (全开/全闭/放松)
 * @param type   RELEASE / GRAB / RELAX
 * @param effort 力度 [0.0 ~ 1.0]
 */
int grasp_execute(struct grasp_dev *dev, grasp_cmd_type_t type, float effort);

/**
 * @brief 停止末端执行器 (释放扭矩 / 掉电)
 *
 * 等效于 grasp_execute(dev, GRASP_CMD_RELAX, 0.0f)。
 * 释放资源前（如 grasp_free）建议先调用此函数。
 */
void grasp_stop(struct grasp_dev *dev);

/**
 * @brief 发送精细位置指令 (可选)
 * @param position [0.0 ~ 1.0] 0.0=完全闭合, 1.0=完全打开
 * @note  仅适用于行程可控的电动夹爪
 */
int grasp_set_position(struct grasp_dev *dev, float position);

/* --- 状态反馈 --- */

/**
 * @brief 获取当前高级状态
 * @return IDLE / MOVING / HOLDING / EMPTY / ERROR
 */
grasp_state_t grasp_get_state(struct grasp_dev *dev);

/**
 * @brief 获取物理数据 (调试/高级用)
 * @param out_pos  实际位置 [0.0 ~ 1.0]
 * @param out_load 实际负载 (电流/气压)
 */
int grasp_get_feedback(struct grasp_dev *dev, float *out_pos, float *out_load);

/**
 * @brief 周期性逻辑 (高频调用)
 * 负责状态机流转：检测电流/气压 -> 判断 Holding vs Empty
 */
void grasp_tick(struct grasp_dev *dev, float dt_s);

/**
 * @brief 校准夹爪 - 学习实际硬件边界
 *
 * 交互式校准流程：
 *   1. 提示用户手动移动夹爪到完全打开位置
 *   2. 读取并保存 open_ticks
 *   3. 提示用户手动移动夹爪到完全闭合位置
 *   4. 读取并保存 closed_ticks
 *   5. 后续所有位置映射使用校准后的范围
 *
 * @param dev 设备句柄
 * @return GRASP_OK 成功，GRASP_ERR_NOSYS 驱动不支持校准
 * @note 校准前会自动执行 RELAX 命令，允许手动移动夹爪
 */
int grasp_calibrate(struct grasp_dev *dev);

#ifdef __cplusplus
}
#endif

#endif  // GRASP_H
