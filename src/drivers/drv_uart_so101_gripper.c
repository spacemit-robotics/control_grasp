/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file drv_uart_so101_gripper.c
 * @brief SO-101 gripper driver using Feetech SMS/STS motor over UART.
 *
 * Single motor (default ID 6) for opening/closing gripper.
 *
 * Position mapping (based on actual hardware measurement):
 *   - Closed: position 0.0 -> ticks 2031
 *   - Open:   position 1.0 -> ticks 3468
 *   - Linear mapping: ticks = 2031 + position * 1437
 *
 * Note: Unlike arm joints which use homing_offset calibration,
 *       gripper uses direct linear mapping based on measured range.
 */
#ifdef HAVE_MOTOR

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "motor.h"          // NOLINT
#include "sts3215_regs.h"   // NOLINT
#include "../grasp_core.h"

/* --- Register access helpers (wrappers for motor_set/get_paras) --- */

static inline int reg_write_byte(struct motor_dev *dev,
                                    uint8_t reg, uint8_t val) {
    return motor_set_paras(dev, &reg, &val, 1);
}

static inline int reg_write_word(struct motor_dev *dev,
                                    uint8_t reg, uint16_t val) {
    return motor_set_paras(dev, &reg, &val, 2);
}

/* 默认夹爪运动速度，Feetech 舵机速度单位 (范围 0-2400) */
#define SO101_GRIPPER_DEFAULT_VEL 600

/* 校准数据配置文件路径 */
#define CALIBRATION_FILE "./config/so101_gripper_calibration.json"

/* ==========================================================================
 * Configuration (passed via grasp_alloc args)
 * ========================================================================== */  // NOLINT

/**
 * @brief SO-101 夹爪配置参数
 *
 * 通过 grasp_alloc 的 args 参数传入。
 * 如果 args 为 NULL，使用默认值 (ttyACM0, 1Mbaud, ID=6)。
 *
 * 用法:
 *   struct so101_gripper_config cfg = {
 *       .uart_path = "/dev/ttyACM0",  // 串口设备路径
 *       .baud = 1000000,              // 波特率
 *       .id = 6,                       // 夹爪舵机 ID
 *       .grasp_cfg = { .max_effort = 0.8, .hold_threshold = 100, .timeout_ms = 3000 },  // NOLINT
 *   };
 *   struct grasp_dev *gripper = grasp_alloc("so101_gripper", &cfg);
 */
struct so101_gripper_config {
    const char *uart_path;        /* 串口设备路径，如 "/dev/ttyACM0" */
    uint32_t baud;                /* 波特率，如 1000000 */
    uint8_t id;                   /* 夹爪舵机 ID，默认 6 */
    grasp_config_t grasp_cfg;     /* 抓取参数 (力度阈值、超时等)，可选覆盖 */  // NOLINT
};

/* ==========================================================================
 * Private Data
 * ========================================================================== */  // NOLINT

/**
 * @brief SO-101 夹爪驱动私有数据
 *
 * 只有 1 个 motor_dev，对应夹爪的单个舵机。
 */
struct so101_gripper_priv {
    struct motor_dev *motor;  /* 夹爪电机句柄 */
    uint8_t motor_id;         /* 舵机 ID (用于保存/加载校准数据) */
    bool has_target;          /* 是否有目标命令 */
    struct motor_cmd target_cmd; /* 目标命令 (用于持续发送) */
    float target_position;    /* 目标位置 [0.0=闭合, 1.0=打开]，用于判断是 GRAB 还是 RELEASE */  // NOLINT

    /* 状态确认机制 */
    float state_confirm_ms;   /* 状态确认计时器（ms），用于等待状态稳定 */  // NOLINT

    /* 位置稳定检测 */
    float last_position;      /* 上次检测的位置 */
    float stable_time_ms;     /* 位置稳定持续时间（ms） */

    /* 校准数据 */
    bool calibrated;          /* 是否已校准 */
    float open_ticks;         /* 完全打开位置的实际 ticks */
    float closed_ticks;       /* 完全闭合位置的实际 ticks */
};

/* ==========================================================================
 * Position Mapping: [0.0 ~ 1.0] <-> Feetech ticks
 * ========================================================================== */  // NOLINT

/* 默认硬件范围（未校准时使用） */
#define GRIPPER_CLOSED_TICKS 2031.0f  /* 完全闭合位置 (默认) */
#define GRIPPER_OPEN_TICKS   3468.0f  /* 完全打开位置 (默认) */

/**
 * @brief 夹爪开合度转 Feetech 刻度（支持校准）
 *
 * 如果已校准，使用实际测量的范围；否则使用默认范围。
 *
 * @param p        私有数据（包含校准信息）
 * @param position 开合度 [0.0, 1.0]
 * @return Feetech ticks
 */
static inline float position_to_ticks_calibrated(struct so101_gripper_priv *p, float position) {  // NOLINT
    float closed = p->calibrated ? p->closed_ticks : GRIPPER_CLOSED_TICKS;
    float open   = p->calibrated ? p->open_ticks   : GRIPPER_OPEN_TICKS;

    float ticks = closed + position * (open - closed);

    /* 双向钳位：兼容 open > closed 和 open < closed 两种硬件方向 */  // NOLINT
    float lo = fminf(open, closed);
    float hi = fmaxf(open, closed);
    if (ticks < lo) ticks = lo;
    if (ticks > hi) ticks = hi;
    return ticks;
}

/**
 * @brief Feetech 刻度转夹爪开合度（支持校准）
 *
 * @param p     私有数据（包含校准信息）
 * @param ticks Feetech ticks
 * @return 开合度 [0.0, 1.0]
 */
static inline float ticks_to_position_calibrated(struct so101_gripper_priv *p, float ticks) {  // NOLINT
    float closed = p->calibrated ? p->closed_ticks : GRIPPER_CLOSED_TICKS;
    float open   = p->calibrated ? p->open_ticks   : GRIPPER_OPEN_TICKS;

    float range = open - closed;
    if (fabsf(range) < 1.0f)
    return 0.5f; /* 避免除零：范围无效时返回中位 */

    return (ticks - closed) / range;
}

/* ==========================================================================
 * Motor Configuration & Torque Protection
 * ========================================================================== */  // NOLINT

/**
 * @brief 配置夹爪舵机参数和过载保护
 *
 * 夹爪需要额外的扭矩保护参数，防止过载损坏:
 *   - Max_Torque_Limit = 500 (限制最大扭矩为约 50%)
 *   - Protection_Current = 250 mA (过流保护阈值)
 *   - Overload_Torque = 25 (过载扭矩检测阈值)
 *   - PID: P=16, I=0, D=0 (与臂一致)
 *   - Return_Delay_Time = 0
 *   - Operating_Mode = 0 (位置伺服)
 *   - Acceleration = 254
 *
 * @param motor 夹爪电机句柄
 * @return 0 成功，-1 失败
 */
static int so101_gripper_configure_motor(struct motor_dev *motor) {
    /* 先关闭扭矩 (解锁 EPROM) */
    reg_write_byte(motor, REG_TORQUE_ENABLE, 0);
    reg_write_byte(motor, REG_LOCK, 0);

    /* Return_Delay_Time = 0 */
    reg_write_byte(motor, STS3215_RETURN_DELAY_TIME, 0);

    /* PID: P=16, I=0, D=0 */
    reg_write_byte(motor, STS3215_P_COEFFICIENT, STS3215_SO101_P);
    reg_write_byte(motor, STS3215_I_COEFFICIENT, STS3215_SO101_I);
    reg_write_byte(motor, STS3215_D_COEFFICIENT, STS3215_SO101_D);

    /* Operating_Mode = 0 (位置伺服) */
    reg_write_byte(motor, REG_MODE, 0);

    /* === 夹爪专用: 扭矩保护 === */

    /* Max_Torque_Limit = 500 (约 50% of 1000) */
    if (reg_write_word(motor, STS3215_MAX_TORQUE_LIMIT_L,
                            STS3215_GRIPPER_MAX_TORQUE) != 0) {
    fprintf(stderr, "[SO101-Gripper] set Max_Torque_Limit failed\n");
    }

    /* Protection_Current = 250 mA */
    if (reg_write_word(motor, STS3215_PROTECTION_CURRENT_L,
                            STS3215_GRIPPER_PROT_CURRENT) != 0) {
    fprintf(stderr, "[SO101-Gripper] set Protection_Current failed\n");
    }

    /* Overload_Torque = 25 */
    if (reg_write_byte(motor, STS3215_OVERLOAD_TORQUE,
                            STS3215_GRIPPER_OVERLOAD_TRQ) != 0) {
    fprintf(stderr, "[SO101-Gripper] set Overload_Torque failed\n");
    }

    /* Acceleration = 254 (SRAM) */
    reg_write_byte(motor, REG_ACC, STS3215_SO101_ACC);

    /* 使能扭矩，锁定 EPROM */
    reg_write_byte(motor, REG_TORQUE_ENABLE, 1);
    reg_write_byte(motor, REG_LOCK, 1);

    printf("[SO101-Gripper] 夹爪参数及过载保护配置完成 "
            "(MaxTorque=%d ProtCurrent=%d OverloadTrq=%d)\n",
            STS3215_GRIPPER_MAX_TORQUE,
            STS3215_GRIPPER_PROT_CURRENT,
            STS3215_GRIPPER_OVERLOAD_TRQ);
    return 0;
}

/* ==========================================================================
 * Driver Ops
 * ========================================================================== */  // NOLINT

/**
 * @brief 执行抓取命令
 *
 * - GRASP_CMD_GRAB:    夹爪完全闭合 (2031 ticks)
 * - GRASP_CMD_RELEASE: 夹爪完全打开 (3468 ticks)
 * - GRASP_CMD_RELAX:   释放扭矩 (舵机空闲模式)
 *
 * @param dev    设备句柄
 * @param type   命令类型
 * @param effort 力度 [0.0~1.0]，当前未使用，预留
 * @return GRASP_OK 成功，GRASP_ERR_PARAM 参数错误
 */
static int so101_gripper_execute(struct grasp_dev *dev, grasp_cmd_type_t type,
                                    float effort) {
    (void)effort; /* TODO: 后续用 effort 缩放速度 */
    if (!dev)
    return GRASP_ERR_PARAM;

    struct so101_gripper_priv *p =
        (struct so101_gripper_priv *)dev->priv_data;

    struct motor_cmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.vel_des = (float)SO101_GRIPPER_DEFAULT_VEL;

    pthread_mutex_lock(&dev->state_lock);

    switch (type) {
    case GRASP_CMD_RELEASE:
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = position_to_ticks_calibrated(p, 1.0f); /* fully open */
    dev->state = GRASP_STATE_MOVING;
    /* 保存目标命令和目标位置，用于 tick() 持续发送和状态判断 */  // NOLINT
    p->has_target = true;
    p->target_cmd = cmd;
    p->target_position = 1.0f; /* 目标是打开 */
    p->state_confirm_ms = 0.0f; /* 重置状态确认计时器 */
    p->stable_time_ms = 0.0f; /* 重置位置稳定计时器 */
    break;
    case GRASP_CMD_GRAB:
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = position_to_ticks_calibrated(p, 0.0f); /* fully closed */
    dev->state = GRASP_STATE_MOVING;
    /* 保存目标命令和目标位置，用于 tick() 持续发送和状态判断 */  // NOLINT
    p->has_target = true;
    p->target_cmd = cmd;
    p->target_position = 0.0f; /* 目标是闭合 */
    p->state_confirm_ms = 0.0f; /* 重置状态确认计时器 */
    p->stable_time_ms = 0.0f; /* 重置位置稳定计时器 */
    break;
    case GRASP_CMD_RELAX:
    /* 直接关闭扭矩，不走 motor_set_cmds
     * (MOTOR_MODE_IDLE 在 Feetech 驱动中会发送 position=0) */
    reg_write_byte(p->motor, REG_TORQUE_ENABLE, 0);
    dev->state = GRASP_STATE_IDLE;
    p->has_target = false;
    dev->elapsed_ms = 0.0f;
    pthread_mutex_unlock(&dev->state_lock);
    return GRASP_OK;
    default:
    pthread_mutex_unlock(&dev->state_lock);
    return GRASP_ERR_PARAM;
    }

    dev->elapsed_ms = 0.0f;
    pthread_mutex_unlock(&dev->state_lock);

    return motor_set_cmds(&p->motor, &cmd, 1);
}

/**
 * @brief 精细位置控制 — 设置夹爪开合度
 *
 * @param dev      设备句柄
 * @param position 目标开合度 [0.0=全闭, 1.0=全开]
 * @return GRASP_OK 成功，GRASP_ERR_PARAM 参数越界
 */
static int so101_gripper_set_position(struct grasp_dev *dev, float position) {
    if (!dev || position < 0.0f || position > 1.0f)
    return GRASP_ERR_PARAM;

    struct so101_gripper_priv *p =
        (struct so101_gripper_priv *)dev->priv_data;

    struct motor_cmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = position_to_ticks_calibrated(p, position);
    cmd.vel_des = (float)SO101_GRIPPER_DEFAULT_VEL;

    pthread_mutex_lock(&dev->state_lock);
    dev->state = GRASP_STATE_MOVING;
    dev->elapsed_ms = 0.0f;
    /* 保存目标命令和目标位置，用于 tick() 持续发送和状态判断 */  // NOLINT
    p->has_target = true;
    p->target_cmd = cmd;
    p->target_position = position;
    p->state_confirm_ms = 0.0f; /* 重置状态确认计时器 */
    p->stable_time_ms = 0.0f; /* 重置位置稳定计时器 */
    pthread_mutex_unlock(&dev->state_lock);
    return motor_set_cmds(&p->motor, &cmd, 1);
}

/**
 * @brief 获取夹爪当前状态
 *
 * @return IDLE/MOVING/HOLDING/EMPTY/ERROR
 */
static grasp_state_t so101_gripper_get_state(struct grasp_dev *dev) {
    if (!dev)
    return GRASP_STATE_ERROR;

    pthread_mutex_lock(&dev->state_lock);
    grasp_state_t s = dev->state;
    pthread_mutex_unlock(&dev->state_lock);
    return s;
}

/**
 * @brief 获取夹爪物理反馈 — 从舵机读取实际位置和负载
 *
 * @param dev      设备句柄
 * @param out_pos  [输出] 当前开合度 [0.0~1.0]，可为 NULL
 * @param out_load [输出] 当前负载值 (Feetech 原始值 0-1000)，可为 NULL  // NOLINT
 * @return GRASP_OK 成功，GRASP_ERR_CONNECT 通信失败
 */
static int so101_gripper_get_feedback(struct grasp_dev *dev, float *out_pos,
                                        float *out_load) {
    if (!dev)
    return GRASP_ERR_PARAM;

    struct so101_gripper_priv *p =
        (struct so101_gripper_priv *)dev->priv_data;

    struct motor_state ms;
    if (motor_get_states(&p->motor, &ms, 1) != 0)
    return GRASP_ERR_CONNECT;

    pthread_mutex_lock(&dev->state_lock);
    dev->cur_position = ticks_to_position_calibrated(p, ms.pos);
    dev->cur_load = ms.trq;

    if (out_pos)
    *out_pos = dev->cur_position;
    if (out_load)
    *out_load = dev->cur_load;
    pthread_mutex_unlock(&dev->state_lock);

    return GRASP_OK;
}

/**
 * @brief 周期更新 — 刷新状态机
 *
 * 应用层应以固定频率调用。流程：
 *   1. 如果有目标命令且处于运动状态，持续发送命令
 *   2. 从舵机读取实际位置和负载
 *   3. 根据负载判断是否夹住物体 (HOLDING)
 *   4. 根据位置判断是否夹空 (EMPTY) 或打开完成 (IDLE)
 *   5. 检查超时
 *
 * @param dev  设备句柄
 * @param dt_s 距上次 tick 的时间间隔 (秒)
 */
static void so101_gripper_tick(struct grasp_dev *dev, float dt_s) {
    if (!dev)
    return;

    struct so101_gripper_priv *p =
        (struct so101_gripper_priv *)dev->priv_data;

    /* 如果有目标命令且处于运动状态，持续发送 */
    if (p->has_target && dev->state == GRASP_STATE_MOVING) {
    motor_set_cmds(&p->motor, &p->target_cmd, 1);
    }

    /* 从硬件刷新位置和负载 */
    so101_gripper_get_feedback(dev, NULL, NULL);

    pthread_mutex_lock(&dev->state_lock);

    if (dev->state == GRASP_STATE_MOVING || dev->state == GRASP_STATE_HOLDING) {
    dev->elapsed_ms += dt_s * 1000.0f;

    /* 检测位置是否稳定 */
    float pos_change = fabsf(dev->cur_position - p->last_position);
    if (pos_change < 0.01f) {
        /* 位置变化 < 1%，累积稳定时间 */
        p->stable_time_ms += dt_s * 1000.0f;
    } else {
        /* 位置变化较大，重置稳定时间 */
        p->stable_time_ms = 0.0f;
    }
    p->last_position = dev->cur_position;

    /* 只有位置稳定 > 200ms 后才进行状态判断 */
    bool position_stable = (p->stable_time_ms >= 200.0f);

    /* 检测是否到达目标位置 */
    float pos_error = fabsf(dev->cur_position - p->target_position);
    bool reached_target = (pos_error < 0.02f); /* 位置误差 < 2% 认为到达 */  // NOLINT
    bool is_closing = (p->target_position < 0.5f); /* 目标是闭合方向 */

    /* 闭合方向：优先检测 EMPTY 和 HOLDING */
    if (is_closing && position_stable) {
        /* 优先检测完全闭合（夹空）- EMPTY 可以覆盖 HOLDING，且有粘性 */  // NOLINT
        if (dev->cur_position < 0.05f || dev->state == GRASP_STATE_EMPTY) {
        /* 一旦进入 EMPTY，就保持 EMPTY（除非位置明显远离闭合位置） */  // NOLINT
        if (dev->state == GRASP_STATE_EMPTY && dev->cur_position > 0.15f) {
            /* 位置远离闭合区域，退出 EMPTY 状态 */
            dev->state = GRASP_STATE_MOVING;
            p->state_confirm_ms = 0.0f;
        } else if (dev->cur_position < 0.05f) {
            /* 位置在闭合区域，进入或保持 EMPTY */
            if (dev->state != GRASP_STATE_EMPTY) {
            dev->state = GRASP_STATE_EMPTY;
            p->state_confirm_ms = 0.0f;
            } else {
            p->state_confirm_ms += dt_s * 1000.0f;
            if (p->state_confirm_ms >= 300.0f) {
                p->has_target = false; /* 确认 0.3 秒后停止监控 */
            }
            }
        }
        } else if (dev->cur_load > dev->config.hold_threshold) { /* 检测负载高（夹住物体）- 不能覆盖 EMPTY 状态 */ // NOLINT
        if (dev->state != GRASP_STATE_HOLDING) {
            dev->state = GRASP_STATE_HOLDING;
            p->state_confirm_ms = 0.0f;
        } else {
            p->state_confirm_ms += dt_s * 1000.0f;
            if (p->state_confirm_ms >= 300.0f) {
            p->has_target = false; /* 确认 0.3 秒后停止监控 */
            }
        }
        } else if (reached_target && p->has_target) { /* 到达目标位置且负载正常（正常完成） */ // NOLINT
        dev->state = GRASP_STATE_IDLE;
        p->has_target = false;
        }
    } else if (!is_closing && position_stable) { /* 张开方向：只检测到达目标 */
        if (reached_target && p->has_target) {
        dev->state = GRASP_STATE_IDLE;
        p->has_target = false;
        }
    }

    /* 通用超时检测（备用，防止异常情况） */
    if (p->has_target && dev->config.timeout_ms > 0 &&
        dev->elapsed_ms >= (float)dev->config.timeout_ms) {
        /* 超时：停止监控，转为 IDLE */
        dev->state = GRASP_STATE_IDLE;
        p->has_target = false;
    }
    }

    pthread_mutex_unlock(&dev->state_lock);
}

/**
 * @brief 释放夹爪实例 — 关闭舵机并释放内存
 *
 * priv_data 由 grasp_dev_free_default 统一释放。
 */
static void so101_gripper_free(struct grasp_dev *dev) {
    if (!dev)
    return;
    struct so101_gripper_priv *p =
        (struct so101_gripper_priv *)dev->priv_data;
    if (p) {
    motor_free(&p->motor, 1);
    /* priv_data freed by grasp_dev_free_default */
    }
    grasp_dev_free_default(dev);
}

/**
 * @brief 等待用户按回车键（清空输入缓冲区）
 *
 * 清空输入缓冲区中的残留字符，然后等待用户按回车。
 * 如果缓冲区中已有换行符（残留），会先消耗它，然后等待用户输入新的回车。  // NOLINT
 */
static void wait_for_enter(void) {
    int c;
    int count = 0;

    /* 清空输入缓冲区 */
    while ((c = getchar()) != '\n' && c != EOF) {
    count++;
    }

    /* 如果第一次就读到了换行符 (count == 0)，说明缓冲区中有残留 */  // NOLINT
    /* 需要再等待用户输入一个新的回车 */
    if (count == 0 && c == '\n') {
    /* 缓冲区中有残留的换行符，再等待一个新的 */
    while ((c = getchar()) != '\n' && c != EOF)
        {}
    }
}

/**
 * @brief 保存校准数据到配置文件 (JSON 格式)
 *
 * @param id           舵机 ID
 * @param open_ticks   打开位置 ticks
 * @param closed_ticks 闭合位置 ticks
 * @return 0 成功，-1 失败
 */
static int save_calibration(uint8_t id, float open_ticks, float closed_ticks) {
    /* 确保 config 目录存在 */
    system("mkdir -p ./config");

    FILE *fp = fopen(CALIBRATION_FILE, "w");
    if (!fp) {
    fprintf(stderr, "警告: 无法保存校准数据到 %s\n", CALIBRATION_FILE);  // NOLINT
    return -1;
    }

    /* 使用 JSON 格式，与 manipulator 保持一致 */
    fprintf(fp, "{\n");
    fprintf(fp, "  \"motor_id\": %u,\n", id);
    fprintf(fp, "  \"open_ticks\": %.1f,\n", open_ticks);
    fprintf(fp, "  \"closed_ticks\": %.1f\n", closed_ticks);
    fprintf(fp, "}\n");

    fclose(fp);
    printf("校准数据已保存到: %s\n", CALIBRATION_FILE);
    return 0;
}

/**
 * @brief 从配置文件加载校准数据 (JSON 格式)
 *
 * @param id           [输入] 舵机 ID
 * @param open_ticks   [输出] 打开位置 ticks
 * @param closed_ticks [输出] 闭合位置 ticks
 * @return 0 成功，-1 失败或文件不存在
 */
static int load_calibration(uint8_t id, float *open_ticks, float *closed_ticks) {  // NOLINT
    FILE *fp = fopen(CALIBRATION_FILE, "r");
    if (!fp) {
    return -1; /* 文件不存在，使用默认值 */
    }

    uint8_t file_id = 0;
    float open = 0.0f, closed = 0.0f;
    bool found_id = false, found_open = false, found_closed = false;

    /* 简单的 JSON 解析 (只解析我们需要的字段) */
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
    if (sscanf(line, "  \"motor_id\": %hhu,", &file_id) == 1) {
        found_id = true;
    } else if (sscanf(line, "  \"open_ticks\": %f,", &open) == 1) {
        found_open = true;
    } else if (sscanf(line, "  \"closed_ticks\": %f", &closed) == 1) {
        found_closed = true;
    }
    }

    fclose(fp);

    /* 验证数据完整性和 ID 匹配 */
    if (found_id && found_open && found_closed && file_id == id) {
    *open_ticks = open;
    *closed_ticks = closed;
    printf("[SO101-Gripper] 已加载校准数据: open=%.1f, closed=%.1f\n",
            open, closed);
    return 0;
    }

    return -1; /* 数据不完整或 ID 不匹配 */
}

/**
 * @brief 校准夹爪 - 交互式学习实际硬件边界
 *
 * 流程：
 *   1. 释放扭矩，允许手动移动
 *   2. 提示用户手动移动到完全打开位置，按回车确认
 *   3. 读取并保存 open_ticks
 *   4. 提示用户手动移动到完全闭合位置，按回车确认
 *   5. 读取并保存 closed_ticks
 *   6. 验证范围有效性
 *   7. 标记 calibrated = true
 *
 * @param dev 设备句柄
 * @return GRASP_OK 成功，GRASP_ERR_CONNECT 读取失败
 */
static int so101_gripper_calibrate(struct grasp_dev *dev) {
    if (!dev)
    return GRASP_ERR_PARAM;

    struct so101_gripper_priv *p = (struct so101_gripper_priv *)dev->priv_data;
    struct motor_state ms;

    printf("\n========== 夹爪校准程序 ==========\n");
    printf("校准将学习夹爪的实际开合范围\n\n");

    /* 步骤 1: 释放扭矩 */
    printf("步骤 1: 释放扭矩，允许手动移动...\n");
    reg_write_byte(p->motor, REG_TORQUE_ENABLE, 0);
    usleep(500000); /* 等待 500ms */

    /* 步骤 2: 测量完全打开位置 */
    printf("\n步骤 2: 请手动将夹爪移动到 **完全打开** 位置\n");
    printf("        (夹爪张开到最大)\n");
    printf("        完成后按 [回车] 键继续...");
    fflush(stdout);
    wait_for_enter(); /* 等待用户按回车 */

    if (motor_get_states(&p->motor, &ms, 1) != 0) {
    fprintf(stderr, "\n错误: 无法读取舵机位置\n");
    return GRASP_ERR_CONNECT;
    }
    p->open_ticks = ms.pos;
    printf("        已记录打开位置: %.1f ticks\n", p->open_ticks);

    /* 步骤 3: 测量完全闭合位置 */
    printf("\n步骤 3: 请手动将夹爪移动到 **完全闭合** 位置\n");
    printf("        (夹爪完全合拢)\n");
    printf("        完成后按 [回车] 键继续...");
    fflush(stdout);
    wait_for_enter(); /* 等待用户按回车 */

    if (motor_get_states(&p->motor, &ms, 1) != 0) {
    fprintf(stderr, "\n错误: 无法读取舵机位置\n");
    return GRASP_ERR_CONNECT;
    }
    p->closed_ticks = ms.pos;
    printf("        已记录闭合位置: %.1f ticks\n", p->closed_ticks);

    /* 步骤 4: 验证范围 */
    printf("\n步骤 4: 验证校准结果...\n");
    float range = p->open_ticks - p->closed_ticks;

    if (range < 100.0f) {
    fprintf(stderr, "错误: 校准范围过小 (%.1f ticks)\n", range);
    fprintf(stderr, "      请确保打开和闭合位置有明显差异\n");
    p->calibrated = false;
    return GRASP_ERR_CONFIG;
    }

    if (p->open_ticks <= p->closed_ticks) {
    fprintf(stderr, "错误: 打开位置 (%.1f) 应该大于闭合位置 (%.1f)\n",  // NOLINT
            p->open_ticks, p->closed_ticks);
    p->calibrated = false;
    return GRASP_ERR_CONFIG;
    }

    /* 步骤 5: 完成校准 */
    p->calibrated = true;
    printf("\n========== 校准成功！ ==========\n");
    printf("闭合位置: %.1f ticks (position 0.0)\n", p->closed_ticks);
    printf("打开位置: %.1f ticks (position 1.0)\n", p->open_ticks);
    printf("有效范围: %.1f ticks\n", range);
    printf("================================\n\n");

    /* 保存校准数据到配置文件 */
    save_calibration(p->motor_id, p->open_ticks, p->closed_ticks);

    /* 恢复扭矩 */
    printf("恢复扭矩控制...\n");
    reg_write_byte(p->motor, REG_TORQUE_ENABLE, 1);

    return GRASP_OK;
}

/**
 * @brief SO-101 夹爪驱动操作表
 *
 * 注册到 grasp 框架的回调函数集合。
 * 所有操作均已实现。
 */
static const struct grasp_ops so101_gripper_ops = {
    .execute = so101_gripper_execute,
    .set_position = so101_gripper_set_position,
    .get_state = so101_gripper_get_state,
    .get_feedback = so101_gripper_get_feedback,
    .tick = so101_gripper_tick,
    .calibrate = so101_gripper_calibrate,
    .free = so101_gripper_free,
};

/* ==========================================================================
 * Factory Function
 * ========================================================================== */  // NOLINT

/**
 * @brief 工厂函数 — 创建一个 SO-101 夹爪实例
 *
 * 由框架在 grasp_alloc("so101_gripper", args) 时调用。
 * 流程：
 *   1. 解析配置 (args)，或使用默认值 (ID=6, 1Mbaud)
 *   2. 分配 grasp_dev + so101_gripper_priv 内存
 *   3. 创建 1 个 motor_dev (Feetech UART)
 *   4. 初始化电机
 *   5. 挂载操作表，标记 running=true
 *
 * @param name 驱动名 (由框架传入，即 "so101_gripper")
 * @param args 配置参数指针 (struct so101_gripper_config*)，可为 NULL
 * @return 成功返回设备句柄，失败返回 NULL
 */
static struct grasp_dev *so101_gripper_factory(const char *name, void *args) {
    struct so101_gripper_config cfg;

    if (args) {
    memcpy(&cfg, args, sizeof(cfg));
    } else {
    /* Defaults */
    cfg.uart_path = "/dev/ttyACM0";
    cfg.baud = 1000000;
    cfg.id = 6;
    cfg.grasp_cfg.max_effort = 1.0f;
    cfg.grasp_cfg.hold_threshold = 100.0f; /* Feetech load units */
    cfg.grasp_cfg.timeout_ms = 3000;
    }

    struct grasp_dev *dev =
        grasp_dev_alloc(name, sizeof(struct so101_gripper_priv));
    if (!dev)
    return NULL;

    /* Apply config */
    dev->config = cfg.grasp_cfg;

    struct so101_gripper_priv *p =
        (struct so101_gripper_priv *)dev->priv_data;

    /* Initialize target command state */
    p->has_target = false;
    memset(&p->target_cmd, 0, sizeof(p->target_cmd));
    p->motor_id = cfg.id;

    /* Initialize calibration state (use default range) */
    p->calibrated = false;
    p->open_ticks = GRIPPER_OPEN_TICKS;
    p->closed_ticks = GRIPPER_CLOSED_TICKS;

    /* Allocate motor */
    p->motor = motor_alloc_uart("drv_uart_feetech", cfg.uart_path, cfg.baud, cfg.id, NULL);  // NOLINT
    if (!p->motor) {
    fprintf(stderr, "[SO101-Gripper] Failed to alloc gripper motor (ID=%u)\n",
            cfg.id);
    grasp_dev_free_default(dev);
    return NULL;
    }

    if (motor_init(&p->motor, 1) != 0) {
    fprintf(stderr, "[SO101-Gripper] motor_init failed\n");
    motor_free(&p->motor, 1);
    grasp_dev_free_default(dev);
    return NULL;
    }

    /* 尝试加载校准数据 */
    if (load_calibration(cfg.id, &p->open_ticks, &p->closed_ticks) == 0) {
    /* 验证加载的数据有效性 */
    float range = p->open_ticks - p->closed_ticks;
    if (range > 100.0f && p->open_ticks > p->closed_ticks) {
        p->calibrated = true;
        printf("[SO101-Gripper] 使用已保存的校准数据 (range=%.1f ticks)\n", range);  // NOLINT
    } else {
        printf("[SO101-Gripper] 校准数据无效，使用默认范围\n");
        p->calibrated = false;
        p->open_ticks = GRIPPER_OPEN_TICKS;
        p->closed_ticks = GRIPPER_CLOSED_TICKS;
    }
    } else {
    printf("[SO101-Gripper] 未找到校准数据，使用默认范围\n");
    }

    /* 配置夹爪舵机参数 + 过载保护 */
    so101_gripper_configure_motor(p->motor);

    dev->ops = &so101_gripper_ops;
    dev->running = true;
    return dev;
}

/**
 * @brief 驱动自动注册
 *
 * 程序启动时自动将 "so101_gripper" 注册到 grasp 框架的驱动链表中。  // NOLINT
 */
REGISTER_GRASP_DRIVER("so101_gripper", so101_gripper_factory)

#endif /* HAVE_MOTOR */
