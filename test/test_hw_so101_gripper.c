/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_hw_so101_gripper.c
 * @brief SO-101 夹爪硬件测试程序
 *
 * 测试 SO-101 夹爪的基本功能：
 *   - 完全打开/闭合
 *   - 精细位置控制
 *   - 状态反馈
 *   - 抓取检测
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "grasp.h" // NOLINT

/* 强制链接驱动 - 引用外部符号确保驱动代码被链接 */
extern void *__drv_info_so101_gripper_factory __attribute__((weak));
/* 强制链接 motor 库的 feetech 驱动 - 引用 C++ weak 符号 */
#ifdef __cplusplus
extern "C" {
#endif
extern void _ZN11FeetechPackC1Ev(void) __attribute__((weak));
#ifdef __cplusplus
}
#endif

static void force_link_drivers(void) {
  (void)__drv_info_so101_gripper_factory;
  /* Reference FeetechPack constructor to pull in feetech driver */
  if (_ZN11FeetechPackC1Ev) {
    /* This forces linker to include the entire feetech_motor_adapter.cpp.o */
  }
}

/* 默认配置 */
#define DEFAULT_UART_PATH "/dev/ttyACM0"
#define DEFAULT_BAUD      1000000
#define DEFAULT_ID        6

/* 外部配置结构（需要与驱动匹配） */
struct so101_gripper_config {
  const char *uart_path;
  uint32_t baud;
  uint8_t id;
  grasp_config_t grasp_cfg;
};

static void print_menu(void) {
  printf("\n========== SO-101 夹爪硬件测试 ==========\n");
  printf("1. 完全打开 (position = 1.0)\n");
  printf("2. 完全闭合 (position = 0.0)\n");
  printf("3. 中间位置 (position = 0.5)\n");
  printf("4. 手动输入位置 (0.0 ~ 1.0)\n");
  printf("5. 抓取测试 (GRAB 命令)\n");
  printf("6. 释放测试 (RELEASE 命令)\n");
  printf("7. 放松测试 (RELAX 命令)\n");
  printf("8. 查看当前状态和反馈\n");
  printf("9. 校准夹爪 (学习实际边界)\n");
  printf("0. 退出\n");
  printf("=========================================\n");
  printf("请选择: ");
}

static void print_state(grasp_state_t state) {
  switch (state) {
  case GRASP_STATE_IDLE:
    printf("IDLE (空闲)");
    break;
  case GRASP_STATE_MOVING:
    printf("MOVING (运动中)");
    break;
  case GRASP_STATE_HOLDING:
    printf("HOLDING (夹住物体)");
    break;
  case GRASP_STATE_EMPTY:
    printf("EMPTY (夹空)");
    break;
  case GRASP_STATE_ERROR:
    printf("ERROR (错误)");
    break;
  default:
    printf("UNKNOWN");
  }
}

static void wait_and_monitor(struct grasp_dev *dev, float duration_s) {
  printf("监控状态 %.1f 秒...\n", duration_s);
  int steps = (int)(duration_s * 20); /* 100ms per step */
  for (int i = 0; i < steps; i++) {
    usleep(100000); /* 100ms */
    grasp_tick(dev, 0.1f);

    grasp_state_t state = grasp_get_state(dev);
    float pos = 0.0f, load = 0.0f;
    grasp_get_feedback(dev, &pos, &load);

    printf("\r[%.1fs] 状态: ", (i + 1) * 0.1f);
    print_state(state);
    printf(" | 位置: %.3f | 负载: %.1f   ", pos, load);
    fflush(stdout);

    /* 只有 EMPTY 和 IDLE 是最终状态，可以停止 */
    /* HOLDING 可能会变成 EMPTY，需要继续监控 */
    if (state == GRASP_STATE_EMPTY || state == GRASP_STATE_IDLE) {
      printf("\n动作完成!\n");
      break;
    }
  }
  printf("\n");
}

int main(int argc, char *argv[]) {
  const char *uart_path = DEFAULT_UART_PATH;
  uint32_t baud = DEFAULT_BAUD;
  uint8_t id = DEFAULT_ID;

  /* 解析命令行参数 */
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      uart_path = argv[++i];
    } else if (strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
      baud = (uint32_t)atoi(argv[++i]);
    } else if (strcmp(argv[i], "--id") == 0 && i + 1 < argc) {
      id = (uint8_t)atoi(argv[++i]);
    } else if (strcmp(argv[i], "--help") == 0) {
      printf("用法: %s [选项]\n", argv[0]);
      printf("选项:\n");
      printf("  --port <path>  串口设备路径 (默认: %s)\n", DEFAULT_UART_PATH);  // NOLINT
      printf("  --baud <rate>  波特率 (默认: %d)\n", DEFAULT_BAUD);
      printf("  --id <id>      夹爪舵机 ID (默认: %d)\n", DEFAULT_ID);
      printf("  --help         显示此帮助信息\n");
      return 0;
    }
  }

  printf("初始化 SO-101 夹爪...\n");
  printf("  串口: %s\n", uart_path);
  printf("  波特率: %u\n", baud);
  printf("  舵机 ID: %u\n", id);

  /* 配置夹爪 */
  struct so101_gripper_config cfg = {
      .uart_path = uart_path,
      .baud = baud,
      .id = id,
      .grasp_cfg = {
              .max_effort = 1.0f,
              .hold_threshold = 100.0f, /* Feetech 负载单位 */
              .timeout_ms = 5000,
          },
  };

  struct grasp_dev *dev = grasp_alloc("so101_gripper", &cfg);
  if (!dev) {
    fprintf(stderr, "错误: 无法创建夹爪实例\n");
    fprintf(stderr, "请检查:\n");
    fprintf(stderr, "  1. 串口设备是否存在: %s\n", uart_path);
    fprintf(stderr, "  2. 是否有串口访问权限 (sudo usermod -aG dialout $USER)\n");  // NOLINT
    fprintf(stderr, "  3. 夹爪是否正确连接并上电\n");
    fprintf(stderr, "  4. 舵机 ID 是否正确: %u\n", id);
    return 1;
  }

  printf("夹爪初始化成功!\n");

  /* 主循环 */
  int choice;
  while (1) {
    print_menu();
    if (scanf("%d", &choice) != 1) {
      while (getchar() != '\n') {
      } /* 清空输入缓冲 */
      printf("无效输入，请重试\n");
      continue;
    }

    int ret;
    float position;

    switch (choice) {
    case 0:
      printf("退出测试程序\n");
      printf("放松夹爪...\n");
      grasp_execute(dev, GRASP_CMD_RELAX, 0.0f);
      usleep(500000); /* 等待500ms确保命令发送 */
      grasp_free(dev);
      return 0;

    case 1:
      printf("\n执行: 完全打开 (position = 1.0)\n");
      ret = grasp_set_position(dev, 1.0f);
      if (ret != GRASP_OK) {
        printf("错误: 命令失败 (ret=%d)\n", ret);
      } else {
        wait_and_monitor(dev, 3.0f);
      }
      break;

    case 2:
      printf("\n执行: 完全闭合 (position = 0.0)\n");
      ret = grasp_set_position(dev, 0.0f);
      if (ret != GRASP_OK) {
        printf("错误: 命令失败 (ret=%d)\n", ret);
      } else {
        wait_and_monitor(dev, 3.0f);
      }
      break;

    case 3:
      printf("\n执行: 中间位置 (position = 0.5)\n");
      ret = grasp_set_position(dev, 0.5f);
      if (ret != GRASP_OK) {
        printf("错误: 命令失败 (ret=%d)\n", ret);
      } else {
        wait_and_monitor(dev, 3.0f);
      }
      break;

    case 4:
      printf("\n请输入目标位置 (0.0 = 完全闭合, 1.0 = 完全打开): ");  // NOLINT
      if (scanf("%f", &position) != 1 || position < 0.0f || position > 1.0f) {
        printf("无效输入，位置必须在 [0.0, 1.0] 范围内\n");
        while (getchar() != '\n')
          {}
        break;
      }
      printf("执行: 移动到位置 %.3f\n", position);
      ret = grasp_set_position(dev, position);
      if (ret != GRASP_OK) {
        printf("错误: 命令失败 (ret=%d)\n", ret);
      } else {
        wait_and_monitor(dev, 3.0f);
      }
      break;

    case 5:
      printf("\n执行: GRAB 命令 (抓取)\n");
      ret = grasp_execute(dev, GRASP_CMD_GRAB, 0.8f);
      if (ret != GRASP_OK) {
        printf("错误: 命令失败 (ret=%d)\n", ret);
      } else {
        wait_and_monitor(dev, 3.0f);
      }
      break;

    case 6:
      printf("\n执行: RELEASE 命令 (释放)\n");
      ret = grasp_execute(dev, GRASP_CMD_RELEASE, 0.0f);
      if (ret != GRASP_OK) {
        printf("错误: 命令失败 (ret=%d)\n", ret);
      } else {
        wait_and_monitor(dev, 3.0f);
      }
      break;

    case 7:
      printf("\n执行: RELAX 命令 (放松/掉电)\n");
      ret = grasp_execute(dev, GRASP_CMD_RELAX, 0.0f);
      if (ret != GRASP_OK) {
        printf("错误: 命令失败 (ret=%d)\n", ret);
      } else {
        printf("夹爪已放松 (舵机掉电)\n");
      }
      break;

    case 8:
      printf("\n当前状态:\n");
      grasp_state_t state = grasp_get_state(dev);
      float pos = 0.0f, load = 0.0f;
      ret = grasp_get_feedback(dev, &pos, &load);
      if (ret == GRASP_OK) {
        printf("  状态: ");
        print_state(state);
        printf("\n");
        printf("  位置: %.3f (0.0=完全闭合, 1.0=完全打开)\n", pos);
        printf("  负载: %.1f (Feetech 单位)\n", load);
      } else {
        printf("  错误: 无法读取反馈 (ret=%d)\n", ret);
      }
      break;

    case 9:
      printf("\n执行: 校准夹爪\n");
      printf("注意: 校准过程中需要手动移动夹爪\n");
      printf("确认继续? (y/n): ");
      char confirm;
      scanf(" %c", &confirm);
      if (confirm == 'y' || confirm == 'Y') {
        ret = grasp_calibrate(dev);
        if (ret == GRASP_OK) {
          printf("校准完成！后续命令将使用校准后的范围\n");
        } else {
          printf("校准失败 (ret=%d)\n", ret);
        }
      } else {
        printf("已取消校准\n");
      }
      break;

    default:
      printf("无效选择，请重试\n");
    }
  }

  return 0;
}
