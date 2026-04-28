/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp.c
 * @brief Core implementation for grasp/end-effector control component
 */

#include "grasp_core.h" // NOLINT
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ==========================================================================
 * Driver Registry (linked list, same pattern as chassis/manipulator)
 * ========================================================================== */

static struct grasp_driver_info *g_driver_list = NULL;

void grasp_driver_register(struct grasp_driver_info *info) {
  if (!info)
    return;
  info->next = g_driver_list;
  g_driver_list = info;
  printf("[GRASP] Registered driver: %s\n", info->name);
}

static struct grasp_driver_info *find_driver(const char *name) {
  struct grasp_driver_info *curr = g_driver_list;

  while (curr) {
    if (curr->name && name && strcmp(curr->name, name) == 0)
      return curr;
    curr = curr->next;
  }

  printf("[GRASP] Driver not found: %s\n", name ? name : "(null)");
  return NULL;
}

/* ==========================================================================
 * Device Allocation Helper
 * ========================================================================== */

struct grasp_dev *grasp_dev_alloc(const char *name, size_t priv_size) {
  struct grasp_dev *dev;
  void *priv = NULL;

  dev = calloc(1, sizeof(*dev));
  if (!dev)
    return NULL;

  if (priv_size) {
    priv = calloc(1, priv_size);
    if (!priv) {
      free(dev);
      return NULL;
    }
    dev->priv_data = priv;
  }

  if (name) {
    dev->name = strdup(name);
    if (!dev->name) {
      free(priv);
      free(dev);
      return NULL;
    }
  }

  dev->state = GRASP_STATE_IDLE;
  dev->cur_position = 0.0f;
  dev->cur_load = 0.0f;
  dev->elapsed_ms = 0.0f;
  dev->running = false;
  pthread_mutex_init(&dev->state_lock, NULL);

  /* Default config */
  dev->config.max_effort = 1.0f;
  dev->config.hold_threshold = 0.2f;
  dev->config.timeout_ms = 3000;

  return dev;
}

void grasp_dev_free_default(struct grasp_dev *dev) {
  if (!dev)
    return;

  pthread_mutex_destroy(&dev->state_lock);

  if (dev->priv_data)
    free(dev->priv_data);
  if (dev->name)
    free((void *)dev->name);
  free(dev);
}

/* ==========================================================================
 * Public API Implementation
 * ========================================================================== */

struct grasp_dev *grasp_alloc(const char *driver_name, void *args) {
  struct grasp_driver_info *drv;

  if (!driver_name)
    return NULL;

  drv = find_driver(driver_name);
  if (!drv || !drv->factory) {
    printf("[GRASP] No driver found: %s\n", driver_name);
    return NULL;
  }

  return drv->factory(driver_name, args);
}

int grasp_execute(struct grasp_dev *dev, grasp_cmd_type_t type, float effort) {
  if (!dev)
    return GRASP_ERR_PARAM;

  if (dev->ops && dev->ops->execute)
    return dev->ops->execute(dev, type, effort);

  return GRASP_ERR_NOSYS;
}

int grasp_set_position(struct grasp_dev *dev, float position) {
  if (!dev)
    return GRASP_ERR_PARAM;
  if (position < 0.0f || position > 1.0f)
    return GRASP_ERR_PARAM;

  if (dev->ops && dev->ops->set_position)
    return dev->ops->set_position(dev, position);

  return GRASP_ERR_NOSYS;
}

grasp_state_t grasp_get_state(struct grasp_dev *dev) {
  if (!dev || !dev->ops || !dev->ops->get_state)
    return GRASP_STATE_ERROR;

  return dev->ops->get_state(dev);
}

int grasp_get_feedback(struct grasp_dev *dev, float *out_pos,
                       float *out_load) {
  if (!dev)
    return GRASP_ERR_PARAM;

  if (dev->ops && dev->ops->get_feedback)
    return dev->ops->get_feedback(dev, out_pos, out_load);

  return GRASP_ERR_NOSYS;
}

void grasp_tick(struct grasp_dev *dev, float dt_s) {
  if (!dev)
    return;

  if (dev->ops && dev->ops->tick)
    dev->ops->tick(dev, dt_s);
}

int grasp_calibrate(struct grasp_dev *dev) {
  if (!dev)
    return GRASP_ERR_PARAM;

  if (dev->ops && dev->ops->calibrate)
    return dev->ops->calibrate(dev);

  return GRASP_ERR_NOSYS;
}

void grasp_stop(struct grasp_dev *dev) {
  if (!dev)
    return;
  /* 等效于 RELAX 命令：释放扭矩，停止所有运动 */
  grasp_execute(dev, GRASP_CMD_RELAX, 0.0f);
}

void grasp_free(struct grasp_dev *dev) {
  if (!dev)
    return;

  /* 释放前先 RELAX，防止舵机在释放后继续执行上次命令 */
  grasp_stop(dev);

  dev->running = false;

  if (dev->ops && dev->ops->free) {
    dev->ops->free(dev);
    return;
  }

  grasp_dev_free_default(dev);
}
