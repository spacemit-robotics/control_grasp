/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file drv_dummy.c
 * @brief Dummy grasp driver (stub implementation)
 *
 * 当没有真实夹爪硬件时，使用此 dummy 实现。
 * 所有抓取命令在下一次 tick() 时立即完成。
 */

#include "../grasp_core.h"
#include <stdio.h> // NOLINT
#include <string.h> // NOLINT

/* ==========================================================================
 * Dummy Driver Operations
 * ========================================================================== */

static int dummy_execute(struct grasp_dev *dev, grasp_cmd_type_t type,
    float effort) {
    (void)effort;
    pthread_mutex_lock(&dev->state_lock);
    switch (type) {
    case GRASP_CMD_RELEASE:
        dev->cur_position = 0.0f;
        dev->state = GRASP_STATE_MOVING;
        break;
    case GRASP_CMD_GRAB:
        dev->cur_position = 1.0f;
        dev->state = GRASP_STATE_MOVING;
        break;
    case GRASP_CMD_RELAX:
        dev->state = GRASP_STATE_IDLE;
        break;
    }
    dev->elapsed_ms = 0.0f;
    pthread_mutex_unlock(&dev->state_lock);
    return GRASP_OK;
}

static int dummy_set_position(struct grasp_dev *dev, float position) {
    pthread_mutex_lock(&dev->state_lock);
    dev->cur_position = position;
    dev->state = GRASP_STATE_MOVING;
    dev->elapsed_ms = 0.0f;
    pthread_mutex_unlock(&dev->state_lock);
    return GRASP_OK;
}

static grasp_state_t dummy_get_state(struct grasp_dev *dev) {
    pthread_mutex_lock(&dev->state_lock);
    grasp_state_t s = dev->state;
    pthread_mutex_unlock(&dev->state_lock);
    return s;
}

static int dummy_get_feedback(struct grasp_dev *dev, float *out_pos,
    float *out_load) {
    pthread_mutex_lock(&dev->state_lock);
    if (out_pos)
        *out_pos = dev->cur_position;
    if (out_load)
        *out_load = dev->cur_load;
    pthread_mutex_unlock(&dev->state_lock);
    return GRASP_OK;
}

static void dummy_tick(struct grasp_dev *dev, float dt_s) {
    pthread_mutex_lock(&dev->state_lock);
    if (dev->state == GRASP_STATE_MOVING) {
        dev->elapsed_ms += dt_s * 1000.0f;
        /* Simulate instant completion */
        if (dev->cur_position >= 0.99f)
            dev->state = GRASP_STATE_EMPTY;
        else
            dev->state = GRASP_STATE_IDLE;
    }
    pthread_mutex_unlock(&dev->state_lock);
}

static const struct grasp_ops dummy_ops = {
    .execute = dummy_execute,
    .set_position = dummy_set_position,
    .get_state = dummy_get_state,
    .get_feedback = dummy_get_feedback,
    .tick = dummy_tick,
    .free = NULL, /* use default free */
};

static struct grasp_dev *dummy_factory(const char *name, void *args) {
    (void)args;
    struct grasp_dev *dev = grasp_dev_alloc(name, 0);
    if (!dev)
        return NULL;
    dev->ops = &dummy_ops;
    return dev;
}

REGISTER_GRASP_DRIVER("dummy", dummy_factory)
