/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_grasp.c
 * @brief Unit tests for grasp module (dummy driver)
 *
 * Tests the core grasp API using the dummy driver (drv_dummy.c).
 * The dummy driver simulates state transitions without hardware.
 *
 * For hardware testing with real grippers (e.g., SO-101), see:
 *   - test_hw_so101_gripper.c: Interactive hardware test program
 *   - HW_TEST_GUIDE.md: Hardware test guide
 */


#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "grasp.h" // NOLINT

static void test_alloc_free(void) {
  printf("[test] alloc/free (dummy driver) ... ");
  struct grasp_dev *dev = grasp_alloc("dummy", NULL);
  assert(dev != NULL);
  grasp_free(dev);
  printf("OK\n");
}

static void test_invalid_driver(void) {
  printf("[test] invalid driver ... ");
  struct grasp_dev *dev = grasp_alloc("nonexistent_driver", NULL);
  assert(dev == NULL);
  printf("OK\n");
}

static void test_null_params(void) {
  printf("[test] null params ... ");
  assert(grasp_execute(NULL, GRASP_CMD_GRAB, 0.5f) == GRASP_ERR_PARAM);
  assert(grasp_set_position(NULL, 0.5f) == GRASP_ERR_PARAM);
  assert(grasp_get_state(NULL) == GRASP_STATE_ERROR);
  assert(grasp_get_feedback(NULL, NULL, NULL) == GRASP_ERR_PARAM);
  printf("OK\n");
}

static void test_execute_grab(void) {
  printf("[test] execute grab ... ");
  struct grasp_dev *dev = grasp_alloc("dummy", NULL);
  assert(dev != NULL);

  int ret = grasp_execute(dev, GRASP_CMD_GRAB, 0.8f);
  assert(ret == GRASP_OK);
  assert(grasp_get_state(dev) == GRASP_STATE_MOVING);

  grasp_free(dev);
  printf("OK\n");
}

static void test_execute_release(void) {
  printf("[test] execute release ... ");
  struct grasp_dev *dev = grasp_alloc("dummy", NULL);
  assert(dev != NULL);

  int ret = grasp_execute(dev, GRASP_CMD_RELEASE, 0.0f);
  assert(ret == GRASP_OK);
  assert(grasp_get_state(dev) == GRASP_STATE_MOVING);

  grasp_free(dev);
  printf("OK\n");
}

static void test_set_position(void) {
  printf("[test] set_position ... ");
  struct grasp_dev *dev = grasp_alloc("dummy", NULL);
  assert(dev != NULL);

  int ret = grasp_set_position(dev, 0.5f);
  assert(ret == GRASP_OK);
  assert(grasp_get_state(dev) == GRASP_STATE_MOVING);

  /* Verify feedback returns the position we set */
  float pos = -1.0f, load = -1.0f;
  ret = grasp_get_feedback(dev, &pos, &load);
  assert(ret == GRASP_OK);
  assert(fabsf(pos - 0.5f) < 0.01f);

  grasp_free(dev);
  printf("OK\n");
}

static void test_set_position_bounds(void) {
  printf("[test] set_position bounds ... ");
  struct grasp_dev *dev = grasp_alloc("dummy", NULL);
  assert(dev != NULL);

  assert(grasp_set_position(dev, -0.1f) == GRASP_ERR_PARAM);
  assert(grasp_set_position(dev, 1.1f) == GRASP_ERR_PARAM);
  assert(grasp_set_position(dev, 0.0f) == GRASP_OK);
  assert(grasp_set_position(dev, 1.0f) == GRASP_OK);

  grasp_free(dev);
  printf("OK\n");
}

static void test_tick_state_transition(void) {
  printf("[test] tick state transition ... ");
  struct grasp_dev *dev = grasp_alloc("dummy", NULL);
  assert(dev != NULL);

  /* Grab → MOVING */
  grasp_execute(dev, GRASP_CMD_GRAB, 0.8f);
  assert(grasp_get_state(dev) == GRASP_STATE_MOVING);

  /* Tick should transition dummy: fully closed (pos=1.0) with no load → EMPTY */  // NOLINT
  grasp_tick(dev, 0.05f);
  grasp_state_t s = grasp_get_state(dev);
  assert(s == GRASP_STATE_EMPTY || s == GRASP_STATE_IDLE);

  grasp_free(dev);
  printf("OK\n");
}

static void test_get_feedback(void) {
  printf("[test] get_feedback ... ");
  struct grasp_dev *dev = grasp_alloc("dummy", NULL);
  assert(dev != NULL);

  float pos = -1.0f, load = -1.0f;
  int ret = grasp_get_feedback(dev, &pos, &load);
  assert(ret == GRASP_OK);
  assert(pos >= 0.0f && pos <= 1.0f);
  assert(load >= 0.0f);

  grasp_free(dev);
  printf("OK\n");
}

int main(void) {
  printf("=== Grasp Module Unit Tests ===\n");

  test_alloc_free();
  test_invalid_driver();
  test_null_params();
  test_execute_grab();
  test_execute_release();
  test_set_position();
  test_set_position_bounds();
  test_tick_state_transition();
  test_get_feedback();

  printf("=== All grasp tests PASSED ===\n");
  return 0;
}
