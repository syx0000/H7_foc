/**
 * @file    ifly_test.c
 * @brief   故障测试代码
 * @author  xlding15
 * @date    2025-11-19
 * @version 1.0
 */

#include "ifly_test.h"
#include "foc_api.h"
#include "foc_bsp.h"
#include "foc_controller.h"
#include "foc_current_loop.h"
#include "foc_data.h"
#include "foc_speed_loop.h"
#include "ifly_fault.h"
#include "ifly_fault_api.h"
#include "ifly_flux_ident.h"
#include "ifly_inertia_ident.h"
#include "ifly_led.h"
#include "foc_position_loop.h"
// #include "FreeRTOS.h"  /* FreeRTOS removed */
// #include "task.h"  /* FreeRTOS removed */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern volatile uint16_t testLogFlag;
extern ControllerStruct controller_eyou;
extern Portection_Value Threshold;
extern uint8_t test_control;
extern LED_STATUSBits LED_STATUS;
extern ifly_Err_Pro_Type motorProValue;

#define TEST_MOTOR_CURRENT_MODE PROFILE_TORQUE_MODE
#define TEST_MOTOR_SPEED_MODE PROFILE_VELOCITY_MOCE
#define TEST_MOTOR_POSIT_MODE PROFILE_POSITION_MODE

void Test_log_print(void) {
  /* 带宽测试完成 → 主循环中打印结果 (ISR 只能置 done 标志) */
  if (controller_eyou.bw_test.done) {
    bw_test_print_results(&controller_eyou.bw_test);
    controller_eyou.bw_test.done = 0;
  }
}

void TestBusOverUdc(void) {
}

void TestBusLowUdc(void) {
}

void TestBoardOverTem(void) {
}

void TestLockedRotorCurrent(void) {
}

void TestLockedRotorSpeed(void) {
}

void TestLockedRotorPosit(void) {
}

void TestSpeedOffset(void) {
}

void TestMotorVelOver(void) {
}

void TestPostionOver(void) {
}

void TestIbusCurrentOver(void) {
}

void TestUVWCurrentOver(void) {
}

void TestCurrentLoopBandwidth(void) {
  /* 保守版: 10-2500Hz 扫频, 注入 0.3A (Q10=307), 工作点偏置 0.5A (Q10=512) */
  controller_eyou.controller_mode = TEST_MOTOR_CURRENT_MODE;
  controller_eyou.I_q_ref         = 512;
  controller_eyou.foc_run         = 1;

  bw_test_init(&controller_eyou.bw_test, 10.0f, 2500.0f, 307.0f);

  printf("BW test started: 10-2500Hz, 0.3A inject, bias 0.5A\r\n");
}

void TestSpeedLoopBandwidth(void) {
}

void TestFluxIdent(void) {
}

void TestInertiaIdent(void) {
}

void TestPositionLoopBandwidth(void) {
}
