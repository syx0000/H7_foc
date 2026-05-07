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
}

void TestSpeedLoopBandwidth(void) {
}

void TestFluxIdent(void) {
}

void TestInertiaIdent(void) {
}

void TestPositionLoopBandwidth(void) {
}
