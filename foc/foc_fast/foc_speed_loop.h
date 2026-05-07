/**
 * @file    foc_speed_loop.h
 * @brief
 * @author  yxsui
 * @date    2025-07-31
 * @version 1.0
 */

#ifndef _FOC_SPEED_LOOP_
#define _FOC_SPEED_LOOP_

#include "foc_controller.h"
// #include "ifly_fault_api.h"

void foc_velocity_close_loop(ControllerStruct* controller);
void speedOverMonitor(ControllerStruct* controller);
void speedOffsetMonitor(ControllerStruct* controller);
int32_t SpeedShowFilterGoing(ControllerStruct* controller, str_FILTER1* ShowFilter);
uint8_t InitSpeedShowFilter(str_FILTER1* ShowFilter);
void SpeedLoopSmoothInit(SpeedLoopSmooth* SpeedSmooth);
int32_t SpeedLoopSmoothRun(int32_t VelocityRef, SpeedLoopSmooth* SpeedSmooth);

// 速度环带宽测试
void spd_bw_test_init(SpeedLoopBWTest* test, float freq_start, float freq_end,
                     float amplitude_base, float f_break);
int32_t spd_bw_test_run(SpeedLoopBWTest* test, int32_t speed_feedback);
void spd_bw_test_print_results(SpeedLoopBWTest* test);

#endif
