/**
 * @file    foc_controller.h
 * @brief   ģ�鹦������
 * @author  yxsui
 * @date    2025-07-31
 * @version 1.0
 */

#ifndef _FOC_POSITION_LOOP_
#define _FOC_POSITION_LOOP_

#include "foc_controller.h"

void foc_position_close_loop(ControllerStruct* controller);

uint8_t InitPositionShowFilter(ControllerStruct* controller);
int32_t PositionShowFilterGoing(ControllerStruct* controller);
int32_t PositionRefFilterGoing(ControllerStruct* controller);

void InitPosSmoothFunc(PositionRefSmooth* p);
int32_t PosSmoothRun(PositionRefSmooth* p, ControllerStruct* controller);

// 位置环带宽测试
void pos_bw_test_init(PositionLoopBWTest* test, float freq_start, float freq_end,
                     float amplitude_base, float f_break);
int32_t pos_bw_test_run(PositionLoopBWTest* test, int32_t position_feedback);
void pos_bw_test_print_results(PositionLoopBWTest* test);

#endif
