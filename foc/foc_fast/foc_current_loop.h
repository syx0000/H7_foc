/**
 * @file    foc_current_loop.h
 * @brief
 * @author  yxsui
 * @date    2025-07-31
 * @version 1.0
 */

#ifndef _FOC_CURRENT_LOOP_
#define _FOC_CURRENT_LOOP_

#include "foc_controller.h"
#include "ifly_fault_api.h"
#include <math.h>
#include <stdbool.h>
//
typedef struct {
  float sample_buffer[50];    // 50ms
  float rms_buffer[20];       // 1
  uint8_t sample_index;       //
  uint32_t window_count;      //
  uint8_t fault_count;        //
} SimpleOverCurrentDetector;

void foc_current_close_loop(ControllerStruct* controller);
uint8_t phase_current_sample(ControllerStruct* controller);
uint8_t phase_current_sample_Check(ControllerStruct* controller, uint16_t IaSampleValue, uint16_t IbSampleValue);

uint8_t InitCurrentShowFilter(ControllerStruct* controller);
int32_t ShowFilterGoing(ControllerStruct* controller, str_FILTER1* ShowFilter);
void CurrentLoopSmoothInit(CurrentLoopSmooth* CurrentSmooth);
int16_t CurrentLoopSmoothRun(int16_t IqRef, CurrentLoopSmooth* CurrentSmooth);
// void check_u_phase_overcurrent_simple(ControllerStruct* controller);
void check_phases_overcurrent_timesliced(ControllerStruct* controller);                      //
void process_single_phase(SimpleOverCurrentDetector* detector, float current, int phase);    //
float sliding_avg_filter(float *buf, uint8_t depth, uint8_t *idx, float new_val, uint8_t filter_valid_cnt);
void weak_magn_control(ControllerStruct* controller);
void deadtime_compensation(ControllerStruct* controller);
void deadtime_compensation_3phase(ControllerStruct* controller);

// 电流环带宽测试
void bw_test_init(CurrentLoopBWTest* test, float freq_start, float freq_end, float amplitude);
int16_t bw_test_run(CurrentLoopBWTest* test, int32_t iq_feedback);
void bw_test_print_results(CurrentLoopBWTest* test);

// 电感辨识 (ISR同步)
void ident_inductance_init(InductanceIdent* ident, uint8_t axis, float freq, float amplitude, float Rs);
void ident_inductance_compute(InductanceIdent* ident);
#endif
