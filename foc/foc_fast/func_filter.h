/**
 * @file    func_filter.h
 * @brief
 * @author  yxsui
 * @date    2025-08-01
 * @version 1.1
 */

#ifndef __FUNC_FILTER_H
#define __FUNC_FILTER_H

#include "foc_bsp.h"
#include "func_trigonometric.h"
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MIDFILTERLENGTH 3    //

/**************************** ***********************************/
//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
typedef struct _str_FILTER1    //
{
  void (*Filter1_Init)(void *arg);
  void (*Filter1)(void *arg);

  uint32_t Ts;        // us
  uint32_t Tc;        // us0~320000us

  uint32_t Ka_Q20;    //(Q20)
  uint32_t Kb_Q20;    //(Q20)

  int32_t InPut;      //
  int32_t OutPut;     //

  int64_t OutPut1;    //
  int64_t OutPut0;    //

  int64_t InPut1;     //
  int64_t InPut0;     //

} str_FILTER1;

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
typedef str_FILTER1* Filter1_Phandle;
extern str_FILTER1 Filter1_Defaults;

/*******************************************************************************
 * ********************************************************************************/

typedef struct movingAverage_s16 {
  void (*FilterInit)(struct movingAverage_s16*, uint16_t);
  void (*FilterRun)(struct movingAverage_s16*, int16_t);
  int16_t* buffer;  /**< Data buffer pointer */
  uint16_t size;    /**< Size of filter buffer */
  uint16_t index;   /**< Current location in buffer */
  uint16_t fill;    /**< Buffer filled level */
  int32_t sum;      /**< Cumulative value of buffer */
  int16_t filtered; /**< Filtered output */
} movingAverage_s16t;

typedef struct movingAverage_s32 {
  void (*FilterInit)(struct movingAverage_s32*, uint16_t);
  void (*FilterRun)(struct movingAverage_s32*, int32_t);
  int32_t* buffer;  /**< Data buffer pointer */
  uint16_t size;    /**< Size of filter buffer */
  uint16_t index;   /**< Current location in buffer */
  uint16_t fill;    /**< Buffer filled level */
  int32_t sum;      /**< Cumulative value of buffer */
  int32_t filtered; /**< Filtered output */
} movingAverage_s32t;

#define DEFAULT_FILTER_DATA {0}
/*******************************************************************************
 * ********************************************************************************/
typedef struct {
  int32_t MaxVelAccEveryPrd;
  int32_t NowVelocityRef;
  int32_t OldVelocityRef;

} SpeedLoopSmooth;

#define DEFAULT_SPEED_SMOOTH_FILTER {0}
/*******************************************************************************
 * ********************************************************************************/
typedef struct {
  int16_t MaxCurAccEveryPrd;
  int16_t NowCurrentRef;
  int16_t OldCurrentRef;

} CurrentLoopSmooth;
/*******************************************************************************
 * ********************************************************************************/
/* 梯形位置规划状态 (Trapezoidal motion profile, V/A 双限).
 * 速度 / 加速度均以"位置 LSB / 位置环 ISR tick"为单位 (位置环 Ts ≈ 400us @ 2.5kHz).
 * 单位换算:
 *   V[LSB/tick]  = V[output_rpm]    * 6 * 1024 * Ts  ≈ V[output_rpm]    * 2.4576
 *   A[LSB/tick²] = A[output_rpm/s]  * 6 * 1024 * Ts² ≈ A[output_rpm/s]  * 9.830e-4
 */
typedef struct {
  /* 配置参数 (InitPosSmoothFunc 设默认值, vmax/amax CLI 实时改) */
  float v_max;                   // 峰值速度 (LSB/tick, 正数)
  float a_max;                   // 加速度  (LSB/tick², 正数)
  /* 状态 */
  float   cur_pos;               // float 累加, 防 sub-LSB 速度被截断为 0
  float   cur_v;
  int32_t old_pos_ref;           // 检测 position_ref 变化
  uint8_t active;                // 1: 斜坡进行中; 0: 已到位/未启动
  uint16_t cooldown;             // snap 后冷却计数, >0 时不重启规划
} PositionRefSmooth;
#define DEFAULT_CURRENT_SMOOTH_FILTER {0}
#define DEFAULT_POS_SMOOTH_FILTER {0}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
typedef struct _str_FILTER1B {
  void (*Filter1B_Init)(void *arg);
  void (*Filter1B)(void *arg);

  int32_t Ts;         //,Unit d
  int32_t Tc;         //,?n*d

  int32_t Ka_Q26;     //(Q20)
  int32_t Kb_Q26;     //(Q20)

  int32_t InPut;      //
  int32_t OutPut;     //

  int64_t OutPut1;    //
  int64_t OutPut0;    //

  int64_t InPut1;     //

} str_FILTER1B;

typedef str_FILTER1B* Filter1B_Phandle;

#define Filter1B_Defaults {(void (*)(long))Filter1B_Init, (void (*)(long))Filter1B}

//---------------------------------------------------------------------------
typedef struct _str_AvrFilter {    //
  int32_t (*AvrageFilter)(void *arg);
  int32_t RefOut_Last;
  int16_t Num;
  int32_t RefOld;
  int32_t DeltaRef;
  int32_t EndRef[33];
  int32_t errEndRef[33];

} str_AvrFilter;

typedef str_AvrFilter* AvrFilter_Phandle;

#define AvrFilter_Defaults {(int32(*)(long))AvrageFilter}

//---------------------------------------------------------------------
typedef struct _str_NotchFilter {
  uint16_t SampFreq;     //
  uint16_t NotchFreq;    //
  uint16_t DeltaF;       // 3dB
  int32_t Num[3];        //
  int32_t Den[3];        //
  int32_t Input[3];      //
  int32_t Output[3];     //
} str_NotchFilter;

// #define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - butterworth*/
#define BIQUAD_Q 0.70710678f

// 1.
typedef struct {
  float LastP;    // 0.02
  float NOWP;     // 0
  float out;      // 0
  float Kg;       // 0
  float Q;        // 0.05
  float R;        // 0.99
} KFP;            //

typedef struct pt1Filter_s {
  float state;
  float k;
} pt1Filter_t;

typedef enum {
  FILTER_LPF,    // 2nd order Butterworth section
  FILTER_NOTCH,
  FILTER_BPF,
} biquadFilterType_e;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
} biquadFilter_t;

/**
 * @brief filter object initialization
 *
 * @param context [in] instance of filter object
 * @param filter_size [in] size of filter buffer
 * @param sample_time [in] filter sampling time in ms
 */
void moving_average_create_s16(movingAverage_s16t* context, uint16_t filter_size);
void moving_average_create_s32(movingAverage_s32t* context, uint16_t filter_size);
/**
 * @brief filter process function
 *
 * @param context [in] instance of filter object
 * @param input [in] data sample to filter
 */
void moving_average_filter_s16(movingAverage_s16t* context, int16_t input);
void moving_average_filter_s32(movingAverage_s32t* context, int32_t filter_input);

//------------------------------------------------------------------
typedef struct {
  int32_t Input[MIDFILTERLENGTH];    //
  int32_t Output;                    //
} str_MidFilter;

void Filter1_Init(void *arg);
void Filter1(void *arg);

void Filter1B_Init(str_FILTER1B* p);
void Filter1B(str_FILTER1B* p);

int32_t AvrageFilter(str_AvrFilter* p, int32_t ref, uint16_t n);
//

void NotchFilter_Init(str_NotchFilter* p, uint16_t Type);
void NotchFilter(str_NotchFilter* p);

float kalmanFilter(KFP* kfp, float input);
float filterGetNotchQ(float centerFreq, float cutoffFreq);
//
void biquadFilterUpdate(
    biquadFilter_t* filter, uint16_t filterFreq, uint16_t refreshRate, float Q, biquadFilterType_e filterType);
//
void biquadFilterInitNotch(biquadFilter_t* filter,
                                        uint16_t samplingFreq,
                                        uint16_t filterFreq,
                                        uint16_t cutoffHz);
//
void biquadFilterInitLPF(biquadFilter_t* filter, uint16_t filterFreq, uint16_t samplingFreq);
// Computes a biquad_t filter on a sample
float biquadFilterApply(biquadFilter_t* filter, float input);
void biquadFilterInit(
    biquadFilter_t* filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType);
// pt1( )
float pt1FilterGain(float f_cut, float dT);
// pt1
void pt1FilterInit(pt1Filter_t* filter, float k);
//
void pt1FilterUpdateCutoff(pt1Filter_t* filter, float k);
// pt1
float pt1FilterApply(pt1Filter_t* filter, float input);

#ifdef __cplusplus
}
#endif

/* Private_Constants ---------------------------------------------------------*/

/* Private_TypesDefinitions --------------------------------------------------*/

/* Exported_Functions --------------------------------------------------------*/

#endif

/********************************* END OF FILE *********************************/
