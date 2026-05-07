/**
 * @file    func_pid.h
 * @brief
 * @author  yxsui
 * @date    2025-08-01
 * @version 1.1
 */

#ifndef _FUNC_PID_H_
#define _FUNC_PID_H_

#include "foc_bsp.h"
#include "func_filter.h"
#include <stdbool.h>

#define DEFAULT_PID_DATA {NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

///;
// AimValueNowValue PID ;
// Robustness
#define PID_INPUT_LIMIT 2097152

typedef struct IncPID {
  void (*PidInit)(struct IncPID*, uint32_t, uint32_t, uint32_t, uint16_t, int32_t);
  int32_t (*PidRun)(struct IncPID*);

  int32_t NowValue;     //
  int32_t AimValue;     //

  int32_t P;            // Kp (upgraded from int16_t to support large gains)
  int32_t I;            // Ki
  int32_t D;            // Kd
  uint16_t PID_Div;     // PID

  int32_t OutputMax;    //

  int32_t iError;       //
  int32_t LastError;    //
  int32_t PrevError;    //

  int32_t InPut;        //
  int32_t OutPut;       //
} IncPID;

void Init_IncPID(IncPID* pid, uint32_t Kp, uint32_t Ki, uint32_t Kd, uint16_t PID_Div, int32_t OutMax);
int32_t IncPIDCal(IncPID* pid);
int32_t PositionPID(IncPID* pid);

typedef struct {
  float desired;             //< set point
  float error;               //< error
  float prevError;           //< previous error
  float integ;               //< integral
  float deriv;               //< derivative
  float kp;                  //< proportional gain
  float ki;                  //< integral gain
  float kd;                  //< derivative gain
  float outP;                //< proportional output (debugging)
  float outI;                //< integral output (debugging)
  float outD;                //< derivative output (debugging)
  float output;
  float iLimit;              //< integral limit
  float outputLimit;         //< total PID output limit, absolute value. '0' means no limit.
  float dt;                  //< delta-time dt
  bool enableDFilter;        //< filter for D term enable flag
  biquadFilter_t dFilter;    //< filter for D term
  uint8_t useOwnDeriv;
} PidObject;

// pid
void pidInit(PidObject* pid,
                          float kp,
                          float ki,
                          float kd,
                          float iLimit,
                          float outputLimit,
                          float dt,
                          bool enableDFilter,
                          float cutoffFreq,
                          bool useOwnDeriv);
// pid
float pidUpdate(PidObject* pid, float error, float deriv);    //=-
// PID
void pidReset(IncPID* pid);
// PID
void pidResetIntegral(PidObject* pid);
// PID
void pidSetIntegral(PidObject* pid, float integ);

#endif
