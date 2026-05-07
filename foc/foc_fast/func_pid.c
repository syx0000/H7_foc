/**
 * @file    func_pid.c
 * @brief
 * @author  yxsui
 * @date    2025-08-01
 * @version 1.1
 */

#include "func_pid.h"

/*******************************************************************************
  : IncPIDCal
    :
    :
  :
    : PIDUk=Kp*(E(k)-E(k-1))+Ki*E(k)+Kd*[E(k)-2E(k-1)+E(k-2)]
********************************************************************************/
int32_t IncPIDCal(IncPID* pid) {
  int32_t Output;                                 //
  pid->iError = pid->AimValue - pid->NowValue;    //

  if (pid->iError > PID_INPUT_LIMIT) {
    pid->iError = PID_INPUT_LIMIT;
    // printf("PID_INPUT_LIMIT \r\n");
  } else if (pid->iError < -PID_INPUT_LIMIT) {
    pid->iError = -PID_INPUT_LIMIT;
    // printf("-PID_INPUT_LIMIT \r\n");
  }

  pid->InPut = pid->iError;
  // int64 中间变量防溢出：P/I/D 都是 int16，e 是 int32，clamp 后 |e-e_last| 可达 2*PID_INPUT_LIMIT≈4M；
  // 当 P ≥ 512 时 P*(e-e_last) 会超 INT32_MAX，先升 int64 再除回 int32
  int64_t numerator = (int64_t)pid->P * (pid->iError - pid->LastError)
                    + (int64_t)pid->I * pid->iError
                    + (int64_t)pid->D * (pid->iError - 2 * pid->LastError + pid->PrevError);
  Output = (int32_t)(numerator / pid->PID_Div);
  pid->PrevError = pid->LastError;    //
  pid->LastError = pid->iError;
  pid->OutPut += Output;

  if (pid->OutPut > pid->OutputMax)    //
  {
    pid->OutPut = pid->OutputMax;
    // printf("pid->OutputMax \r\n");
  } else if (pid->OutPut < -pid->OutputMax) {
    pid->OutPut = -pid->OutputMax;
    // printf("-pid->OutputMax \r\n");
  }

  return pid->OutPut;    //
}

/*******************************************************************************
  : Init_IncPID
    :
    :
  :
    : PID
********************************************************************************/
void Init_IncPID(IncPID* pid, uint32_t Kp, uint32_t Ki, uint32_t Kd, uint16_t PID_Div, int32_t OutMax) {
  pid->P         = (int32_t)Kp;
  pid->I         = (int32_t)Ki;
  pid->D         = (int32_t)Kd;
  pid->PID_Div   = PID_Div;
  pid->OutputMax = OutMax;
  pid->NowValue  = 0;
  pid->AimValue  = 0;
  pid->iError    = 0;
  pid->LastError = 0;
  pid->PrevError = 0;
  pid->InPut     = 0;
  pid->OutPut    = 0;
}

/*******************************************************************************
  : PositionPID
    :
    :
  :
    : PID
********************************************************************************/
int32_t PositionPID(IncPID* pid) {
  float error      = pid->AimValue - pid->NowValue;
  float derivative = error - pid->PrevError;
  pid->iError += error;
  pid->PrevError = error;

  // I 项限幅（防累积饱和）
  int64_t pidI_64 = (int64_t)pid->I * (int64_t)pid->iError;
  int32_t pidI;
  if (pidI_64 > 20000)
    pidI = 20000;
  else if (pidI_64 < -20000)
    pidI = -20000;
  else
    pidI = (int32_t)pidI_64;

  // 防 int32×float 溢出：先除 DIV 再乘（牺牲精度换安全）
  // 或用 int64 中间变量（保留精度）
  int64_t numerator = (int64_t)pid->P * (int64_t)error
                    + (int64_t)pidI
                    + (int64_t)pid->D * (int64_t)derivative;
  pid->OutPut = (int32_t)(numerator / pid->PID_Div);
  return pid->OutPut;
}

/*******************************************************************************
  : pidInit
    :
    :
  :
    : PID
********************************************************************************/
void pidInit(PidObject* pid,
             float kp,
             float ki,
             float kd,
             float iLimit,
             float outputLimit,
             float dt,
             bool enableDFilter,
             float cutoffFreq,
             bool useOwnDeriv) {
  pid->desired       = 0;
  pid->error         = 0;
  pid->prevError     = 0;
  pid->integ         = 0;
  pid->deriv         = 0;
  pid->kp            = kp;
  pid->ki            = ki;
  pid->kd            = kd;
  pid->outP          = 0;
  pid->outI          = 0;
  pid->outD          = 0;
  pid->iLimit        = iLimit;
  pid->outputLimit   = outputLimit;
  pid->dt            = dt;
  pid->enableDFilter = enableDFilter;

  if (pid->enableDFilter) {
    biquadFilterInitLPF(&pid->dFilter, (1.0f / dt), cutoffFreq);
  }

  pid->useOwnDeriv = useOwnDeriv;
}

/*******************************************************************************
  : pidUpdate
    :
    :
  :
    : pid
********************************************************************************/
float pidUpdate(PidObject* pid, float error, float deriv)    //=-
{
  float output = 0.0f;
  pid->error   = error;
  pid->integ += pid->error * pid->dt;

  //
  if (pid->iLimit != 0) {
    pid->integ = constrainf(pid->integ, -pid->iLimit, pid->iLimit);
  }

  if (pid->useOwnDeriv) {
    pid->deriv = deriv;
  } else {
    pid->deriv = (pid->error - pid->prevError) / pid->dt;
  }

  if (pid->enableDFilter) {
    pid->deriv = biquadFilterApply(&pid->dFilter, pid->deriv);
  }

  pid->outP = pid->kp * pid->error;
  pid->outI = pid->ki * pid->integ;
  pid->outD = pid->kd * pid->deriv;
  output    = pid->outP + pid->outI + pid->outD;

  //
  if (pid->outputLimit != 0) {
    output = constrainf(output, -pid->outputLimit, pid->outputLimit);
  }

  pid->prevError = pid->error;
  pid->output    = output;
  return output;
}

/*******************************************************************************
  : pidReset
    :
    :
  :
    : PID
********************************************************************************/
void pidReset(IncPID* pid) {
  pid->iError    = 0;
  pid->LastError = 0;
  pid->PrevError = 0;
}

/*******************************************************************************
  : pidResetIntegral
    :
    :
  :
    : PID
********************************************************************************/
void pidResetIntegral(PidObject* pid) {
  pid->integ = 0;
}

/*******************************************************************************
  : pidSetIntegral
    :
    :
  :
    : PID
********************************************************************************/
void pidSetIntegral(PidObject* pid, float integ) {
  pid->integ = integ;
}
