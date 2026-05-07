/**
 * @file    func_filter.c
 * @brief
 * @author  yxsui
 * @date    2025-08-01
 * @version 1.1
 */

#include "func_filter.h"
//========================================

//====================================
/*******************************************************************************
  :
    :
    :
  :
    :
********************************************************************************/
//
int32_t AvrageFilter(str_AvrFilter* p, int32_t ref, uint16_t n) {
  int32_t RefOut = 0;
  uint16_t temp;
  if ((n < 6) && (n > 0)) {
    temp = 1 << n;
    if (p->Num < 0)
      p->Num = temp;
    //
    p->RefOld            = ref;
    p->DeltaRef          = p->RefOld >> n;
    p->errEndRef[p->Num] = p->RefOld - (1 << n) * p->DeltaRef;
    p->EndRef[p->Num]    = p->errEndRef[p->Num] + p->DeltaRef;
    if (p->Num > 1)
      RefOut = (int32_t)((int64_t)p->RefOut_Last + p->DeltaRef + p->errEndRef[p->Num - 2] - p->EndRef[p->Num - 1]);
    else if (p->Num == 1)
      RefOut = (int32_t)((int64_t)p->RefOut_Last + p->DeltaRef + p->errEndRef[temp] - p->EndRef[0]);
    else if (p->Num == 0)
      RefOut = (int32_t)((int64_t)p->RefOut_Last + p->DeltaRef + p->errEndRef[temp - 1] - p->EndRef[temp]);
    // RefOut       = RefOut_Last + DeltaRef+ errEndRef[(1<<n)-1] - EndRef[1<<n];
    p->RefOut_Last = RefOut;
    p->Num--;    //
    return RefOut;
  } else
    return ref;
}
/*******************************************************************************
  : moving_average_create_s16()
    :
    :
  :
    :
********************************************************************************/
void moving_average_create_s16(movingAverage_s16t* context, uint16_t filter_size) {
  free(context->buffer);
  context->size     = filter_size;
  context->buffer   = (int16_t*)malloc(filter_size * sizeof(int16_t));
  context->index    = 0;
  context->sum      = 0;
  context->fill     = 0;
  context->filtered = 0;
}
/*******************************************************************************
  :int moving_average_filter_s16()
    :
    :
  :
    :
********************************************************************************/
void moving_average_filter_s16(movingAverage_s16t* context, int16_t filter_input) {
  if (context->fill == context->size) {
    context->sum -= context->buffer[context->index];
  }
  context->buffer[context->index] = filter_input;
  context->sum += context->buffer[context->index];
  context->index++;
  if (context->index >= context->size) {
    context->index = 0;
  }
  if (context->fill < context->size) {
    context->fill++;
  }
  context->filtered = (int16_t)(context->sum / context->fill);
}
/*******************************************************************************
  : moving_average_create_s16()
    :
    :
  :
    :
********************************************************************************/
void moving_average_create_s32(movingAverage_s32t* context, uint16_t filter_size) {
  free(context->buffer);
  context->size     = filter_size;
  context->buffer   = (int32_t*)malloc(filter_size * sizeof(int32_t));
  context->index    = 0;
  context->sum      = 0;
  context->fill     = 0;
  context->filtered = 0;
}
/*******************************************************************************
  : int moving_average_filter_s16()
    :
    :
  :
    :
********************************************************************************/
void moving_average_filter_s32(movingAverage_s32t* context, int32_t filter_input) {
  if (context->fill == context->size) {
    context->sum -= context->buffer[context->index];
  }
  context->buffer[context->index] = filter_input;
  context->sum += context->buffer[context->index];
  context->index++;
  if (context->index >= context->size) {
    context->index = 0;
  }
  if (context->fill < context->size) {
    context->fill++;
  }
  context->filtered = (int32_t)(context->sum / context->fill);
}

//==========================================================================
//   Filter1_Init()
//     Ts;
//           Tc;
//   :   Ka;
//           Kb
// :
//==========================================================================

void Filter1_Init(void *arg) {
  str_FILTER1* p = (str_FILTER1 *)arg;
  if (p->Tc >= p->Ts) {
    p->Ka_Q20 = ((int32_t)p->Ts << 20) / ((p->Tc << 1) + p->Ts);
    if (p->Ka_Q20 < 400)                          // Tc < 1600 * Ts
      p->Ka_Q20 = 400;
    p->Kb_Q20 = (1L << 20) - (p->Ka_Q20 << 1);    // Q20=2^20
  } else {
    p->Ka_Q20 = 0;
    p->Kb_Q20 = 0;
  }
  // p->Ka = 31;
  // p->Kb = 962;
}

//==========================================================================
//   Filter1B_Init()
//     Ts;
//           Tc;
//   :   Ka;
//           Kb
// :
//==========================================================================

void Filter1B_Init(str_FILTER1B* p) {
  if (p->Tc >= p->Ts) {
    //... Ka=1/(Tc+Ts) @Q26
    p->Ka_Q26 = ((int32_t)p->Ts << 26) / (p->Tc + p->Ts);
    if (p->Ka_Q26 < 1) {
      p->Ka_Q26 = 1;
    }
    //... Kb=1-1/(Tc+Ts) @Q20
    p->Kb_Q26 = (1L << 26) - (p->Ka_Q26);
  } else {
    p->Ka_Q26 = 0;
    p->Kb_Q26 = 0;
  }
}

//==========================================================================
//   Filter1()
//     Ka;
//           Kb
//           InPut;
//
//   :   OutPut
// :
//==========================================================================

void Filter1(void *arg) {
  str_FILTER1* p = (str_FILTER1 *)arg;
  if ((p->Ka_Q20 == 0) && (p->Kb_Q20 == 0)) {
    p->OutPut = p->InPut;
    return;
  }
  p->InPut1  = (int64_t)p->InPut << 9;
  p->OutPut1 = ((int64_t)p->Kb_Q20 * p->OutPut0 + (int64_t)p->Ka_Q20 * (p->InPut1 + p->InPut0)) >> 20;
  p->OutPut0 = p->OutPut1;
  p->InPut0  = p->InPut1;
  p->OutPut  = p->OutPut1 >> 9;
}

//==========================================================================
//   Filter1B()
//     Ka;
//           Kb
//           InPut;
//
//   :   OutPut
// :
//==========================================================================

void Filter1B(str_FILTER1B* p) {
  if ((p->Ka_Q26 == 0) && (p->Kb_Q26 == 0)) {
    p->OutPut = p->InPut;
  } else {
    //... y(n)=Kb*y(n-1)+Ka*x(n)
    p->InPut1  = (int64_t)p->InPut << 9;
    p->OutPut1 = ((int64_t)p->Kb_Q26 * p->OutPut0 + (int64_t)p->Ka_Q26 * p->InPut1) >> 26;
    p->OutPut0 = p->OutPut1;
    p->OutPut  = p->OutPut1 >> 9;
  }
}

//--------------------------------------------------zzsong4----------------------------------------
// 2.
// KFP KFP_X={0.02,0,0,0,0.001,0.005};

/**
 *
 *@param KFP *kfp
 *   float input
 *@return
 */
float kalmanFilter(KFP* kfp, float input) {
  // k = k-1 +
  kfp->NOWP = kfp->LastP + kfp->Q;
  // = k / k +
  kfp->Kg = kfp->NOWP / (kfp->NOWP + kfp->R);
  // k =  +  *  -
  kfp->out = kfp->out + kfp->Kg * (input - kfp->out);    //
  //:  kfp->LastP
  kfp->LastP = (1 - kfp->Kg) * kfp->NOWP;
  return kfp->out;
}

// PT1
// pt1( )
float pt1FilterGain(float f_cut, float dT) {
  float RC = 1 / (2 * M_PIf * f_cut);
  return dT / (RC + dT);
}

// pt1
void pt1FilterInit(pt1Filter_t* filter, float k) {
  filter->state = 0.0f;
  filter->k     = k;
}

//
void pt1FilterUpdateCutoff(pt1Filter_t* filter, float k) {
  filter->k = k;
}

// pt1
float pt1FilterApply(pt1Filter_t* filter, float input) {
  float temp = input - filter->state;
  if (temp < -220 || temp > 220)
    filter->state = filter->state + filter->k * temp;    //??
  else
    filter->state = filter->state + temp;
  return filter->state;
}

// Q(f0)(f1)
// Q = f0 / (f2 - f1);F2 = f02/ f1
//
float filterGetNotchQ(float centerFreq, float cutoffFreq) {
  return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

//
void biquadFilterInitLPF(biquadFilter_t* filter, uint16_t filterFreq, uint16_t samplingFreq) {
  biquadFilterInit(filter, filterFreq, samplingFreq, BIQUAD_Q, FILTER_LPF);
}

void biquadFilterInit(
    biquadFilter_t* filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType) {
  biquadFilterUpdate(filter, filterFreq, refreshRate, Q, filterType);
  // zero initial samples
  filter->x1 = filter->x2 = 0;
  filter->y1 = filter->y2 = 0;
}

//
void biquadFilterUpdate(
    biquadFilter_t* filter, uint16_t filterFreq, uint16_t refreshRate, float Q, biquadFilterType_e filterType) {
  // setup variables
  const float omega = 2.0f * M_PIf * filterFreq * refreshRate * 0.000001f;
  const float sn    = sin_approx(omega);
  const float cs    = cos_approx(omega);
  const float alpha = sn / (2.0f * Q);
  switch (filterType) {
  case FILTER_LPF:
    // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
    // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
    filter->b1 = 1 - cs;
    filter->b0 = filter->b1 * 0.5f;
    filter->b2 = filter->b0;
    filter->a1 = -2 * cs;
    filter->a2 = 1 - alpha;
    break;
  case FILTER_NOTCH:
    filter->b0 = 1;
    filter->b1 = -2 * cs;
    filter->b2 = 1;
    filter->a1 = filter->b1;
    filter->a2 = 1 - alpha;
    break;
  case FILTER_BPF:
    filter->b0 = alpha;
    filter->b1 = 0;
    filter->b2 = -alpha;
    filter->a1 = -2 * cs;
    filter->a2 = 1 - alpha;
    break;
  }
  const float a0 = 1 + alpha;
  // precompute the coefficients
  filter->b0 /= a0;
  filter->b1 /= a0;
  filter->b2 /= a0;
  filter->a1 /= a0;
  filter->a2 /= a0;
  // update weight
  /// filter->weight = weight;
}

//(   )
void biquadFilterInitNotch(biquadFilter_t* filter, uint16_t samplingFreq, uint16_t filterFreq, uint16_t cutoffHz) {
  float Q = filterGetNotchQ(filterFreq, cutoffHz);
  biquadFilterUpdate(filter, samplingFreq, filterFreq, Q, FILTER_NOTCH);
}

/* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in
 * coefficients */
float biquadFilterApply(biquadFilter_t* filter, float input) {
  const float result = filter->b0 * input + filter->x1;
  filter->x1         = filter->b1 * input - filter->a1 * result + filter->x2;
  filter->x2         = filter->b2 * input - filter->a2 * result;
  return result;
}

str_FILTER1 Filter1_Defaults = {.Filter1_Init = Filter1_Init,
                                .Filter1      = Filter1,
                                .Ts           = 0,
                                .Tc           = 0,
                                .Ka_Q20       = 0,
                                .Kb_Q20       = 0,
                                .InPut        = 0,
                                .OutPut       = 0,
                                .OutPut1      = 0,
                                .OutPut0      = 0,
                                .InPut1       = 0,
                                .InPut0       = 0};
