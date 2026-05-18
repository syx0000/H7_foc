/**
 * @file    foc_kernel.c
 * @brief
 * @author  yxsui
 * @date    2025-08-01
 * @version 1.1
 */

#include "foc_kernel.h"

/*******************************************************************************
  : Get_sincos_value
    :
    : sincos
  :
    :
********************************************************************************/
Trig_Components get_sincos_value(int32_t Angle) {
  uint32_t uhindex = Angle;
  Trig_Components Local_Components;
  uhindex /= 64;

  switch ((uint16_t)(uhindex)&SIN_MASK) {
  case U0_90:
    Local_Components.hSin = hSin_Cos_Table[(uint8_t)(uhindex)];
    Local_Components.hCos = hSin_Cos_Table[(uint8_t)(0xFFu - (uint8_t)(uhindex))];
    break;

  case U90_180:
    Local_Components.hSin = hSin_Cos_Table[(uint8_t)(0xFFu - (uint8_t)(uhindex))];
    Local_Components.hCos = -hSin_Cos_Table[(uint8_t)(uhindex)];
    break;

  case U180_270:
    Local_Components.hSin = -hSin_Cos_Table[(uint8_t)(uhindex)];
    Local_Components.hCos = -hSin_Cos_Table[(uint8_t)(0xFFu - (uint8_t)(uhindex))];
    break;

  case U270_360:
    Local_Components.hSin = -hSin_Cos_Table[(uint8_t)(0xFFu - (uint8_t)(uhindex))];
    Local_Components.hCos = hSin_Cos_Table[(uint8_t)(uhindex)];
    break;

  default:
    break;
  }

  return Local_Components;    // sin,cos0-327680-1.
}

/*******************************************************************************
  : Rev_Park
    : @d d
          @q q
          @Theta
    : @*Valpha alpha
          @*Vbeta Vbeta
  :
    : PARK
          Valapha=Ud*cos(theta)-Uq*sin(theta)
          Vbeta=Ud*sin(theta)+Uq*cos(theta)
********************************************************************************/
void rev_park(int32_t d, int32_t q, int32_t Theta, int32_t* Valpha, int32_t* Vbeta) {
  int32_t alpha_tmp1, alpha_tmp2, beta_tmp1, beta_tmp2;
  Trig_Components Local_Vector_Components;
  Local_Vector_Components = get_sincos_value(Theta);
  alpha_tmp1              = (q * (int32_t)Local_Vector_Components.hCos) / 32768;
  alpha_tmp2              = (d * (int32_t)Local_Vector_Components.hSin) / 32768;
  *Vbeta                  = (int32_t)((alpha_tmp1 + alpha_tmp2));    // 32768Q15
  beta_tmp1               = (q * (int32_t)Local_Vector_Components.hSin) / 32768;
  beta_tmp2               = (d * (int32_t)Local_Vector_Components.hCos) / 32768;
  *Valpha                 = (int32_t)((beta_tmp2 - beta_tmp1));
}

/*******************************************************************************
  : clarke_transf
    : @d a
          @q b
    : @*alpha alpha
          @*beta beta
  :
    : CLARK
          Ialpha=Ia=-Ib-Ic
          Ibata=(Ib-Ic)*sqrt(3)/3
********************************************************************************/
void clarke_transf(int32_t I_a, int32_t I_b, int32_t* alpha, int32_t* beta) {
  *alpha = I_a;
  int64_t a_tmp = (int64_t)divSQRT_3 * I_a;
  int64_t b_tmp = (int64_t)divSQRT_3 * I_b;
  *beta = (int32_t)((a_tmp + b_tmp + b_tmp) / 32768);
}

/*******************************************************************************
  :park_transf
    :
    : dq
  :
    : park
          Id=Ialpha*cos(theta)+Ibeta*sin(theta)
          Iq=-Ialpha*sin(theta)+Ibata*cos(theta)
********************************************************************************/
void park_transf(int32_t I_alpha, int32_t I_beta, int32_t* d, int32_t* q, int32_t theta) {
  Trig_Components Local_Vector_Components;
  Local_Vector_Components = get_sincos_value(theta);
  int64_t d_tmp1 = (int64_t)I_alpha * Local_Vector_Components.hCos;
  int64_t d_tmp2 = (int64_t)I_beta  * Local_Vector_Components.hSin;
  *d = (int32_t)((d_tmp1 + d_tmp2) / 32768);
  int64_t q_tmp1 = (int64_t)I_alpha * Local_Vector_Components.hSin;
  int64_t q_tmp2 = (int64_t)I_beta  * Local_Vector_Components.hCos;
  *q = (int32_t)((q_tmp2 - q_tmp1) / 32768);
}

/*******************************************************************************
  : svpwm_calc
    :
    : timer ccr
  :
    :
********************************************************************************/
void svpwm_calc(int32_t Valpha, int32_t Vbeta, uint32_t* CCR1, uint32_t* CCR2, uint32_t* CCR3) {
  int32_t sector;
  uint32_t Tcmp1, Tcmp2, Tcmp3;    // float
  int32_t T_x, T_y, f_temp, Ta, Tb, Tc;
  sector = 0;
  Tcmp1  = 0;
  Tcmp2  = 0;
  Tcmp3  = 0;
  //
  uint16_t PWMDIV = 1024;

  if (Vbeta > 0) {
    sector = 1;
  }

  if (((divSQRT_3_2 * Valpha) / PWMDIV - (div1_2 * Vbeta) / PWMDIV) > 0) {
    sector += 2;
  }

  if ((-divSQRT_3_2 * Valpha) / PWMDIV - (div1_2 * Vbeta) / PWMDIV > 0) {
    sector += 4;
  }

  switch (sector) {
  case 1:
    T_x = ((-3 * (div1_2 * Valpha) / PWMDIV + (divSQRT_3_2 * Vbeta) / PWMDIV)) * (PWM_T / UDC);    // Tx  *Q10*Q16/Q10
    T_y = ((3 * div1_2 * Valpha) / PWMDIV + (divSQRT_3_2 * Vbeta) / PWMDIV) * (PWM_T / UDC);       // Ty  *Q10*Q16/Q10
    break;

  case 2:
    T_x = ((3 * div1_2 * Valpha) / PWMDIV + (divSQRT_3_2 * Vbeta) / PWMDIV) * (PWM_T / UDC);
    T_y = -((2 * divSQRT_3_2 * Vbeta)) / PWMDIV * (PWM_T / UDC);
    break;

  case 3:
    T_x = -((-3 * div1_2 * Valpha) / PWMDIV + (divSQRT_3_2 * Vbeta) / PWMDIV) * (PWM_T / UDC);
    T_y = ((2 * divSQRT_3_2 * Vbeta)) / PWMDIV * (PWM_T / UDC);
    break;

  case 4:
    T_x = (-(2 * divSQRT_3_2 * Vbeta)) / PWMDIV * (PWM_T / UDC);
    T_y = ((-3 * div1_2 * Valpha) / PWMDIV + (divSQRT_3_2 * Vbeta) / PWMDIV) * (PWM_T / UDC);
    break;

  case 5:
    T_x = ((2 * divSQRT_3_2 * Vbeta)) / PWMDIV * (PWM_T / UDC);
    T_y = -((3 * div1_2 * Valpha) / PWMDIV + (divSQRT_3_2 * Vbeta) / PWMDIV) * (PWM_T / UDC);
    break;

  default:
    T_x = -((3 * div1_2 * Valpha) / PWMDIV + (divSQRT_3_2 * Vbeta) / PWMDIV) * (PWM_T / UDC);
    T_y = -((-3 * div1_2 * Valpha) / PWMDIV + (divSQRT_3_2 * Vbeta) / PWMDIV) * (PWM_T / UDC);    // Q15
    break;
  }

  //
  f_temp = (T_x + T_y);

  if ((f_temp / 32768) > (int32_t)PWM_T) {
    // T_x /= f_temp;
    // T_y /= (T_x + T_y);
    T_x = T_x * PWM_T / f_temp;
    T_y = T_y * PWM_T / f_temp;
  }    // Q15

  Ta = (PWM_T - (T_x + T_y) / 32768) / 4;
  Tb = (T_x / 65536 + Ta);
  Tc = (T_y / 65536 + Tb);

  switch (sector) {
  case 1:
    Tcmp1 = Tb;
    Tcmp2 = Ta;
    Tcmp3 = Tc;
    break;

  case 2:
    Tcmp1 = Ta;
    Tcmp2 = Tc;
    Tcmp3 = Tb;
    break;

  case 3:
    Tcmp1 = Ta;
    Tcmp2 = Tb;
    Tcmp3 = Tc;
    break;

  case 4:
    Tcmp1 = Tc;
    Tcmp2 = Tb;
    Tcmp3 = Ta;
    break;

  case 5:
    Tcmp1 = Tc;
    Tcmp2 = Ta;
    Tcmp3 = Tb;
    break;

  case 6:
    Tcmp1 = Tb;
    Tcmp2 = Tc;
    Tcmp3 = Ta;
    break;
  }

  *CCR1 = (uint32_t)Tcmp1;
  *CCR2 = (uint32_t)Tcmp2;
  *CCR3 = (uint32_t)Tcmp3;
}

/*******************************************************************************
  : limit_norm
    :
    :
  :
    :
********************************************************************************/
void limit_norm(int32_t* x, int32_t* y, int16_t limit) {
  uint16_t norm = qsqrt(*x * *x + *y * *y);

  if (norm > limit) {
    *x = *x * limit / norm;
    *y = *y * limit / norm;
  }
}
