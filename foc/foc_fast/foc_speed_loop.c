/**
 * @file    foc_speed_loop.c
 * @brief
 * @author  yxsui
 * @date    2025-07-31
 * @version 1.0
 */

#include "foc_speed_loop.h"
#include "ifly_fault_api.h"
#include "foc_api.h"
#include <math.h>
extern Portection_Value Threshold;
extern ControllerStruct controller_eyou;
extern ifly_Err_Pro_Type motorProValue;

/*******************************************************************************
  :foc_velocity_close_loop
    :
    :
  :
    :
********************************************************************************/
void foc_velocity_close_loop(ControllerStruct* controller) {
  /* MIT PD 模式直接算 Iq, 跳过斜坡和速度 PID */
  if (controller->controller_mode == MIT_PD_MODE) {
    static float vel_filt = 0.0f;
    float pos_cur = (float)controller->real_position_out / (1024.0f * 180.0f / 3.14159265f);
    float vel_raw = (float)controller->dtheta_mech_out / 1024.0f * (2.0f * 3.14159265f / 60.0f);
    vel_filt += 0.05f * (vel_raw - vel_filt);
    float pos_err = controller->mit_p_des - pos_cur;
    float vel_err = controller->mit_v_des - vel_filt;
    /* 位置死区: 消除减速箱间隙引起的极限环 (±0.003 rad ≈ 0.17° 输出端) */
    if (pos_err >  0.003f) pos_err -= 0.003f;
    else if (pos_err < -0.003f) pos_err += 0.003f;
    else pos_err = 0.0f;
    float iq_out = controller->mit_kp * pos_err
                 + controller->mit_kd * vel_err
                 + controller->mit_t_ff;
    int32_t iq_q10 = (int32_t)(iq_out * 1024.0f);
    int32_t max_cur = (int32_t)controller->FlashData.MaxCurrent;
    if (iq_q10 >  max_cur) iq_q10 =  max_cur;
    if (iq_q10 < -max_cur) iq_q10 = -max_cur;
    controller->I_q_ref = iq_q10;
    return;
  }

  set_velocity_ref_loop(controller->velocity_ref);
  //
#if USE_SPEED_LOOP_SMOOTH
  // if (abs(controller->velocity_ref - controller->dtheta_mech) > 1024*10)
  // {
  controller->velocity_ref_filterd = SpeedLoopSmoothRun(controller->velocity_ref, &controller->SpeedSmooth);
  // }
#else
  controller->velocity_ref_filterd = controller->velocity_ref;
#endif

  // 速度环带宽测试信号注入（斜坡滤波之后、PID 之前）
  int32_t spd_sweep = spd_bw_test_run(&controller->spd_bw_test, controller->dtheta_mech);
  controller->velocity_ref_filterd += spd_sweep;

  if ((controller->controller_mode == PROFILE_VELOCITY_MOCE ||
       controller->controller_mode == CYCLIC_SYNC_VELOCITY_MODE ||
       controller->controller_mode == PROFILE_POSITION_MODE ||
       controller->controller_mode == CYCLIC_SYNC_POSITION_MODE)) {
    // PID
    int32_t TempIq = 0;
    #if USE_SPEED_NOTCH
    int32_t spd_fb = (int32_t)biquadFilterApply(&controller->speed_notch, (float)controller->dtheta_mech);
    controller->IncPID_Speed.NowValue = spd_fb;
    #else
    controller->IncPID_Speed.NowValue = controller->dtheta_mech;
    #endif
    controller->IncPID_Speed.AimValue = controller->velocity_ref_filterd;
    controller->IncPID_Speed.PidRun(&controller->IncPID_Speed);
    TempIq              = controller->IncPID_Speed.OutPut;
    controller->I_q_ref = TempIq;
    // controller->I_d_ref = 0;
  }
}

/*******************************************************************************
  :InitSpeedShowFilter
    :
    :
  :
    :
********************************************************************************/
uint8_t InitSpeedShowFilter(str_FILTER1* ShowFilter) {
  // 1msus-----------IS620 1ms
  ShowFilter->Ts = 1000L;
  // 50msus---------IS620 50ms
  ShowFilter->Tc = 80000L;
  // ka,kb
  ShowFilter->Filter1_Init(ShowFilter);
  return 0;
}

/*******************************************************************************
  :SpeedShowFilterGoing
    :
    :
  :
    :
********************************************************************************/
int32_t SpeedShowFilterGoing(ControllerStruct* controller, str_FILTER1* ShowFilter) {
  ShowFilter->InPut = controller->dtheta_mech;
  ShowFilter->Filter1(ShowFilter);
  return (int32_t)ShowFilter->OutPut;    //
}

/*******************************************************************************
  : SpeedLoopSmoothInit
    : int32_t VelocityRef 1/1024 rpm
          uint16_t SmoothTime ms
    :
  :
    :
********************************************************************************/
void SpeedLoopSmoothInit(SpeedLoopSmooth* SpeedSmooth) {
  SpeedSmooth->MaxVelAccEveryPrd = DEFAULT_MAX_SPEED / (MIN_ACC_TIME);
  SpeedSmooth->NowVelocityRef    = 0;
  SpeedSmooth->OldVelocityRef    = 0;
  return;
}

/*******************************************************************************
  :SpeedLoopSmoothRun
    : int32_t VelocityRef 1/1024 rpm
          uint16_t SmoothTime ms
    :
  :
    :
********************************************************************************/
int32_t SpeedLoopSmoothRun(int32_t VelocityRef, SpeedLoopSmooth* SpeedSmooth) {
  int32_t Temp = VelocityRef - SpeedSmooth->NowVelocityRef;

  if (ABS(Temp) <= SpeedSmooth->MaxVelAccEveryPrd)
    SpeedSmooth->NowVelocityRef = VelocityRef;

  if (VelocityRef > SpeedSmooth->NowVelocityRef) {
    SpeedSmooth->NowVelocityRef = SpeedSmooth->MaxVelAccEveryPrd + SpeedSmooth->OldVelocityRef;
  } else if (VelocityRef < SpeedSmooth->NowVelocityRef) {
    SpeedSmooth->NowVelocityRef = -SpeedSmooth->MaxVelAccEveryPrd + SpeedSmooth->OldVelocityRef;
  }

  SpeedSmooth->OldVelocityRef = SpeedSmooth->NowVelocityRef;
  return SpeedSmooth->NowVelocityRef;
}

/*******************************************************************************
  :speedOffsetMonitor
    :
    :
  :
    :
********************************************************************************/
void speedOffsetMonitor(ControllerStruct* controller) {
  static int16_t filter_count = 0;
  //
  int32_t speed_ref_abs = abs(controller->velocity_ref_filterd);
  // int32_t speed_actual_abs = ABS(controller->dtheta_mech);
  int32_t offset = abs(controller->velocity_ref_filterd - controller->dtheta_mech);
  if (speed_ref_abs > 103420)    // (1rpm)
  {
    //
    if (offset > speed_ref_abs * Threshold.velocity_coe / 10)    // 10
      filter_count++;
    else
      filter_count = 0;
  } else {
    //
    filter_count = 0;
  }
  if (filter_count > MOTOR_SPEED_OFFSET_COUNT)    //
  {
    controller->ServoErrFlag.Bit.speedOffsetErr = 1;
  }
}

/*******************************************************************************
 * 速度环带宽测试 - 正弦扫频 + 同步检测
 * 原理: 在 velocity_ref_filterd 上叠加单频正弦信号，通过同步检测提取响应幅值和相位，
 *       逐点扫频得到 Bode 图数据。
 * 使用: 串口发送 "logtest130" 启动，测试完成后自动打印结果。
 *******************************************************************************/

#define SPD_BW_FS        ((float)SPEED_LOOP_FRE)  // 速度环采样率 5000Hz
#define SPD_BW_SETTLE    5    // 每个频率点前等待稳态的周期数
#define SPD_BW_MEASURE   10   // 每个频率点测量的周期数
#define SPD_BW_OVP_MARGIN_DV 30  // 母线提前中止余量 (0.1V), 比硬件 OVP 阈值低 3V 即主动停机
#define SPD_BW_AMP_FLOOR_RATIO 0.10f  // 高频段最大衰减比, 防 SNR 崩 (10x 衰减地板)

/* 1/f 注入幅值: f<=f_break 段恒幅, f>f_break 段按 f_break/f 衰减(恒加速度). */
static int32_t spd_bw_amp_at(float freq, float amp_base, float f_break) {
  float scale = (freq > f_break) ? (f_break / freq) : 1.0f;
  if (scale < SPD_BW_AMP_FLOOR_RATIO) scale = SPD_BW_AMP_FLOOR_RATIO;
  return (int32_t)(amp_base * scale);
}

void spd_bw_test_init(SpeedLoopBWTest* test, float freq_start, float freq_end,
                      float amplitude_base, float f_break) {
  test->freq_start = freq_start;
  test->freq_end   = freq_end;
  test->amplitude_base = amplitude_base;  // 内部速度单位, 1rpm = 1024*25
  test->f_break    = (f_break > 0.0f) ? f_break : freq_start;
  test->points_per_decade = 10;

  // 计算总点数（+1 补偿截断，确保覆盖 freq_end）
  float decades = log10f(freq_end / freq_start);
  test->total_points = (uint16_t)(decades * test->points_per_decade) + 1;
  if (test->total_points > BW_TEST_MAX_POINTS)
    test->total_points = BW_TEST_MAX_POINTS;

  test->current_point = 0;
  test->current_freq  = freq_start;
  test->done          = 0;
  test->stopping      = 0;
  test->ramp_ref      = 0;

  // 计算第一个频率点的相位步进
  // phase_step = freq / fs * 2^32
  test->phase_step = (uint32_t)(test->current_freq / SPD_BW_FS * 4294967296.0f);
  test->phase_accum = 0;

  // 每个频率点的采样数 = (settle + measure) 周期 * 每周期采样数
  float samples_per_cycle = SPD_BW_FS / test->current_freq;
  test->samples_needed = (uint32_t)(samples_per_cycle * (SPD_BW_SETTLE + SPD_BW_MEASURE));
  test->settle_samples = (uint32_t)(samples_per_cycle * SPD_BW_SETTLE);

  test->sum_sin = 0;
  test->sum_cos = 0;
  test->ref_sin = 0;
  test->ref_cos = 0;
  test->sample_count = 0;

  test->current_amplitude = spd_bw_amp_at(test->current_freq,
                                          test->amplitude_base, test->f_break);

  test->abort_reason = 0;
  test->abort_udc    = 0;
  test->abort_freq   = 0;

  for (uint16_t i = 0; i < BW_TEST_MAX_POINTS; i++) test->udc_peak[i] = 0;

  test->enable = 1;
}

/*******************************************************************************
  spd_bw_test_run - 每个速度环周期调用一次 (5kHz)
  返回值: 叠加到 velocity_ref_filterd 上的注入信号（内部速度单位）
*******************************************************************************/
int32_t spd_bw_test_run(SpeedLoopBWTest* test, int32_t speed_feedback) {
  if (!test->enable) {
    // 斜坡停机阶段
    if (test->stopping) {
      // 每周期递减 1024，5kHz 下约 1rpm/s 降速
      if (test->ramp_ref > 0) {
        test->ramp_ref -= 1024;
        if (test->ramp_ref < 0) test->ramp_ref = 0;
      } else if (test->ramp_ref < 0) {
        test->ramp_ref += 1024;
        if (test->ramp_ref > 0) test->ramp_ref = 0;
      }
      controller_eyou.velocity_ref = test->ramp_ref;
      if (test->ramp_ref == 0) {
        test->stopping = 0;
      }
    }
    return 0;
  }

  // 母线过压看门狗: 距硬件 OVP 阈值还有 SPD_BW_OVP_MARGIN_DV 时主动中止 + 斜坡停机,
  // 抢在 dcVoltageProFunc 触发 OverBusVolErr 前优雅退出, 保留已采集到的 Bode 点
  if (Threshold.OverUdc > SPD_BW_OVP_MARGIN_DV &&
      motorProValue.Udc > (uint32_t)(Threshold.OverUdc - SPD_BW_OVP_MARGIN_DV)) {
    test->abort_reason = 1;
    test->abort_udc    = motorProValue.Udc;
    test->abort_freq   = test->current_freq;
    test->enable       = 0;
    test->done         = 1;
    test->stopping     = 1;
    test->ramp_ref     = controller_eyou.velocity_ref;
    return 0;
  }

  // 利用查表 sin/cos: phase_accum 高 16 位映射到 0~65535 (uq16)
  uint16_t angle = (uint16_t)(test->phase_accum >> 16);
  Trig_Components sc = get_sincos_value((int32_t)angle);
  // sc.hSin, sc.hCos 为 Q15 格式 (-32768 ~ 32767)

  // 注入信号: inject = current_amplitude * sin(angle)
  // 使用 int64_t 中间变量避免溢出（速度值范围大）
  int32_t inject = (int32_t)(((int64_t)test->current_amplitude * sc.hSin) >> 15);

  // 稳态等待阶段不累加，只在测量阶段累加
  if (test->sample_count >= test->settle_samples) {
    // 同步检测: 累加 response * sin/cos 和 reference * sin/cos
    test->sum_sin += (int64_t)speed_feedback * sc.hSin;
    test->sum_cos += (int64_t)speed_feedback * sc.hCos;
    test->ref_sin += (int64_t)inject * sc.hSin;
    test->ref_cos += (int64_t)inject * sc.hCos;
    // 测量阶段同步抓 Udc 峰值, 用于事后看母线随频率上行的曲线/共振点
    if (motorProValue.Udc > test->udc_peak[test->current_point]) {
      test->udc_peak[test->current_point] = motorProValue.Udc;
    }
  }

  // 相位累加
  test->phase_accum += test->phase_step;
  test->sample_count++;

  // 当前频率点采样完成
  if (test->sample_count >= test->samples_needed) {
    // 计算响应幅值和参考幅值
    double resp_amp = sqrt((double)test->sum_sin * test->sum_sin +
                           (double)test->sum_cos * test->sum_cos);
    double ref_amp  = sqrt((double)test->ref_sin * test->ref_sin +
                           (double)test->ref_cos * test->ref_cos);

    // 增益 = 响应幅值 / 参考幅值
    float gain = (ref_amp > 0) ? (float)(resp_amp / ref_amp) : 0.0f;
    test->gain_db[test->current_point] = 20.0f * log10f(gain + 1e-10f);

    // 相位 = atan2(resp) - atan2(ref)
    float resp_phase = atan2f((float)test->sum_sin, (float)test->sum_cos);
    float ref_phase  = atan2f((float)test->ref_sin, (float)test->ref_cos);
    float phase_diff = (resp_phase - ref_phase) * 180.0f / M_PIf;
    // 归一化到 -180 ~ 180
    while (phase_diff > 180.0f) phase_diff -= 360.0f;
    while (phase_diff < -180.0f) phase_diff += 360.0f;
    test->phase_deg[test->current_point] = phase_diff;

    test->freq_list[test->current_point] = test->current_freq;

    // 下一个频率点
    test->current_point++;
    if (test->current_point >= test->total_points) {
      test->enable = 0;
      test->done   = 1;
      // 启动斜坡停机，从当前 velocity_ref 缓慢降到 0
      test->stopping = 1;
      test->ramp_ref = controller_eyou.velocity_ref;
      return 0;
    }

    // 对数扫频
    test->current_freq = test->freq_start *
        powf(10.0f, (float)test->current_point / test->points_per_decade);
    // 限制不超过 Nyquist
    if (test->current_freq > SPD_BW_FS / 2.0f)
      test->current_freq = SPD_BW_FS / 2.0f;

    test->phase_step = (uint32_t)(test->current_freq / SPD_BW_FS * 4294967296.0f);
    test->phase_accum = 0;

    float spc = SPD_BW_FS / test->current_freq;
    test->samples_needed = (uint32_t)(spc * (SPD_BW_SETTLE + SPD_BW_MEASURE));
    test->settle_samples = (uint32_t)(spc * SPD_BW_SETTLE);

    // 切到下一个频点时按 1/f 规律更新注入幅值
    test->current_amplitude = spd_bw_amp_at(test->current_freq,
                                            test->amplitude_base, test->f_break);

    test->sum_sin = 0;
    test->sum_cos = 0;
    test->ref_sin = 0;
    test->ref_cos = 0;
    test->sample_count = 0;
  }

  return inject;
}

/*******************************************************************************
  spd_bw_test_print_results - 通过串口打印速度环 Bode 图数据
*******************************************************************************/
void spd_bw_test_print_results(SpeedLoopBWTest* test) {
  printf("\r\n===== Speed Loop Bandwidth Test Results =====\r\n");
  if (test->abort_reason == 1) {
    printf("[ABORTED] Udc near OVP at f=%.1f Hz: Udc=%u (0.1V), OverUdc=%u (0.1V)\r\n",
           test->abort_freq, (unsigned)test->abort_udc, (unsigned)Threshold.OverUdc);
    printf("Hint: lower freq_end, scale inject amplitude with 1/f, or add brake resistor.\r\n");
  }
  printf("Kp=%d Ki=%d Kd=%d PID_Div=%u vel_ref=%d inject=%.0f\r\n",
         (int)controller_eyou.IncPID_Speed.P,
         (int)controller_eyou.IncPID_Speed.I,
         (int)controller_eyou.IncPID_Speed.D,
         (unsigned)controller_eyou.IncPID_Speed.PID_Div,
         (int)controller_eyou.velocity_ref,
         test->amplitude_base);
  printf("Freq(Hz)\tGain(dB)\tPhase(deg)\tUdc_pk(0.1V)\r\n");
  for (uint16_t i = 0; i < test->current_point; i++) {
    printf("%.1f\t\t%.2f\t\t%.1f\t\t%u\r\n",
           test->freq_list[i],
           test->gain_db[i],
           test->phase_deg[i],
           (unsigned)test->udc_peak[i]);
  }

  // 找 -3dB 带宽: 要求连续两点都低于阈值才算穿越, 抑制高频段噪声单点假阳性
  float bw = 0;
  float gain_0 = test->gain_db[0];  // 低频基准增益
  float target = gain_0 - 3.0f;
  for (uint16_t i = 1; i + 1 < test->current_point; i++) {
    if (test->gain_db[i] < target && test->gain_db[i + 1] < target) {
      // 在 i-1 → i 段内插值出真实穿越点
      float f0 = test->freq_list[i - 1];
      float f1 = test->freq_list[i];
      float g0 = test->gain_db[i - 1];
      float g1 = test->gain_db[i];
      bw = f0 + (f1 - f0) * (target - g0) / (g1 - g0);
      break;
    }
  }
  if (bw > 0)
    printf("-3dB Bandwidth: %.1f Hz\r\n", bw);
  else
    printf("-3dB point not reached in test range\r\n");

  // 还原测试期间临时放宽的速度偏差阈值
  if (test->saved_velocity_coe != 0) {
    Threshold.velocity_coe = test->saved_velocity_coe;
    test->saved_velocity_coe = 0;
    printf("velocity_coe restored to %u\r\n", (unsigned)Threshold.velocity_coe);
  }

  printf("=============================================\r\n");
}
