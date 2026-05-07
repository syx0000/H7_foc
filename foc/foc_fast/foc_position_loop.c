/**
 * @file    foc_controller.c
 * @brief
 * @author  yxsui
 * @date    2025-07-31
 * @version 1.0
 */

#include "foc_position_loop.h"
#include "ifly_fault_api.h"
#include "math.h"
#include "stdlib.h"

/* 默认梯形规划: 1.5 output rpm 巡航 / 30 output rpm/s 加速 (50ms 加速时间).
 * 这两个值适配当前测试架 (母线电容 ~100µF, 无刹车电阻); 接到正式驱动器后可放开. */
#define POS_TRAPEZOID_DEFAULT_VMAX_RPM   33.0f
#define POS_TRAPEZOID_DEFAULT_AMAX_RPS   80.0f
/* output rpm → LSB/tick 系数 (位置环 Ts = 1/2500 s) */
#define POS_TRAPEZOID_VMAX_SCALE         (6.0f * 1024.0f / 2500.0f)              /* ≈ 2.4576 */
#define POS_TRAPEZOID_AMAX_SCALE         (6.0f * 1024.0f / (2500.0f * 2500.0f))  /* ≈ 9.830e-4 */
// extern ControllerStruct controller_eyou;
extern Portection_Value Threshold;
/*******************************************************************************
  : foc_position_close_loop_pid
    :
    :
  :
    :
********************************************************************************/
void foc_position_close_loop(ControllerStruct* controller) {
  /* PP 模式: 走梯形规划 (V/A 双限, 防 OVP / 速度环饱和).
   * CSP 模式: 透传, master 已自带轨迹规划, 本地再规划会引入相位延迟与抖动.
   * 其它模式: 不进入下面的 PID 分支, ref_filterd 保持透传值即可. */
  if (controller->controller_mode == PROFILE_POSITION_MODE) {
    controller->position_ref_filterd = PosSmoothRun(&controller->SmoothPosRef, controller);
  } else {
    controller->position_ref_filterd = controller->position_ref;
    /* 退出 PP 模式: 清规划状态, 下次进入 PP 时强制从 real_position_out 重锚定 */
    controller->SmoothPosRef.active = 0;
  }

  // 位置环带宽测试信号注入（梯形规划之后、PID 之前）
  int32_t pos_sweep = pos_bw_test_run(&controller->pos_bw_test, controller->real_position_out);
  controller->position_ref_filterd += pos_sweep;

  // PID,
  if (controller->controller_mode == PROFILE_POSITION_MODE ||
      controller->controller_mode == CYCLIC_SYNC_POSITION_MODE) {
    controller->IncPID_Position.NowValue = controller->real_position_out;
    controller->IncPID_Position.AimValue = controller->position_ref_filterd;
    controller->IncPID_Position.PidRun(&controller->IncPID_Position);
    controller->velocity_ref = controller->IncPID_Position.OutPut;

    /* add proportional feedforward from position error */
    {
      int32_t pos_err = controller->position_ref_filterd - controller->real_position_out;
      /* gain stored Q10: out unit same as velocity_ref (rpm/1024) */
      int32_t ff = (controller->pos_err_ff_gain * pos_err);
      controller->velocity_ref += ff;
    }
  } else {
    controller->position_ref = controller->real_position_out;
    controller->IncPID_Position.PidInit(&controller->IncPID_Position,
                                        controller->FlashData.Position_Kp,
                                        controller->FlashData.Position_Ki,
                                        controller->FlashData.Position_Kd,
                                        DEFAULT_PID_POSITION_DIV,
                                        controller->FlashData.Pid_PositionLimit);
  }
}

/*******************************************************************************
  :InitSpeedShowFilter
    :
    :
  :
    :
********************************************************************************/
uint8_t InitPositionShowFilter(ControllerStruct* controller) {
  // 1msus-----------IS620 1ms
  controller->PositionShowFilter.Ts = 1000L;
  // 50msus---------IS620 50ms
  controller->PositionShowFilter.Tc = 50000L;
  // ka,kb
  controller->PositionShowFilter.Filter1_Init(&controller->PositionShowFilter);
  // 1msus-----------IS620 1ms
  controller->PositionRefFilter.Ts = 1000L;
  // 50msus---------IS620 200ms
  controller->PositionRefFilter.Tc = 200000L;
  // ka,kb
  controller->PositionRefFilter.Filter1_Init(&controller->PositionRefFilter);
  return 0;
}

/*******************************************************************************
  :PositionShowFilterGoing
    :
    :
  :
    :
********************************************************************************/
int32_t PositionShowFilterGoing(ControllerStruct* controller) {
  controller->PositionShowFilter.InPut = controller->real_position;
  controller->PositionShowFilter.Filter1(&controller->PositionShowFilter);
  return (int32_t)controller->PositionShowFilter.OutPut;    //
}

/*******************************************************************************
  : SpeedRefFilterGoing
    :
    :
  :
    :
********************************************************************************/
int32_t PositionRefFilterGoing(ControllerStruct* controller) {
  controller->PositionRefFilter.InPut = controller->position_ref;
  controller->PositionRefFilter.Filter1(&controller->PositionRefFilter);
  return (int32_t)controller->PositionRefFilter.OutPut;    //
}

/*******************************************************************************
  : InitPosSmoothFunc
    : 设置梯形规划默认 V_max / A_max; 状态清零, 待首次 ref 变化时从 real_position
      重新锚定 (bumpless).
********************************************************************************/
void InitPosSmoothFunc(PositionRefSmooth* p) {
  p->v_max       = POS_TRAPEZOID_DEFAULT_VMAX_RPM * POS_TRAPEZOID_VMAX_SCALE;
  p->a_max       = POS_TRAPEZOID_DEFAULT_AMAX_RPS * POS_TRAPEZOID_AMAX_SCALE;
  p->cur_pos     = 0.0f;
  p->cur_v       = 0.0f;
  p->old_pos_ref = 0;
  p->active      = 0;
}

/*******************************************************************************
  : PosSmoothRun
    : 梯形位置规划 (V_max + A_max 双限, 相平面 bang-bang 时间最优).
      ref 变化时: 静止则从 real_position_out 起步 (bumpless), 运动中则继承
      cur_pos / cur_v 由算法自动重规划.
    : 返回当前 tick 的规划位置 (1/1024 °_out).
********************************************************************************/
ATTR_RAMFUNC
int32_t PosSmoothRun(PositionRefSmooth* p, ControllerStruct* controller) {
  /* 静止状态: 始终从 real_position_out 重锚定 cur_pos, 防上次状态污染.
   * 启动条件 = "目标位置 ≠ 实际位置" (非"ref 变化"), 这样冷启动时
   * old_pos_ref=0 与用户 tar0 重合也能正确启动. */
  if (!p->active) {
    p->cur_pos     = (float)controller->real_position_out;
    p->cur_v       = 0.0f;
    p->old_pos_ref = controller->position_ref;
    if (controller->position_ref == controller->real_position_out) {
      return controller->position_ref;    /* 已对齐, 透传 */
    }
    p->active = 1;
  } else if (controller->position_ref != p->old_pos_ref) {
    /* 运动中目标变化: 不动 cur_pos / cur_v, 让 bang-bang 自动重规划 */
    p->old_pos_ref = controller->position_ref;
  }

  /* 相平面: 当前 (cur_pos, cur_v) → 目标 (ref, 0).
   * brake_dist 是从 |cur_v| 全力刹到 0 的位移; 比较 |D| 与 brake_dist 决定
   * 加速 / 巡航 / 减速 / 反向加速. */
  float D          = (float)controller->position_ref - p->cur_pos;
  float v          = p->cur_v;
  float brake_dist = (v * v) / (2.0f * p->a_max);

  float v_new;
  if (D > 0.0f) {
    if (v < 0.0f)              v_new = v + p->a_max;     /* 反向先减速 */
    else if (D > brake_dist)   v_new = v + p->a_max;     /* 加速 / 巡航(被 v_max 限) */
    else                       v_new = v - p->a_max;     /* 减速准备到位 */
  } else if (D < 0.0f) {
    if (v > 0.0f)              v_new = v - p->a_max;
    else if (-D > brake_dist)  v_new = v - p->a_max;
    else                       v_new = v + p->a_max;
  } else {
    if (v > 0.0f)       v_new = v - p->a_max;
    else if (v < 0.0f)  v_new = v + p->a_max;
    else                v_new = 0.0f;
  }

  /* V_max 限幅 */
  if (v_new >  p->v_max)  v_new =  p->v_max;
  if (v_new < -p->v_max)  v_new = -p->v_max;

  p->cur_v   = v_new;
  p->cur_pos += v_new;

  /* 到位 snap: ε 取一拍 a_max 速度 + 1 LSB 距离, 防止终态极限环抖动 */
  float remaining = (float)controller->position_ref - p->cur_pos;
  if (fabsf(remaining) < 1.0f && fabsf(v_new) < p->a_max) {
    p->cur_pos = (float)controller->position_ref;
    p->cur_v   = 0.0f;
    p->active  = 0;
  }

  return (int32_t)p->cur_pos;
}

/*******************************************************************************
 * 位置环带宽测试 - 正弦扫频 + 同步检测
 * 原理: 在 position_ref_filterd 上叠加单频正弦信号，通过同步检测提取响应幅值和相位，
 *       逐点扫频得到 Bode 图数据。
 * 使用: 串口发送 "logtest160" 启动，测试完成后自动打印结果。
 *******************************************************************************/

#define POS_BW_FS        ((float)(FOC_FREQUENCY / POSITION_CALCULATE_DIV))  // 位置环采样率 2500Hz
#define POS_BW_SETTLE    5    // 每个频率点前等待稳态的周期数
#define POS_BW_MEASURE   10   // 每个频率点测量的周期数
#define POS_BW_AMP_FLOOR_RATIO 0.10f  // 高频段最大衰减比, 防 SNR 崩 (10x 衰减地板)
#define POS_BW_OVP_MARGIN_DV 30  // 母线提前中止余量 (0.1V), 比硬件 OVP 阈值低 3V 即主动停机

extern ifly_Err_Pro_Type motorProValue;

/* 1/f 注入幅值: f<=f_break 段恒幅, f>f_break 段按 f_break/f 衰减(恒速度). */
static int32_t pos_bw_amp_at(float freq, float amp_base, float f_break) {
  float scale = (freq > f_break) ? (f_break / freq) : 1.0f;
  if (scale < POS_BW_AMP_FLOOR_RATIO) scale = POS_BW_AMP_FLOOR_RATIO;
  return (int32_t)(amp_base * scale);
}

void pos_bw_test_init(PositionLoopBWTest* test, float freq_start, float freq_end,
                      float amplitude_base, float f_break) {
  test->freq_start = freq_start;
  test->freq_end   = freq_end;
  test->amplitude_base = amplitude_base;  // 位置单位, 1° = 1024
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

  // 计算第一个频率点的相位步进
  test->phase_step = (uint32_t)(test->current_freq / POS_BW_FS * 4294967296.0f);
  test->phase_accum = 0;

  // 每个频率点的采样数
  float samples_per_cycle = POS_BW_FS / test->current_freq;
  test->samples_needed = (uint32_t)(samples_per_cycle * (POS_BW_SETTLE + POS_BW_MEASURE));
  test->settle_samples = (uint32_t)(samples_per_cycle * POS_BW_SETTLE);

  test->sum_sin = 0;
  test->sum_cos = 0;
  test->ref_sin = 0;
  test->ref_cos = 0;
  test->sample_count = 0;

  test->current_amplitude = pos_bw_amp_at(test->current_freq,
                                          test->amplitude_base, test->f_break);

  test->abort_reason = 0;
  test->abort_freq   = 0;

  test->enable = 1;
}

/*******************************************************************************
  pos_bw_test_run - 每个位置环周期调用一次 (2.5kHz)
  返回值: 叠加到 position_ref_filterd 上的注入信号（位置单位 1°/1024）
*******************************************************************************/
int32_t pos_bw_test_run(PositionLoopBWTest* test, int32_t position_feedback) {
  if (!test->enable) {
    if (test->stopping) {
      test->stopping = 0;
    }
    return 0;
  }

  // 母线过压看门狗: 距硬件 OVP 阈值还有 POS_BW_OVP_MARGIN_DV 时主动中止
  if (Threshold.OverUdc > POS_BW_OVP_MARGIN_DV &&
      motorProValue.Udc > (uint32_t)(Threshold.OverUdc - POS_BW_OVP_MARGIN_DV)) {
    test->abort_reason = 1;
    test->abort_freq   = test->current_freq;
    test->enable       = 0;
    test->done         = 1;
    test->stopping     = 1;
    return 0;
  }

  // 利用查表 sin/cos: phase_accum 高 16 位映射到 0~65535 (uq16)
  uint16_t angle = (uint16_t)(test->phase_accum >> 16);
  Trig_Components sc = get_sincos_value((int32_t)angle);

  // 注入信号: inject = current_amplitude * sin(angle)
  int32_t inject = (int32_t)(((int64_t)test->current_amplitude * sc.hSin) >> 15);

  // 稳态等待阶段不累加，只在测量阶段累加
  if (test->sample_count >= test->settle_samples) {
    test->sum_sin += (int64_t)position_feedback * sc.hSin;
    test->sum_cos += (int64_t)position_feedback * sc.hCos;
    test->ref_sin += (int64_t)inject * sc.hSin;
    test->ref_cos += (int64_t)inject * sc.hCos;
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
      test->stopping = 1;
      return 0;
    }

    // 对数扫频
    test->current_freq = test->freq_start *
        powf(10.0f, (float)test->current_point / test->points_per_decade);
    // 限制不超过 Nyquist
    if (test->current_freq > POS_BW_FS / 2.0f)
      test->current_freq = POS_BW_FS / 2.0f;

    test->phase_step = (uint32_t)(test->current_freq / POS_BW_FS * 4294967296.0f);
    test->phase_accum = 0;

    float spc = POS_BW_FS / test->current_freq;
    test->samples_needed = (uint32_t)(spc * (POS_BW_SETTLE + POS_BW_MEASURE));
    test->settle_samples = (uint32_t)(spc * POS_BW_SETTLE);

    // 切到下一个频点时按 1/f 规律更新注入幅值
    test->current_amplitude = pos_bw_amp_at(test->current_freq,
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
  pos_bw_test_print_results - 通过串口打印位置环 Bode 图数据
*******************************************************************************/
void pos_bw_test_print_results(PositionLoopBWTest* test) {
  printf("\r\n===== Position Loop Bandwidth Test Results =====\r\n");
  if (test->abort_reason == 1) {
    printf("[ABORTED] at f=%.1f Hz\r\n", test->abort_freq);
  }
  printf("Freq(Hz)\tGain(dB)\tPhase(deg)\r\n");
  for (uint16_t i = 0; i < test->current_point; i++) {
    printf("%.1f\t\t%.2f\t\t%.1f\r\n",
           test->freq_list[i],
           test->gain_db[i],
           test->phase_deg[i]);
  }

  // 找 -3dB 带宽
  float bw = 0;
  float gain_0 = test->gain_db[0];
  float target = gain_0 - 3.0f;
  for (uint16_t i = 1; i + 1 < test->current_point; i++) {
    if (test->gain_db[i] < target && test->gain_db[i + 1] < target) {
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

  printf("================================================\r\n");
}