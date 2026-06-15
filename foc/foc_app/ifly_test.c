/**
 * @file    ifly_test.c
 * @brief   故障测试代码
 * @author  xlding15
 * @date    2025-11-19
 * @version 1.0
 */

#include "ifly_test.h"
#include "foc_api.h"
#include "foc_bsp.h"
#include "foc_controller.h"
#include "foc_current_loop.h"
#include "foc_data.h"
#include "foc_speed_loop.h"
#include "ifly_fault.h"
#include "ifly_fault_api.h"
#include "ifly_flux_ident.h"
#include "ifly_inertia_ident.h"
#include "ifly_led.h"
#include "foc_position_loop.h"
// #include "FreeRTOS.h"  /* FreeRTOS removed */
// #include "task.h"  /* FreeRTOS removed */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern volatile uint16_t testLogFlag;
extern ControllerStruct controller_eyou;
extern Portection_Value Threshold;
extern uint8_t test_control;
extern LED_STATUSBits LED_STATUS;
extern ifly_Err_Pro_Type motorProValue;

#define TEST_MOTOR_CURRENT_MODE PROFILE_TORQUE_MODE
#define TEST_MOTOR_SPEED_MODE PROFILE_VELOCITY_MOCE
#define TEST_MOTOR_POSIT_MODE PROFILE_POSITION_MODE

void Test_log_print(void) {
  if (controller_eyou.bw_test.done) {
    bw_test_print_results(&controller_eyou.bw_test);
    controller_eyou.bw_test.done = 0;
    /* 测试结束后走安全停机, 防止 PWM 卡死在最后一拍 CCR 状态 */
    fault_safe_shutdown();
  }
  if (controller_eyou.spd_bw_test.done) {
    spd_bw_test_print_results(&controller_eyou.spd_bw_test);
    controller_eyou.spd_bw_test.done = 0;
    fault_safe_shutdown();
  }
  if (controller_eyou.pos_bw_test.done) {
    pos_bw_test_print_results(&controller_eyou.pos_bw_test);
    controller_eyou.pos_bw_test.done = 0;
    fault_safe_shutdown();
  }
}

void TestBusOverUdc(void) {
}

void TestBusLowUdc(void) {
}

void TestBoardOverTem(void) {
}

void TestLockedRotorCurrent(void) {
}

void TestLockedRotorSpeed(void) {
}

void TestLockedRotorPosit(void) {
}

void TestSpeedOffset(void) {
}

void TestMotorVelOver(void) {
}

void TestPostionOver(void) {
}

void TestIbusCurrentOver(void) {
}

void TestUVWCurrentOver(void) {
}

void TestCurrentLoopBandwidth(void) {
  /* 保守版: 10-2500Hz 扫频, 注入 0.3A (Q10=307), 工作点偏置 0.5A (Q10=512) */
  controller_eyou.controller_mode = TEST_MOTOR_CURRENT_MODE;
  controller_eyou.I_q_ref         = 666;
  controller_eyou.foc_run         = 1;

  bw_test_init(&controller_eyou.bw_test, 10.0f, 2500.0f, 307.0f);

  printf("BW test started: 10-2500Hz, 0.3A inject, bias 0.5A\r\n");
}

void TestSpeedLoopBandwidth(void) {
  controller_eyou.controller_mode = TEST_MOTOR_SPEED_MODE;
  controller_eyou.velocity_ref    = 10 * 1024 * 25;  // 10rpm 偏置
  controller_eyou.foc_run         = 1;

  spd_bw_test_init(&controller_eyou.spd_bw_test, 1.0f, 200.0f,
                   2.0f * 1024 * 25, 5.0f);  // 注入 2rpm, 5Hz 以上按 1/f 衰减

  controller_eyou.spd_bw_test.saved_velocity_coe = Threshold.velocity_coe;
  Threshold.velocity_coe = 100;  // 临时放宽速度偏差保护

  controller_eyou.spd_bw_test.saved_phase_loss_iq = Threshold.PhaseLossActiveIq;
  Threshold.PhaseLossActiveIq = 65535;  // 临时关闭缺相保护 (启用阈值拉到最大)

  printf("Speed BW test started: 1-200Hz, 2rpm inject, bias 10rpm\r\n");
}

void TestFluxIdent(void) {
  if (controller_eyou.FlashData.MotorParamFlag != OFFEST_IS_CORRECTED_FLAG) {
    printf("Flux ident FAILED: motor params (Rs/Ld/Lq) not identified, run bwtest3 first\r\n");
    return;
  }
  FluxIdentCfg cfg;
  FluxIdentResult result;
  flux_ident_default_cfg(&cfg);
  cfg.rs_known = controller_eyou.ident_test.Rs;

  printf("Flux ident started...\r\n");
  FluxIdentErr err = runFluxIdent(&controller_eyou, &cfg, &result);
  if (err != FLUX_IDENT_OK) {
    printf("Flux ident FAILED: err=%d\r\n", (int)err);
    return;
  }
  printf("Flux ident OK: psi_f=%.6f Wb  we=%.2f rad/s\r\n", result.flux, result.we);
  printf("  uq1=%.3f iq1=%.3f id1=%.3f\r\n", result.uq1, result.iq1, result.id1);
  printf("  uq2=%.3f iq2=%.3f id2=%.3f\r\n", result.uq2, result.iq2, result.id2);

  /* 存入 Flash 预留字段 temp7 (float 按位复用 int32_t) */
  union { float f; int32_t i; } u;
  u.f = result.flux;
  controller_eyou.FlashData.temp7 = u.i;
  controller_eyou.FlashData.FluxIdentFlag = OFFEST_IS_CORRECTED_FLAG;
  controller_eyou.ident_test.flux_psi = result.flux;
  WriteRunDataToFlash(&controller_eyou, MOTORID0_RUN_DATA_ADDRESS);
  printf("psi_f saved to Flash\r\n");
}

void TestInertiaIdent(void) {
  if (controller_eyou.FlashData.MotorParamFlag != OFFEST_IS_CORRECTED_FLAG) {
    printf("Inertia ident FAILED: motor params not identified, run bwtest3 first\r\n");
    return;
  }
  if (controller_eyou.FlashData.FluxIdentFlag != OFFEST_IS_CORRECTED_FLAG) {
    printf("Inertia ident FAILED: flux not identified, run bwtest4 first\r\n");
    return;
  }
  InertiaIdentCfg cfg;
  InertiaIdentResult result;
  inertia_ident_default_cfg(&cfg);

  /* 从 Flash 读取已辨识的参数 (Flag 已验证有效) */
  union { float f; int32_t i; } u;
  u.i = controller_eyou.FlashData.temp2;  cfg.Ld    = u.f;
  u.i = controller_eyou.FlashData.temp3;  cfg.Lq    = u.f;
  u.i = controller_eyou.FlashData.temp7;  cfg.psi_f = u.f;

  printf("Inertia ident started (psi_f=%.6f Ld=%.3e Lq=%.3e)...\r\n",
         cfg.psi_f, cfg.Ld, cfg.Lq);
  InertiaIdentErr err = runInertiaIdent(&controller_eyou, &cfg, &result);
  if (err != INERTIA_IDENT_OK) {
    printf("Inertia ident FAILED: err=%d\r\n", (int)err);
    return;
  }
  printf("Inertia ident OK: J=%.6e kg·m²  cycles=%u\r\n",
         result.J, (unsigned)result.cycles_used);
  printf("  a+/- = %.1f / %.1f rad/s²\r\n", result.a_pos_avg, result.a_neg_avg);
  printf("  Te+/- = %.4f / %.4f N·m\r\n", result.Te_pos_avg, result.Te_neg_avg);

  /* 存入 Flash 预留字段 temp8 */
  u.f = result.J;
  controller_eyou.FlashData.temp8 = u.i;
  controller_eyou.FlashData.InertiaIdentFlag = OFFEST_IS_CORRECTED_FLAG;
  WriteRunDataToFlash(&controller_eyou, MOTORID0_RUN_DATA_ADDRESS);
  printf("J saved to Flash\r\n");
}

void TestPositionLoopBandwidth(void) {
  /* 用 CSP 模式而非 PP 模式: PP 模式有梯形规划吃掉注入信号 */
  controller_eyou.controller_mode = CYCLIC_SYNC_POSITION_MODE;
  controller_eyou.position_ref    = controller_eyou.real_position_out;  // 当前位置作为静态参考
  controller_eyou.foc_run         = 1;

  /* 4~100Hz 扫频, 注入 2° (= 2*1024 位置单位), 4Hz 以上按 1/f 衰减
   * 跳过 1~3Hz 死区段 (减速箱反向间隙 + 静摩擦), 该段电机不动数据无意义 */
  pos_bw_test_init(&controller_eyou.pos_bw_test, 4.0f, 100.0f,
                   2.0f * 1024, 4.0f);

  controller_eyou.pos_bw_test.saved_velocity_coe = Threshold.velocity_coe;
  Threshold.velocity_coe = 100;  // 临时放宽速度偏差保护

  printf("Position BW test started: 4-100Hz, 2deg inject, static ref (CSP mode)\r\n");
}

void TestMotorParamsIdent(void) {
  /* 强制重新辨识 Rs/Ld/Lq: 清 Flag → identifyMotorParamsCached 走辨识分支 → 写 Flash */
  controller_eyou.FlashData.MotorParamFlag = 0xFFFF;
  identifyMotorParamsCached(&controller_eyou);
}

void TestAutoTuneSpeed(void) {
  if (controller_eyou.FlashData.FluxIdentFlag != OFFEST_IS_CORRECTED_FLAG) {
    printf("AutoTune Speed FAILED: flux not identified, run bwtest4 first\r\n");
    return;
  }
  if (controller_eyou.FlashData.InertiaIdentFlag != OFFEST_IS_CORRECTED_FLAG) {
    printf("AutoTune Speed FAILED: inertia not identified, run bwtest5 first\r\n");
    return;
  }
  extern uint8_t NPP;
  union { float f; int32_t i; } u;
  u.i = controller_eyou.FlashData.temp7;  float psi_f = u.f;
  u.i = controller_eyou.FlashData.temp8;  float J     = u.f;
  autoTuneSpeedLoopPI(J, psi_f, NPP);
}

void TestAutoTuneCurrent(void) {
  if (controller_eyou.FlashData.MotorParamFlag != OFFEST_IS_CORRECTED_FLAG) {
    printf("AutoTune Current FAILED: motor params not identified, run bwtest3 first\r\n");
    return;
  }
  union { float f; int32_t i; } u;
  u.i = controller_eyou.FlashData.temp1;  float Rs = u.f;
  u.i = controller_eyou.FlashData.temp2;  float Ld = u.f;
  u.i = controller_eyou.FlashData.temp3;  float Lq = u.f;
  autoTuneCurrentLoopPI(Rs, Ld, Lq);
}

void TestAutoTunePosition(void) {
  /* 位置环 autoTune 不依赖电机参数, 直接按经验公式计算 */
  autoTunePositionLoopPI();
}

/*******************************************************************************
 * 死区补偿查表自动标定 (bwtest10)
 *
 * 原理: 锁定转子, theta_elec=0 钉死, 注入恒定 V_d (沿 A 相)
 *       闭环逼近一组目标电流, 反推 V_dt = V_d - Rs × |I_a|
 *       输出 LUT 数据到串口, 用户复制到 foc_current_loop.c 的 s_dt_lut
 *
 * 前置: 1) 转子机械锁住 (手柄/夹具) — 否则会转, 数据无效
 *       2) bwtest3 跑过, Rs 在 Flash temp1 (MotorParamFlag 有效)
 *       3) 上电后通信确认电源 48V 接好
 *
 * 安全: V_d 上限 8V (对应 Rs=0.08Ω 时电流约 100A, 实际靠 i_target 控制)
 *       任意点 V_d 加到 8V 仍不收敛 → 终止 (转子未锁 / Rs 错)
 *       每点超时 10s → 终止
 ******************************************************************************/
void TestDeadtimeCalibration(void) {
  if (controller_eyou.FlashData.MotorParamFlag != OFFEST_IS_CORRECTED_FLAG) {
    printf("Dt cal FAILED: motor params not identified, run bwtest3 first\r\n");
    return;
  }
  union { float f; int32_t i; } u;
  u.i = controller_eyou.FlashData.temp1;
  float Rs = u.f;
  if (Rs <= 0.0f || Rs > 1.0f) {
    printf("Dt cal FAILED: invalid Rs=%.4f Ohm\r\n", Rs);
    return;
  }

  printf("\r\n===== Deadtime Calibration =====\r\n");
  printf("WARNING: rotor MUST be locked (held mechanically). Starting in 3s...\r\n");
  HAL_Delay(3000);
  printf("Rs (Flash) = %.4f Ohm\r\n", Rs);

  /* 目标电流序列 (Q10), 从小到大 */
  const int32_t targets_q10[] = { 512, 1024, 2048, 5120, 10240, 20480, 30720 };
  const int N = sizeof(targets_q10) / sizeof(targets_q10[0]);
  int32_t v_dt_results[7];
  int32_t i_a_results[7];
  int success_count = 0;
  int32_t v_d_q10 = 0;

  uint8_t old_foc_run = controller_eyou.foc_run;

  /* 进辨识通道 (旁路 PI + 编码器不覆盖 theta_elec, 同 measurePhaseResistance) */
  controller_eyou.ident_test.enable = 1;
  controller_eyou.ident_test.done = 0;
  controller_eyou.ident_test.amplitude = 0;
  controller_eyou.ident_test.settle_samples = 0xFFFFFFFF;
  controller_eyou.ident_test.measure_samples = 0;
  controller_eyou.ident_test.sample_count = 0;
  controller_eyou.theta_elec = 0;
  controller_eyou.V_d = 0;
  controller_eyou.V_q = 0;
  controller_eyou.foc_run = 1;

  printf("Idx  I_target(A)  V_d(V)   I_a_avg(A)  V_dt(V)  Iq10  Vq10\r\n");

  for (int idx = 0; idx < N; idx++) {
    int32_t i_target = targets_q10[idx];
    int32_t tol = i_target / 20;        /* 5% 收敛容差 */
    if (tol < 50) tol = 50;             /* 过零区最小容差 0.05A */

    /* 闭环逼近: 每 5ms 加 0.05V, 直到 |I_a - target| < tol */
    uint32_t timeout_ms = 0;
    while (1) {
      int32_t i_a_now = controller_eyou.I_a;
      int32_t err = i_target - i_a_now;
      if (err < 0) err = -err;
      if (err < tol) break;

      v_d_q10 += 50;
      if (v_d_q10 > 8 * 1024) {
        printf("FAIL idx=%d: V_d > 8V no convergence (rotor not locked? Rs wrong?)\r\n", idx);
        goto cleanup;
      }
      controller_eyou.V_d = v_d_q10;
      HAL_Delay(5);
      timeout_ms += 5;
      if (timeout_ms > 10000) {
        printf("FAIL idx=%d: timeout 10s, Vd=%.3fV Ia=%.3fA tar=%.3fA\r\n",
               idx, v_d_q10 / 1024.0f, i_a_now / 1024.0f, i_target / 1024.0f);
        goto cleanup;
      }
    }

    /* 稳态等待 200ms + 采样 100ms 平均 */
    HAL_Delay(200);
    int64_t sum_ia = 0;
    const int n_samp = 100;
    for (int k = 0; k < n_samp; k++) {
      HAL_Delay(1);
      sum_ia += controller_eyou.I_a;
    }
    int32_t i_a_avg_q10 = (int32_t)(sum_ia / n_samp);
    float i_a_avg = i_a_avg_q10 / 1024.0f;
    float v_d = v_d_q10 / 1024.0f;
    float i_abs = (i_a_avg < 0) ? -i_a_avg : i_a_avg;
    float v_dt = v_d - Rs * i_abs;
    int32_t v_dt_q10 = (int32_t)(v_dt * 1024.0f);
    if (v_dt_q10 < 0) v_dt_q10 = 0;     /* 防 Rs 偏差导致负值 */

    i_a_results[idx]  = i_a_avg_q10 < 0 ? -i_a_avg_q10 : i_a_avg_q10;
    v_dt_results[idx] = v_dt_q10;
    success_count++;

    printf("%-3d  %7.2f     %6.3f   %7.3f     %5.3f    %4ld  %4ld\r\n",
           idx, i_target / 1024.0f, v_d, i_a_avg, v_dt,
           (long)i_a_results[idx], (long)v_dt_q10);
  }

cleanup:
  controller_eyou.ident_test.enable = 0;
  controller_eyou.V_d = 0;
  controller_eyou.V_q = 0;
  set_phase_voltage(&controller_eyou, 0, 0, 0);
  controller_eyou.foc_run = old_foc_run;

  /* 打印 LUT 复制粘贴格式 */
  printf("\r\n----- s_dt_lut copy-paste data -----\r\n");
  printf("static const dt_pt_t s_dt_lut[] = {\r\n");
  printf("    {     0,    0 },   /* zero crossing */\r\n");
  for (int i = 0; i < success_count; i++) {
    printf("    { %5ld, %4ld },   /* %.2fA -> %.3fV */\r\n",
           (long)i_a_results[i], (long)v_dt_results[i],
           i_a_results[i] / 1024.0f, v_dt_results[i] / 1024.0f);
  }
  printf("};\r\n");
  printf("================================================\r\n");
}

/*******************************************************************************
 * bwtest11: 相位补偿自动辨识
 * 原理: BEMF FF 开启时, 稳态 V_d_pi = ωe × ψf × ε (角度误差)
 * 两速度点测量 → 线性回归得 offset 和 comp (正/反方向各一次)
 ******************************************************************************/
void TestAutoPhaseComp(void)
{
  /* 前置检查: 需要 ψf 和 BEMF FF */
  if (controller_eyou.FlashData.FluxIdentFlag != OFFEST_IS_CORRECTED_FLAG) {
    printf("PhaseComp ident FAILED: psi_f not identified, run bwtest4 first\r\n");
    return;
  }
  float psi_f = controller_eyou.ident_test.flux_psi;
  if (psi_f < 1e-6f) {
    printf("PhaseComp ident FAILED: psi_f=0\r\n");
    return;
  }

  printf("=== Phase Comp Auto-Ident (bwtest11) ===\r\n");

  /* 保存当前参数 */
  extern int16_t g_theta_offset_pos, g_theta_offset_neg;
  extern int16_t g_theta_comp_pos, g_theta_comp_neg;
  int16_t saved_off_pos  = g_theta_offset_pos;
  int16_t saved_off_neg  = g_theta_offset_neg;
  int16_t saved_comp_pos = g_theta_comp_pos;
  int16_t saved_comp_neg = g_theta_comp_neg;
  uint8_t saved_mode = controller_eyou.controller_mode;
  uint8_t saved_run  = controller_eyou.foc_run;
  uint8_t saved_velocity_coe = Threshold.velocity_coe;
  uint16_t saved_phase_loss_iq = Threshold.PhaseLossActiveIq;

  /* 临时放宽保护 (速度阶跃瞬态不触发偏差/缺相保护) */
  Threshold.velocity_coe = 100;
  Threshold.PhaseLossActiveIq = 65535;

  /* 清零相位补偿 (让误差全部暴露到 V_d_pi) */
  g_theta_offset_pos = 0; g_theta_offset_neg = 0;
  g_theta_comp_pos = 0;   g_theta_comp_neg = 0;

  /* 切速度模式 */
  controller_eyou.controller_mode = CYCLIC_SYNC_VELOCITY_MODE;
  controller_eyou.foc_run = 1;

  /* 测量参数 */
  const int32_t speed_lo = 750;   /* rpm 电机端 */
  const int32_t speed_hi = 2000;  /* rpm 电机端 */
  const uint32_t settle_ms = 1500;
  const uint32_t sample_ms = 1000;

  float we_lo = speed_lo * 2.0f * 3.14159265f / 60.0f * NPP;
  float we_hi = speed_hi * 2.0f * 3.14159265f / 60.0f * NPP;
  /* dtheta_per_sample (theta_elec counts / sample @10kHz) */
  float dt_lo = we_lo / 10000.0f * 65536.0f / (2.0f * 3.14159265f);
  float dt_hi = we_hi / 10000.0f * 65536.0f / (2.0f * 3.14159265f);

  int16_t new_off_pos = 0, new_off_neg = 0;
  int16_t new_comp_pos = 0, new_comp_neg = 0;
  uint8_t success = 0;

  /* velocity_ref 单位 = rpm × 1024 电机端 */
  int32_t vel_lo_pos = speed_lo * 1024;
  int32_t vel_hi_pos = speed_hi * 1024;

  /* ===== 正转 ===== */
  printf("Pos direction: ramping to %ld rpm...\r\n", (long)speed_lo);
  controller_eyou.velocity_ref = vel_lo_pos;
  for (uint32_t i = 0; i < settle_ms; i++) {
    if (controller_eyou.ServoErrFlag.All_Flag) goto fault;
    HAL_Delay(1);
  }

  int64_t sum_vd;
  float vd_avg_lo, vd_avg_hi;

  /* 低速采集 */
  sum_vd = 0;
  for (uint32_t i = 0; i < sample_ms; i++) {
    HAL_Delay(1);
    sum_vd += controller_eyou.IncPID_DAxis.OutPut;
    if (controller_eyou.ServoErrFlag.All_Flag) goto fault;
  }
  vd_avg_lo = (float)sum_vd / (float)sample_ms;
  printf("  lo: Vd_pi_avg = %.1f (Q10)\r\n", vd_avg_lo);

  /* 高速采集 */
  printf("  ramping to %ld rpm...\r\n", (long)speed_hi);
  controller_eyou.velocity_ref = vel_hi_pos;
  for (uint32_t i = 0; i < settle_ms; i++) {
    if (controller_eyou.ServoErrFlag.All_Flag) goto fault;
    HAL_Delay(1);
  }
  sum_vd = 0;
  for (uint32_t i = 0; i < sample_ms; i++) {
    HAL_Delay(1);
    sum_vd += controller_eyou.IncPID_DAxis.OutPut;
    if (controller_eyou.ServoErrFlag.All_Flag) goto fault;
  }
  vd_avg_hi = (float)sum_vd / (float)sample_ms;
  printf("  hi: Vd_pi_avg = %.1f (Q10)\r\n", vd_avg_hi);

  /* 计算正转参数 */
  {
    float eps_lo = (vd_avg_lo / 1024.0f) / (we_lo * psi_f);  /* rad */
    float eps_hi = (vd_avg_hi / 1024.0f) / (we_hi * psi_f);
    float eps_lo_cnt = eps_lo * 65536.0f / (2.0f * 3.14159265f);
    float eps_hi_cnt = eps_hi * 65536.0f / (2.0f * 3.14159265f);
    float comp_f = (eps_hi_cnt - eps_lo_cnt) * 10.0f / (dt_hi - dt_lo);
    float off_cnt = eps_lo_cnt - dt_lo * comp_f / 10.0f;
    float off_01deg = off_cnt * 100.0f / 1820.0f;
    new_off_pos  = (int16_t)(off_01deg >= 0 ? off_01deg + 0.5f : off_01deg - 0.5f);
    new_comp_pos = (int16_t)(comp_f >= 0 ? comp_f + 0.5f : comp_f - 0.5f);
    printf("  Pos result: offset=%d (0.1deg), comp=%d\r\n", new_off_pos, new_comp_pos);
  }

  /* ===== 减速停车 ===== */
  controller_eyou.velocity_ref = 0;
  HAL_Delay(1000);
  if (controller_eyou.ServoErrFlag.All_Flag) goto fault;

  /* ===== 反转 ===== */
  printf("Neg direction: ramping to -%ld rpm...\r\n", (long)speed_lo);
  controller_eyou.velocity_ref = -vel_lo_pos;
  for (uint32_t i = 0; i < settle_ms; i++) {
    if (controller_eyou.ServoErrFlag.All_Flag) goto fault;
    HAL_Delay(1);
  }

  /* 低速采集 */
  sum_vd = 0;
  for (uint32_t i = 0; i < sample_ms; i++) {
    HAL_Delay(1);
    sum_vd += controller_eyou.IncPID_DAxis.OutPut;
    if (controller_eyou.ServoErrFlag.All_Flag) goto fault;
  }
  vd_avg_lo = (float)sum_vd / (float)sample_ms;
  printf("  lo: Vd_pi_avg = %.1f (Q10)\r\n", vd_avg_lo);

  /* 高速采集 */
  printf("  ramping to -%ld rpm...\r\n", (long)speed_hi);
  controller_eyou.velocity_ref = -vel_hi_pos;
  for (uint32_t i = 0; i < settle_ms; i++) {
    if (controller_eyou.ServoErrFlag.All_Flag) goto fault;
    HAL_Delay(1);
  }
  sum_vd = 0;
  for (uint32_t i = 0; i < sample_ms; i++) {
    HAL_Delay(1);
    sum_vd += controller_eyou.IncPID_DAxis.OutPut;
    if (controller_eyou.ServoErrFlag.All_Flag) goto fault;
  }
  vd_avg_hi = (float)sum_vd / (float)sample_ms;
  printf("  hi: Vd_pi_avg = %.1f (Q10)\r\n", vd_avg_hi);

  /* 计算反转参数 (反转时 V_d_pi 符号翻转, ε 仍取正值代表补偿量) */
  {
    float eps_lo = -(vd_avg_lo / 1024.0f) / (we_lo * psi_f);  /* 反转取负 */
    float eps_hi = -(vd_avg_hi / 1024.0f) / (we_hi * psi_f);
    float eps_lo_cnt = eps_lo * 65536.0f / (2.0f * 3.14159265f);
    float eps_hi_cnt = eps_hi * 65536.0f / (2.0f * 3.14159265f);
    float comp_f = (eps_hi_cnt - eps_lo_cnt) * 10.0f / (dt_hi - dt_lo);
    float off_cnt = eps_lo_cnt - dt_lo * comp_f / 10.0f;
    float off_01deg = off_cnt * 100.0f / 1820.0f;
    new_off_neg  = (int16_t)(off_01deg >= 0 ? off_01deg + 0.5f : off_01deg - 0.5f);
    new_comp_neg = (int16_t)(comp_f >= 0 ? comp_f + 0.5f : comp_f - 0.5f);
    printf("  Neg result: offset=%d (0.1deg), comp=%d\r\n", new_off_neg, new_comp_neg);
  }

  /* 结果合理性校验 */
  #define PHCOMP_OFF_MAX  400   /* ±40° */
  #define PHCOMP_COMP_MAX 100   /* comp 合理范围 0~100 (≈0~10 个采样延迟) */
  if (abs(new_off_pos) > PHCOMP_OFF_MAX || abs(new_off_neg) > PHCOMP_OFF_MAX ||
      new_comp_pos < -PHCOMP_COMP_MAX || new_comp_pos > PHCOMP_COMP_MAX ||
      new_comp_neg < -PHCOMP_COMP_MAX || new_comp_neg > PHCOMP_COMP_MAX) {
    printf("PhaseComp ident FAILED: result out of range!\r\n");
    printf("  off_pos=%d off_neg=%d comp_pos=%d comp_neg=%d\r\n",
           new_off_pos, new_off_neg, new_comp_pos, new_comp_neg);
    printf("  Valid: |offset|<=%d, comp in [-10,%d]\r\n", PHCOMP_OFF_MAX, PHCOMP_COMP_MAX);
    goto cleanup;
  }

  success = 1;
  goto cleanup;

fault:
  printf("PhaseComp ident FAULT: ServoErrFlag=0x%04X\r\n",
         (unsigned)controller_eyou.ServoErrFlag.All_Flag);

cleanup:
  controller_eyou.velocity_ref = 0;
  HAL_Delay(500);
  controller_eyou.controller_mode = saved_mode;
  controller_eyou.foc_run = saved_run;
  Threshold.velocity_coe = saved_velocity_coe;
  Threshold.PhaseLossActiveIq = saved_phase_loss_iq;

  if (success) {
    g_theta_offset_pos = new_off_pos;
    g_theta_offset_neg = new_off_neg;
    g_theta_comp_pos   = new_comp_pos;
    g_theta_comp_neg   = new_comp_neg;
    SavePhaseCompToFlash();
    printf("\r\n=== Phase Comp Auto-Ident SUCCESS ===\r\n");
    printf("  Pos: offset=%d (0.1deg=%.1fdeg), comp=%d\r\n",
           new_off_pos, new_off_pos * 0.1f, new_comp_pos);
    printf("  Neg: offset=%d (0.1deg=%.1fdeg), comp=%d\r\n",
           new_off_neg, new_off_neg * 0.1f, new_comp_neg);
    printf("  (Saved to Flash)\r\n");
  } else {
    /* 辨识失败,恢复原参数 */
    g_theta_offset_pos = saved_off_pos;
    g_theta_offset_neg = saved_off_neg;
    g_theta_comp_pos   = saved_comp_pos;
    g_theta_comp_neg   = saved_comp_neg;
    printf("Phase comp restored to previous values\r\n");
  }
}
