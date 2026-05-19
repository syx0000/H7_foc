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
  controller_eyou.I_q_ref         = 512;
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
