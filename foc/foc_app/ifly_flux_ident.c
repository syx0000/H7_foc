/**
 * @file    ifly_flux_ident.c
 * @brief   磁链辨识 — 任务级阻塞编排 (方案 C)
 *
 *   ψ_f = ((u_q1 − R_s i_q1) i_d2 − (u_q2 − R_s i_q2) i_d1) / (ω_e (i_d2 − i_d1))
 *
 *   单位约定(与项目内现有变量一致):
 *     I_d/I_q             : Q10 A   (1024 = 1 A)
 *     V_d/V_q             : Q10 V   (1024 = 1 V)
 *     dtheta_mech         : 电机端 rpm × 1024  (即 1 电机 rpm = 1024)
 *     velocity_ref        : 载端 rpm × 1024 × 101  (内部含减速比 101)
 *     两者数值同尺度: 内部值 1024×101 既是 "载端 1 rpm" 也是 "电机端 101 rpm × 1024"
 *
 *   ω_e [rad/s] = (dtheta_mech / 1024) * (2π/60) * NPP    (用电机端 rpm 算电气角速度)
 */

#include "ifly_flux_ident.h"
#include "foc_api.h"
// #include "FreeRTOS.h"  /* FreeRTOS removed */
// #include "task.h"  /* FreeRTOS removed */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

extern uint8_t NPP;

/* 单位换算常量(语义严格分离):
 *   LOAD_RPM_INTERNAL : 载端 rpm → velocity_ref / dtheta_mech 内部值(已含减速比 101)
 *   MOTOR_RPM_Q10     : 电机端 rpm × 1024 = dtheta_mech 物理意义(用于电气 ω_e 换算)
 */
#define LOAD_RPM_INTERNAL        (1024.0f * 101.0f)
#define MOTOR_RPM_Q10            (1024.0f)
#define MIN_WE_RAD_S             (5.0f)          /* 低于该 ω_e 不做辨识(避免分母过小) */
#define MIN_DELTA_ID_Q10         (256)           /* 0.25 A,低于则放大噪声 */
#define SPEED_STABLE_TOL_RATIO   (5)             /* 速度稳态判定容差 = 5% × 目标 */
#define SPEED_STABLE_MIN_TOL     ((int32_t)(0.5f * LOAD_RPM_INTERNAL))  /* 至少 0.5 rpm */
#define SPEED_STABLE_HOLD_MS     (100u)          /* 连续 100 ms 在容差内才判稳 */

/* I_q 饱和保护: 抱闸阻塞 / 编码器异常 / 角偏置错时,速度环 PID 会饱和到
 * INC_PID_SPEED_LIMIT (= DEFAULT_MAX_CURRENT = 25 A Q10)。这里在速度未稳态阶段
 * 同步监测 |I_q|,持续 IQ_SAT_HOLD_MS 超阈值即放弃,避免 30ms 触发堵转保护->复位。 */
#define IQ_SAT_THRESHOLD_Q10     ((int32_t)(0.85f * (float)DEFAULT_MAX_CURRENT))
#define IQ_SAT_HOLD_MS           (50u)

/* 内部累加器:稳态平均所用 */
typedef struct {
  int64_t  sum_uq;
  int64_t  sum_iq;
  int64_t  sum_id;
  int64_t  sum_dtheta;
  uint32_t n;
} FluxAcc;

void flux_ident_default_cfg(FluxIdentCfg *cfg) {
  cfg->id_ref_1                = 0;                                  /* 0 A */
  cfg->id_ref_2                = 1024;                               /* 1 A */
  cfg->speed_target            = (int32_t)(20.0f * LOAD_RPM_INTERNAL); /* 20 rpm */
  cfg->accel_ms                = 4000;                               /* 4 s 加速,~5 rpm/s */
  cfg->decel_ms                = 2000;                               /* 2 s 减速 */
  cfg->settle_ms               = 300;
  cfg->measure_ms              = 300;
  cfg->speed_steady_timeout_ms = 5000;
  cfg->rs_known                = 0.0f;
}

/* 速度参考线性斜坡: total_ms 内由 from 平滑过渡到 to。
 * 期间检查故障/Iq 饱和,触发即返回 -1。 */
static int ramp_velocity(ControllerStruct *c, int32_t from, int32_t to,
                         uint32_t total_ms) {
  uint32_t iq_sat_run = 0;
  for (uint32_t i = 1; i <= total_ms; ++i) {
    if (c->ServoErrFlag.All_Flag) return -1;
    if (abs(c->I_q) > IQ_SAT_THRESHOLD_Q10) {
      if (++iq_sat_run >= IQ_SAT_HOLD_MS) {
        printf("Flux ident: I_q saturated during ramp (%.2fA), abort\r\n",
               (float)c->I_q / 1024.0f);
        return -1;
      }
    } else {
      iq_sat_run = 0;
    }
    c->velocity_ref = from + (int32_t)(((int64_t)(to - from) * i) / total_ms);
    HAL_Delay(1);
  }
  return 0;
}

static int wait_speed_steady(ControllerStruct *c, int32_t target,
                             uint32_t timeout_ms) {
  int32_t  abs_target = abs(target);
  int32_t  tol        = abs_target / (100 / SPEED_STABLE_TOL_RATIO);
  if (tol < SPEED_STABLE_MIN_TOL) tol = SPEED_STABLE_MIN_TOL;

  uint32_t hold       = 0;
  uint32_t iq_sat_run = 0;
  while (timeout_ms--) {
    if (c->ServoErrFlag.All_Flag) return -1;
    /* I_q 饱和保护: 防止抱闸/编码器异常拉爆 PID 触发堵转保护 */
    if (abs(c->I_q) > IQ_SAT_THRESHOLD_Q10) {
      if (++iq_sat_run >= IQ_SAT_HOLD_MS) {
        printf("Flux ident: I_q saturated (%.2fA), abort\r\n",
               (float)c->I_q / 1024.0f);
        return -1;
      }
    } else {
      iq_sat_run = 0;
    }
    if (abs(c->dtheta_mech - target) < tol) {
      if (++hold >= SPEED_STABLE_HOLD_MS) return 0;
    } else {
      hold = 0;
    }
    HAL_Delay(1);
  }
  return -1;
}

static int collect_window(ControllerStruct *c, uint32_t ms, FluxAcc *acc) {
  acc->sum_uq = acc->sum_iq = acc->sum_id = acc->sum_dtheta = 0;
  acc->n      = 0;
  for (uint32_t i = 0; i < ms; ++i) {
    if (c->ServoErrFlag.All_Flag) return -1;
    acc->sum_uq     += c->V_q;
    acc->sum_iq     += c->I_q;
    acc->sum_id     += c->I_d;
    acc->sum_dtheta += c->dtheta_mech;
    acc->n++;
    HAL_Delay(1);
  }
  return acc->n > 0 ? 0 : -1;
}

static void average_window(const FluxAcc *acc,
                           float *uq, float *iq, float *id, float *we) {
  float inv_n     = 1.0f / (float)acc->n;
  *uq = ((float)acc->sum_uq * inv_n) / 1024.0f;
  *iq = ((float)acc->sum_iq * inv_n) / 1024.0f;
  *id = ((float)acc->sum_id * inv_n) / 1024.0f;
  /* dtheta_mech / 1024 = 电机端 rpm,直接乘 NPP 得电气角速度 */
  float motor_rpm = ((float)acc->sum_dtheta * inv_n) / MOTOR_RPM_Q10;
  *we = motor_rpm * (2.0f * (float)M_PI / 60.0f) * (float)NPP;
}

FluxIdentErr runFluxIdent(ControllerStruct *c,
                          const FluxIdentCfg *cfg,
                          FluxIdentResult *r) {
  /* 保存现场(velocity_ref 不恢复,收尾统一置 0) */
  uint8_t saved_mode         = c->controller_mode;
  int32_t saved_id_ref       = c->I_d_ref;

  r->err_code = FLUX_IDENT_OK;
  r->flux    = 0.0f;

  /* 切速度模式,使能 FOC,velocity_ref 从 0 起步避免冲击 */
  c->controller_mode = CYCLIC_SYNC_VELOCITY_MODE;
  c->velocity_ref    = 0;
  c->I_d_ref         = cfg->id_ref_1;
  c->foc_run         = 1;
  // printf("Flux ident: ramping up over %ums...\r\n", (unsigned)cfg->accel_ms);

  /* 1.a) 缓加速到目标转速 */
  if (ramp_velocity(c, 0, cfg->speed_target, cfg->accel_ms) != 0) {
    r->err_code = c->ServoErrFlag.All_Flag ? FLUX_IDENT_ERR_FAULT
                                           : FLUX_IDENT_ERR_FAULT;
    goto cleanup;
  }
  // printf("Flux ident: speed at target, waiting steady...\r\n");

  /* 1.b) 等转速稳态 */
  if (wait_speed_steady(c, cfg->speed_target, cfg->speed_steady_timeout_ms) != 0) {
    r->err_code = c->ServoErrFlag.All_Flag ? FLUX_IDENT_ERR_FAULT
                                           : FLUX_IDENT_ERR_TIMEOUT;
    goto cleanup;
  }
  // printf("Flux ident: speed steady, sampling window 1...\r\n");

  /* 2) 窗口1: id = id_ref_1 */
  FluxAcc acc1;
  if (collect_window(c, cfg->measure_ms, &acc1) != 0) {
    r->err_code = FLUX_IDENT_ERR_FAULT;
    goto cleanup;
  }
  average_window(&acc1, &r->uq1, &r->iq1, &r->id1, &r->we);
  float we1 = r->we;
  // printf("Flux ident: window 1 done, switching id_ref...\r\n");

  /* 3) 切 id_ref_2,等 settle 让电流环跟踪到位 */
  c->I_d_ref = cfg->id_ref_2;
  for (uint32_t i = 0; i < cfg->settle_ms; ++i) {
    if (c->ServoErrFlag.All_Flag) { r->err_code = FLUX_IDENT_ERR_FAULT; goto cleanup; }
    HAL_Delay(1);
  }

  /* 4) 窗口2: id = id_ref_2 */
  // printf("Flux ident: sampling window 2...\r\n");
  FluxAcc acc2;
  if (collect_window(c, cfg->measure_ms, &acc2) != 0) {
    r->err_code = FLUX_IDENT_ERR_FAULT;
    goto cleanup;
  }
  float we2;
  average_window(&acc2, &r->uq2, &r->iq2, &r->id2, &we2);
  r->we = 0.5f * (we1 + we2);

  /* 5) 校验 */
  if (fabsf(r->we) < MIN_WE_RAD_S) {
    r->err_code = FLUX_IDENT_ERR_LOW_SPEED;
    goto cleanup;
  }
  if (fabsf((float)(cfg->id_ref_2 - cfg->id_ref_1)) < (float)MIN_DELTA_ID_Q10) {
    r->err_code = FLUX_IDENT_ERR_DELTA_ID;
    goto cleanup;
  }

  /* 6) 计算 ψ_f (Rs 补偿) */
  float Rs   = cfg->rs_known;
  float num  = (r->uq1 - Rs * r->iq1) * r->id2
             - (r->uq2 - Rs * r->iq2) * r->id1;
  float den  = r->we * (r->id2 - r->id1);
  r->flux    = num / den;

  /* 7) 成功路径缓减速回零,避免突然停止冲击 */
  c->I_d_ref = 0;
  // printf("Flux ident: ramping down over %ums...\r\n", (unsigned)cfg->decel_ms);
  ramp_velocity(c, cfg->speed_target, 0, cfg->decel_ms);

cleanup:
  /* 收尾:I_d_ref 清零、velocity_ref 直接置零(失败路径由外层 motorStopProgress 兜底) */
  c->I_d_ref         = saved_id_ref;
  c->velocity_ref    = 0;
  c->controller_mode = saved_mode;
  return r->err_code;
}
