/**
 * @file    ifly_inertia_ident.c
 * @brief   惯量辨识 — 任务级阻塞编排 (coast-down 法)
 *
 *   每周期 4 段, 每方向 (+/-) 各采一对 drive/coast:
 *       drive: Iq=±iq_step, ω 从 ω_lo 加速到 ω_hi → 测 a_drive 与 Te_drive
 *       coast: Iq=0,        ω 从 ω_hi 减速回 ω_lo → 测 a_coast (Te=0)
 *
 *   两段动力学:
 *       drive: J·a_drive = Te_drive - Tf - B·ω
 *       coast: J·a_coast =          - Tf - B·ω
 *   差分: a_drive - a_coast = Te_drive / J  (Tf 与 Bω 严格相消)
 *
 *   每周期 J_k = 0.5 × (J+ + J-), J± = Te_drv± / (a_drv± - a_cst±)。
 *   多周期平均, 前 warmup_cycles 周期不入统计。
 *
 *   单位约定 (与项目内现有变量一致):
 *     I_q                : Q10 A   (1024 = 1 A)
 *     dtheta_mech        : 电机端 rpm × 1024
 *     speed_threshold    : 载端 rpm × 1024 × 101 (内部含减速比 101)
 *
 *   ω_m [rad/s] = (dtheta_mech / 1024) * (2π/60)   (机械角速度,不乘 NPP)
 *   Te [N·m]   = 1.5 * NPP * (psi_f * Iq + (Ld - Lq) * Id * Iq)
 */

#include "ifly_inertia_ident.h"
#include "foc_api.h"
// #include "FreeRTOS.h"  /* FreeRTOS removed */
// #include "task.h"  /* FreeRTOS removed */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

extern uint8_t NPP;
extern ifly_Err_Pro_Type motorProValue;

/* 默认参数
 * 与对称双向法不同, coast-down 不需要靠 Iq 大小压低 Tf/Te 偏置 (摩擦项已被
 * 差分消掉), 只需保证起动力矩压过静摩擦/抱闸残余即可。 */
#define DEFAULT_IQ_STEP_Q10        (1024+360)                              /* 1.35 A */
#define DEFAULT_SPEED_TH_INTERNAL  ((int32_t)(20.0f * 1024.0f * 101.0f))   /* 载端 20 rpm */
#define DEFAULT_SETTLE_MS          (100u)
#define DEFAULT_ACCEL_TIMEOUT_MS   (5000u)
#define DEFAULT_CYCLES             (10u)
#define DEFAULT_WARMUP_CYCLES      (2u)

/* 校验阈值 */
#define MIN_DA_RAD_S2              (1.0f)   /* (a_drive - a_coast) 下限 */
#define OMEGA_LO_RATIO             (0.2f)   /* ω_lo = 0.2 * ω_th, drive/coast 共享窗口 */

/* 单位换算 */
#define MOTOR_RPM_Q10              (1024.0f)
#define RPM_TO_RAD_S               (2.0f * (float)M_PI / 60.0f)

typedef struct {
  int64_t  sum_iq;            /* Q10 A 累加 (drive 用; coast 由于 Iq=0 累加值~0) */
  int64_t  sum_id;
  int32_t  dtheta_start;
  int32_t  dtheta_end;
  uint32_t n;                 /* 采样数 = 耗时 ms */
} InertiaAcc;

void inertia_ident_default_cfg(InertiaIdentCfg *cfg) {
  cfg->iq_step          = DEFAULT_IQ_STEP_Q10;
  cfg->speed_threshold  = DEFAULT_SPEED_TH_INTERNAL;
  cfg->settle_ms        = DEFAULT_SETTLE_MS;
  cfg->accel_timeout_ms = DEFAULT_ACCEL_TIMEOUT_MS;
  cfg->cycles           = DEFAULT_CYCLES;
  cfg->warmup_cycles    = DEFAULT_WARMUP_CYCLES;
  cfg->psi_f            = 0.0f;
  cfg->Ld               = 0.0f;
  cfg->Lq               = 0.0f;
}

/* 等 ms 毫秒,期间检查故障。故障返回 -1,正常返回 0 */
static int wait_ms_check(ControllerStruct *c, uint32_t ms) {
  while (ms--) {
    if (c->ServoErrFlag.All_Flag) {
      printf("Inertia ident: wait fault Udc=%d flag=0x%08X dtheta=%d Iq=%d\r\n",
             (int)motorProValue.Udc, (unsigned)c->ServoErrFlag.All_Flag,
             (int)c->dtheta_mech, (int)c->I_q);
      return -1;
    }
    HAL_Delay(1);
  }
  return 0;
}

/* 等待 dtheta_mech 跨过门限 (不入累加, 仅作同步等待)。
 * reach_above=1: 等到 dtheta_mech >= target_signed
 * reach_above=0: 等到 dtheta_mech <= target_signed
 * 故障/超时返回 -1 */
static int wait_speed_cross(ControllerStruct *c,
                            int32_t target_signed,
                            uint8_t reach_above,
                            uint32_t timeout_ms) {
  while (timeout_ms--) {
    if (c->ServoErrFlag.All_Flag) {
      printf("Inertia ident: wait_speed fault Udc=%d flag=0x%08X dtheta=%d/%d Iq=%d\r\n",
             (int)motorProValue.Udc, (unsigned)c->ServoErrFlag.All_Flag,
             (int)c->dtheta_mech, (int)target_signed, (int)c->I_q);
      return -1;
    }
    if (reach_above) {
      if (c->dtheta_mech >= target_signed) return 0;
    } else {
      if (c->dtheta_mech <= target_signed) return 0;
    }
    HAL_Delay(1);
  }
  return -1;
}

/* 累加 Iq/dtheta, 直到 dtheta_mech 跨过门限。
 * reach_above=1: 等到 dtheta_mech >= target_signed
 * reach_above=0: 等到 dtheta_mech <= target_signed */
static int collect_to_speed(ControllerStruct *c,
                            int32_t target_signed,
                            uint8_t reach_above,
                            uint32_t timeout_ms,
                            InertiaAcc *acc) {
  acc->sum_iq       = 0;
  acc->sum_id       = 0;
  acc->dtheta_start = c->dtheta_mech;
  acc->dtheta_end   = c->dtheta_mech;
  acc->n            = 0;
  while (timeout_ms--) {
    if (c->ServoErrFlag.All_Flag) {
      printf("Inertia ident: collect fault Udc=%d flag=0x%08X dtheta=%d/%d Iq=%d n=%u\r\n",
             (int)motorProValue.Udc, (unsigned)c->ServoErrFlag.All_Flag,
             (int)c->dtheta_mech, (int)target_signed,
             (int)c->I_q, (unsigned)acc->n);
      return -1;
    }
    acc->sum_iq    += c->I_q;
    acc->sum_id    += c->I_d;
    acc->dtheta_end = c->dtheta_mech;
    acc->n++;
    if (reach_above) {
      if (c->dtheta_mech >= target_signed) return 0;
    } else {
      if (c->dtheta_mech <= target_signed) return 0;
    }
    HAL_Delay(1);
  }
  return -1;
}

/* 算平均 Te 和平均加速度 (机械, rad/s²)
 * Te = 1.5 * NPP * (psi_f * Iq + (Ld - Lq) * Id * Iq)
 * coast 段 Iq~0, Te 也~0 */
static void avg_segment(const InertiaAcc *acc,
                        const InertiaIdentCfg *cfg,
                        float *Te, float *a) {
  if (acc->n == 0) { *Te = 0.0f; *a = 0.0f; return; }
  float inv_n  = 1.0f / (float)acc->n;
  float Iq_avg = ((float)acc->sum_iq * inv_n) / 1024.0f;
  float Id_avg = ((float)acc->sum_id * inv_n) / 1024.0f;
  *Te = 1.5f * (float)NPP *
        (cfg->psi_f * Iq_avg + (cfg->Ld - cfg->Lq) * Id_avg * Iq_avg);

  float dtheta_delta = (float)(acc->dtheta_end - acc->dtheta_start);
  float dt_s         = (float)acc->n * 0.001f;                /* 1 ms 节拍 */
  float omega_delta  = (dtheta_delta / MOTOR_RPM_Q10) * RPM_TO_RAD_S;
  *a = omega_delta / dt_s;
}

InertiaIdentErr runInertiaIdent(ControllerStruct *c,
                                const InertiaIdentCfg *cfg,
                                InertiaIdentResult *r) {
  /* 入参校验 */
  r->err_code    = INERTIA_IDENT_OK;
  r->J           = 0.0f;
  r->a_pos_avg   = 0.0f;
  r->a_neg_avg   = 0.0f;
  r->Te_pos_avg  = 0.0f;
  r->Te_neg_avg  = 0.0f;
  r->cycles_used = 0;

  if (cfg->psi_f <= 0.0f || cfg->iq_step == 0 ||
      cfg->cycles == 0 || cfg->accel_timeout_ms == 0) {
    r->err_code = INERTIA_IDENT_ERR_PARAM;
    return r->err_code;
  }

  /* ω_lo = OMEGA_LO_RATIO * ω_th: drive 与 coast 共享 [ω_lo, ω_th] 窗口,
   * 让两侧平均 ω 相同, 粘性摩擦 Bω 项严格相消 */
  int32_t omega_lo = (int32_t)((float)cfg->speed_threshold * OMEGA_LO_RATIO);

  /* 备份现场 */
  uint8_t saved_mode    = c->controller_mode;
  int32_t saved_iq_ref  = c->I_q_ref;
  int32_t saved_id_ref  = c->I_d_ref;

  /* 切 CST 模式, I_d_ref=0, foc_run=1 */
  c->controller_mode = CYCLIC_SYNC_TORQUE_MODE;
  c->I_d_ref         = 0;
  c->I_q_ref         = 0;
  c->foc_run         = 1;

  /* 累加器（提前声明以避免 goto cleanup 跨过初始化） */
  double sum_J          = 0.0;
  double sum_a_drv_pos  = 0.0, sum_a_drv_neg  = 0.0;
  double sum_Te_drv_pos = 0.0, sum_Te_drv_neg = 0.0;

  /* 初始稳定 */
  if (wait_ms_check(c, 50) != 0) {
    r->err_code = INERTIA_IDENT_ERR_FAULT; goto cleanup;
  }

  // printf("Inertia ident: coast-down, cycles=%u (warmup %u), omega [%d, %d]\r\n",
  //        (unsigned)cfg->cycles, (unsigned)cfg->warmup_cycles,
  //        (int)omega_lo, (int)cfg->speed_threshold);

  for (uint32_t k = 0; k < cfg->cycles; ++k) {
    InertiaAcc drive_pos, coast_pos, drive_neg, coast_neg;

    /* === drive+ : 从当前 ω 加速到 +ω_th, 中段 [ω_lo, ω_th] 入统计 === */
    c->I_q_ref = cfg->iq_step;
    if (wait_speed_cross(c, omega_lo, 1, cfg->accel_timeout_ms) != 0) {
      r->err_code = c->ServoErrFlag.All_Flag ? INERTIA_IDENT_ERR_FAULT
                                             : INERTIA_IDENT_ERR_TIMEOUT;
      printf("Inertia ident: cycle %u drive+ wait_lo fail\r\n", (unsigned)k);
      goto cleanup;
    }
    if (collect_to_speed(c, cfg->speed_threshold, 1, cfg->accel_timeout_ms,
                         &drive_pos) != 0) {
      r->err_code = c->ServoErrFlag.All_Flag ? INERTIA_IDENT_ERR_FAULT
                                             : INERTIA_IDENT_ERR_TIMEOUT;
      printf("Inertia ident: cycle %u drive+ collect fail\r\n", (unsigned)k);
      goto cleanup;
    }

    /* === coast+ : Iq=0, 从 ω_th 自由减速回 ω_lo === */
    c->I_q_ref = 0;
    if (wait_ms_check(c, cfg->settle_ms) != 0) {
      r->err_code = INERTIA_IDENT_ERR_FAULT; goto cleanup;
    }
    if (collect_to_speed(c, omega_lo, 0, cfg->accel_timeout_ms,
                         &coast_pos) != 0) {
      r->err_code = c->ServoErrFlag.All_Flag ? INERTIA_IDENT_ERR_FAULT
                                             : INERTIA_IDENT_ERR_TIMEOUT;
      printf("Inertia ident: cycle %u coast+ fail\r\n", (unsigned)k);
      goto cleanup;
    }

    /* === drive- : 从当前 ω 加速到 -ω_th, 中段 [-ω_lo, -ω_th] 入统计 === */
    c->I_q_ref = -cfg->iq_step;
    if (wait_speed_cross(c, -omega_lo, 0, cfg->accel_timeout_ms) != 0) {
      r->err_code = c->ServoErrFlag.All_Flag ? INERTIA_IDENT_ERR_FAULT
                                             : INERTIA_IDENT_ERR_TIMEOUT;
      printf("Inertia ident: cycle %u drive- wait_lo fail\r\n", (unsigned)k);
      goto cleanup;
    }
    if (collect_to_speed(c, -cfg->speed_threshold, 0, cfg->accel_timeout_ms,
                         &drive_neg) != 0) {
      r->err_code = c->ServoErrFlag.All_Flag ? INERTIA_IDENT_ERR_FAULT
                                             : INERTIA_IDENT_ERR_TIMEOUT;
      printf("Inertia ident: cycle %u drive- collect fail\r\n", (unsigned)k);
      goto cleanup;
    }

    /* === coast- : Iq=0, 从 -ω_th 自由减速回 -ω_lo === */
    c->I_q_ref = 0;
    if (wait_ms_check(c, cfg->settle_ms) != 0) {
      r->err_code = INERTIA_IDENT_ERR_FAULT; goto cleanup;
    }
    if (collect_to_speed(c, -omega_lo, 1, cfg->accel_timeout_ms,
                         &coast_neg) != 0) {
      r->err_code = c->ServoErrFlag.All_Flag ? INERTIA_IDENT_ERR_FAULT
                                             : INERTIA_IDENT_ERR_TIMEOUT;
      printf("Inertia ident: cycle %u coast- fail\r\n", (unsigned)k);
      goto cleanup;
    }

    /* 算 Te / a, 两侧 J */
    float a_drv_pos, Te_drv_pos, a_cst_pos, Te_cst_pos;
    float a_drv_neg, Te_drv_neg, a_cst_neg, Te_cst_neg;
    avg_segment(&drive_pos, cfg, &Te_drv_pos, &a_drv_pos);
    avg_segment(&coast_pos, cfg, &Te_cst_pos, &a_cst_pos);
    avg_segment(&drive_neg, cfg, &Te_drv_neg, &a_drv_neg);
    avg_segment(&coast_neg, cfg, &Te_cst_neg, &a_cst_neg);
    (void)Te_cst_pos; (void)Te_cst_neg;     /* coast Te ~0, 仅 drive Te 入公式 */

    float da_pos = a_drv_pos - a_cst_pos;
    float da_neg = a_drv_neg - a_cst_neg;
    float J_pos  = (fabsf(da_pos) > MIN_DA_RAD_S2) ? Te_drv_pos / da_pos : 0.0f;
    float J_neg  = (fabsf(da_neg) > MIN_DA_RAD_S2) ? Te_drv_neg / da_neg : 0.0f;
    float J_k    = 0.5f * (J_pos + J_neg);

    /* per-cycle 诊断: drive/coast 加速度 + 两侧 J + 合并 J */
    {
      //const char *tag = (k < cfg->warmup_cycles) ? " (warmup)" : "";
      //printf("  cycle %u%s: drv+/-=%.1f/%.1f cst+/-=%.1f/%.1f Te+/-=%.4f/%.4f "
      //       "J+/-=%.3e/%.3e Jk=%.3e\r\n",
      //       (unsigned)k, tag,
      //       a_drv_pos, a_drv_neg, a_cst_pos, a_cst_neg,
      //       Te_drv_pos, Te_drv_neg,
      //       J_pos, J_neg, J_k);
    }

    if (k < cfg->warmup_cycles) continue;

    sum_a_drv_pos  += (double)a_drv_pos;
    sum_a_drv_neg  += (double)a_drv_neg;
    sum_Te_drv_pos += (double)Te_drv_pos;
    sum_Te_drv_neg += (double)Te_drv_neg;
    sum_J          += (double)J_k;
    r->cycles_used++;
  }

cleanup:
  /* 收尾: 把 I_q_ref 拉零, 等转子停下, 恢复现场 */
  c->I_q_ref = 0;
  HAL_Delay(100);
  c->controller_mode = saved_mode;
  c->I_q_ref         = saved_iq_ref;
  c->I_d_ref         = saved_id_ref;

  if (r->err_code != INERTIA_IDENT_OK) return r->err_code;

  if (r->cycles_used == 0) {
    r->err_code = INERTIA_IDENT_ERR_TIMEOUT;
    return r->err_code;
  }

  float n = (float)r->cycles_used;
  r->a_pos_avg  = (float)(sum_a_drv_pos  / n);
  r->a_neg_avg  = (float)(sum_a_drv_neg  / n);
  r->Te_pos_avg = (float)(sum_Te_drv_pos / n);
  r->Te_neg_avg = (float)(sum_Te_drv_neg / n);
  r->J          = (float)(sum_J / n);

  if (!isfinite(r->J) || r->J <= 0.0f) {
    r->err_code = INERTIA_IDENT_ERR_DIVERGE;
  }
  return r->err_code;
}
