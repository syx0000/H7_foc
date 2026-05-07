/**
 * @file    ifly_inertia_ident.h
 * @brief   惯量辨识 — 任务级阻塞编排
 *
 *   折合到电机轴的机械方程 (含负载/摩擦):
 *       Te = J·dω_m/dt + B·ω_m + T_load + T_friction(sign(ω_m))
 *
 *   双向对称加速 + 多周期平均, 差分消去 T_load / B·ω / T_f:
 *       J = (mean(Te+) - mean(Te-)) / (mean(a+) - mean(a-))
 *
 *   流程: 切 CST 模式 → +Iq 加速到 +ω_th → settle → -Iq 反向加速到 -ω_th
 *          → settle → +Iq → ... 重复 cycles 周期 → 收尾恢复现场
 *   不动任何 ISR; 运行期被低优先级任务 vTaskDelay 让出 CPU。
 */

#ifndef __IFLY_INERTIA_IDENT_H__
#define __IFLY_INERTIA_IDENT_H__

#include <stdint.h>
#include "foc_controller.h"

typedef enum {
  INERTIA_IDENT_OK            = 0,
  INERTIA_IDENT_ERR_TIMEOUT   = 1,  /* 加速段没在限时内到达目标速度 */
  INERTIA_IDENT_ERR_PARAM     = 2,  /* 入参非法 (psi_f<=0, iq_step=0, ...) */
  INERTIA_IDENT_ERR_FAULT     = 3,  /* 辨识途中触发故障 */
  INERTIA_IDENT_ERR_DIVERGE   = 4,  /* (a+ - a-) 太小, 结果发散 */
} InertiaIdentErr;

typedef struct {
  int32_t  iq_step;            /* Q10 A, 默认 ±2*1024 = ±2 A */
  int32_t  speed_threshold;    /* 载端内部值 (rpm × 1024 × 101), 默认 20 rpm */
  uint32_t settle_ms;          /* 反向后等暂态结束, 默认 100 ms */
  uint32_t accel_timeout_ms;   /* 单段加速到 speed_threshold 上限, 默认 1500 ms */
  uint32_t cycles;             /* 双向周期数, 默认 10 */
  uint32_t warmup_cycles;      /* 前 N 周期不计入统计 (摩擦/起动暖机), 默认 2 */
  float    psi_f;              /* [Wb] 磁链, 必填 */
  float    Ld;                 /* [H] d 轴电感, 必填 */
  float    Lq;                 /* [H] q 轴电感, 必填 */
} InertiaIdentCfg;

typedef struct {
  InertiaIdentErr err_code;
  float    J;                  /* [kg·m²] 折合到电机轴的总惯量 */
  float    a_pos_avg;          /* [rad/s²] 正向加速段平均加速度 (机械) */
  float    a_neg_avg;          /* [rad/s²] 反向加速段平均加速度 (机械) */
  float    Te_pos_avg;         /* [N·m] 正向加速段平均电磁转矩 */
  float    Te_neg_avg;         /* [N·m] 反向加速段平均电磁转矩 */
  uint32_t cycles_used;        /* 实际成功完成的周期数 */
} InertiaIdentResult;

/* 给 cfg 填一组保守默认值;调用方按需覆盖。psi_f/Ld/Lq 留 0,必须由调用者填。 */
void inertia_ident_default_cfg(InertiaIdentCfg *cfg);

/* 阻塞执行一次惯量辨识;期间用 vTaskDelay 让出 CPU,不卡 RTOS。
 * 返回 0 成功,非 0 见 InertiaIdentErr。结果写入 *result。 */
InertiaIdentErr runInertiaIdent(ControllerStruct *c,
                                const InertiaIdentCfg *cfg,
                                InertiaIdentResult *result);

#endif
