#ifndef __IFLY_FLUX_IDENT_H__
#define __IFLY_FLUX_IDENT_H__

#include <stdint.h>
#include "foc_controller.h"

/* 磁链辨识 — 任务级编排 (方案 C)
 *
 *   稳态 q 轴电压方程: u_q = R_s i_q + ω_e L_d i_d + ω_e ψ_f
 *   两个不同 i_d 工况(同 ω_e、同 R_s、同 L_d)消去得:
 *
 *       ψ_f = ((u_q1 - R_s i_q1) i_d2 - (u_q2 - R_s i_q2) i_d1)
 *             / (ω_e (i_d2 - i_d1))
 *
 *   流程: 切速度模式 → 速度环拉到目标转速 → 等稳态 → 设 id_ref_1 采窗口1
 *          → 切 id_ref_2 等 settle → 采窗口2 → 计算 → 收尾
 *   不动任何 ISR;运行期被低优先级任务阻塞调用 (vTaskDelay 让出 CPU)。
 */

typedef enum {
  FLUX_IDENT_OK            = 0,
  FLUX_IDENT_ERR_TIMEOUT   = 1,  /* 速度未在限时内稳定 */
  FLUX_IDENT_ERR_LOW_SPEED = 2,  /* |ω_e| 太低,公式发散 */
  FLUX_IDENT_ERR_DELTA_ID  = 3,  /* |i_d2 - i_d1| 太小,放大噪声 */
  FLUX_IDENT_ERR_FAULT     = 4,  /* 辨识途中触发故障 */
} FluxIdentErr;

typedef struct {
  int32_t  id_ref_1;        /* Q10 A,建议 0 */
  int32_t  id_ref_2;        /* Q10 A,建议 ±1024(±1A),与 id_ref_1 拉开避免饱和差异 */
  int32_t  speed_target;    /* 内部速度量 (rpm * 1024 * 101) */
  uint32_t accel_ms;        /* 从 0 缓升到 speed_target 的时间,缓解速度环冲击/EMI */
  uint32_t decel_ms;        /* 从 speed_target 缓降到 0 的时间(收尾阶段) */
  uint32_t settle_ms;       /* 切 id_ref 后等稳态时间 */
  uint32_t measure_ms;      /* 每窗口测量时长 */
  uint32_t speed_steady_timeout_ms;  /* 等待速度稳态的最大时长 */
  float    rs_known;        /* 已知 R_s [Ω],可来自 measurePhaseResistance / Flash */
} FluxIdentCfg;

typedef struct {
  FluxIdentErr err_code;
  float        flux;        /* ψ_f [Wb] */
  /* 调试中间量 */
  float        uq1, uq2;    /* [V] */
  float        iq1, iq2;    /* [A] */
  float        id1, id2;    /* [A] */
  float        we;          /* [rad/s] 电角速度,两窗口平均 */
} FluxIdentResult;

/* 给 Cfg 填一组保守的默认值;调用方按需覆盖 */
void flux_ident_default_cfg(FluxIdentCfg *cfg);

/* 阻塞执行一次磁链辨识;期间用 vTaskDelay 让出 CPU,不卡 RTOS。
 * 返回 0 成功,非 0 见 FluxIdentErr。结果写入 *result。 */
FluxIdentErr runFluxIdent(ControllerStruct *controller,
                          const FluxIdentCfg *cfg,
                          FluxIdentResult *result);

#endif
