/**
 * @file    foc_current_loop.c
 * @brief
 * @author  yxsui
 * @date    2025-07-31
 * @version 1.0
 */

#include "foc_current_loop.h"
#include "foc_api.h"
#include "can_wly.h"
extern Portection_Value Threshold;
extern ControllerStruct controller_eyou;
/*******************************************************************************
  : foc_current_close_loop
    :
    :
  :
    :
********************************************************************************/
void foc_current_close_loop(ControllerStruct* controller) {
  phase_current_sample(controller);    //

  // Ia Ib Ic Id Iq
  clarke_transf(controller->I_a, controller->I_b, &controller->I_alpha, &controller->I_beta);
  park_transf(controller->I_alpha, controller->I_beta, &controller->I_d, &controller->I_q, controller->theta_elec);

  /****************************辨识模式：旁路PI****************************************/
  if (controller->ident_test.enable) {
    InductanceIdent* ident = &controller->ident_test;

    if (ident->amplitude > 0) {
      // AC模式：正弦注入，用于电感辨识
      uint16_t angle = (uint16_t)(ident->phase_accum >> 16);
      Trig_Components sc = get_sincos_value((int32_t)angle);
      int16_t v_inj = (int16_t)(((int32_t)ident->amplitude * sc.hSin) >> 15);

      if (ident->axis == 0) {
        controller->V_d = v_inj;  controller->V_q = 0;
      } else {
        controller->V_d = 0;      controller->V_q = v_inj;
      }

      int32_t i_meas = (ident->axis == 0) ? controller->I_d : controller->I_q;

      if (ident->sample_count >= ident->settle_samples) {
        ident->v_sin += (int64_t)v_inj * sc.hSin;
        ident->v_cos += (int64_t)v_inj * sc.hCos;
        ident->i_sin += (int64_t)i_meas * sc.hSin;
        ident->i_cos += (int64_t)i_meas * sc.hCos;
      }

      ident->phase_accum += ident->phase_step;
    }
    // DC模式 (amplitude==0)：V_d/V_q由外部设定，ISR只做直通
    // 用于电阻辨识等恒压注入场景

    ident->sample_count++;

    if (ident->sample_count >= ident->settle_samples + ident->measure_samples) {
      ident->enable = 0;
      ident->done = 1;
    }

    limit_norm(&controller->V_d, &controller->V_q, g_vs_limit);
    set_phase_voltage(controller, controller->V_d, controller->V_q, controller->theta_elec);
    return;
  }

  set_torque_ref_loop(controller->I_q_ref);

  // 带宽测试信号注入
  int16_t sweep_signal = bw_test_run(&controller->bw_test, controller->I_q);
  /* CAN 0x2F05 cmd=1 触发的单频注入 (协议带宽测试), 与 sweep_signal 互斥使用一般不会同时开 */
  int16_t can_test_inject = can_wly_test_isr_pre();

  /* === 弱磁控制 (移到 PI 前): 用上拍 V_d/V_q 算本拍 compensation_weak ===
   * 时序好处: 圆限幅、PI-Id AimValue、BEMF FF 都用 [N] id_weak, 完全同步
   * Vs 信息仍是 [N-1] (V_d/V_q 上拍 PI 输出), 这是物理无法消除的测量延迟 */
#if USE_WEAK_MAGN
  weak_magn_control(controller);
#endif

  /* I_q_ref 流水线:
   *   step1: iq_basic = I_q_ref (速度环输出, 不含扰动)
   *   step2: 弱磁圆限幅基础指令 → anti-windup 触发速度环
   *   step3: CurrentLoopSmoothRun 斜坡平滑基础指令变化率
   *   step4: 叠加扫频 / CAN 注入信号 (避免被斜坡吃掉, 保留高频带宽)
   *   step5: 弱磁圆限幅总电流 (保护性, 不再 anti-windup)
   * 设计: sweep 在斜坡后叠加保留全频带, 但仍受 step5 圆限幅约束总电流不超 Imax */
  int32_t iq_basic = controller->I_q_ref;

  /* === step2: 基础指令圆限幅 + 速度环 anti-windup === */
#if USE_WEAK_MAGN
  int32_t iq_avail_q10 = (int32_t)controller->FlashData.MaxCurrent;
  if (controller->compensation_weak < 0) {
    int32_t id_abs   = -controller->compensation_weak;       /* Q10, ≥0 */
    int32_t imax_q10 = (int32_t)controller->FlashData.MaxCurrent;
    int64_t iq_avail_sq = (int64_t)imax_q10 * imax_q10
                        - (int64_t)id_abs   * id_abs;
    if (iq_avail_sq > 0) {
      iq_avail_q10 = (int32_t)qsqrt((uint32_t)iq_avail_sq);
      int32_t iq_before = iq_basic;
      if (iq_basic >  iq_avail_q10) iq_basic =  iq_avail_q10;
      if (iq_basic < -iq_avail_q10) iq_basic = -iq_avail_q10;
      /* 圆限幅命中 → 通知速度环 anti-windup, 防止指令撤销后超速 */
      if (iq_before != iq_basic) {
        controller->IncPID_Speed.saturated = 1;
      }
    }
  }
#endif

  /* === step3: 斜坡 (限制基础指令变化率, 不平滑扫频/CAN 注入) === */
#if USE_CURRENT_LOOP_FILTER
  int32_t iq_smooth = (controller->controller_mode == MIT_PD_MODE) ?
                      iq_basic :
                      CurrentLoopSmoothRun(iq_basic, &controller->CurrentSmooth);
#else
  int32_t iq_smooth = iq_basic;
#endif

  /* === step4: 叠加扫频 / CAN 注入信号 (保留 sweep 高频成分) === */
  controller->I_q_ref_filterd = iq_smooth + sweep_signal + can_test_inject;

  /* === step5: 总电流圆限幅 (保护性, sweep 大幅注入或参数误判时兜底) ===
   * 不触发 anti-windup: sweep 是测试扰动, 不应反作用到速度环积分 */
#if USE_WEAK_MAGN
  if (controller->compensation_weak < 0) {
    if (controller->I_q_ref_filterd >  iq_avail_q10) controller->I_q_ref_filterd =  iq_avail_q10;
    if (controller->I_q_ref_filterd < -iq_avail_q10) controller->I_q_ref_filterd = -iq_avail_q10;
  }
#endif

  /****************************辨识参数时，关闭该段，避免影响****************************************/

  /* BEMF 前馈用电角速度: 来源 = 指令侧 velocity_ref_filterd (载端 rpm×1024×25)
   * 改用指令侧而非反馈侧 dtheta_mech, 解决反向切换瞬间方向不一致问题:
   *   - dtheta_mech 经 16 拍滑动均值 + 0.7ms LPF, 反向切换滞后 1.6~2.3ms
   *   - I_q_ref_filterd 是指令侧 ramp, 反向时几拍内翻号
   *   - 两者不同步 → Vd_ff/Vq_ff 短暂方向错 → PI 必须反向抵消但被限幅
   * 指令侧 ωe 与 I_q_ref 同源 (速度环→电流环), 变向时同步翻号
   * 跟踪误差 ≤ 5% (速度环带宽 45.7Hz), 残差由 PI 收尾完全够
   * 仍保留 α=1/8 LPF, 抑制 velocity_ref 阶跃和速度环 ramp 高频纹波 */
#if USE_BEMF_FF
  static float omega_e_filt = 0.0f;
  /* 过零钳制: 电机端 |speed| < 30 rpm 时 BEMF 主项 ψ_f×ωe < 0.24V,
   * 关掉前馈让 PI 接管, 避免低速指令抖动放大 */
  const int32_t MOTOR_DEAD_RPM_Q10 = 30 * 1024;
  if (controller->ident_test.flux_psi > 0.0f) {
    /* velocity_ref_filterd 单位 = 电机端 rpm×1024 (与速度环 PID 同单位) */
    int32_t v_motor_q10 = controller->velocity_ref_filterd;
    int32_t v_abs = v_motor_q10 < 0 ? -v_motor_q10 : v_motor_q10;
    if (v_abs < MOTOR_DEAD_RPM_Q10) {
      omega_e_filt = 0.0f;
    } else {
      float omega_e_raw = (float)v_motor_q10 * controller->bemf_omega_e_k;
      omega_e_filt += (omega_e_raw - omega_e_filt) * 0.125f;
    }
  } else {
    omega_e_filt = 0.0f;
  }

  /* 动态 PI OutputMax: 矢量预算 = g_vs_limit - |Vff| - 死区预留
   * 用指令侧电流 (I_q_ref_filterd / I_d_ref) 预测当拍 Vff 模长, 含 ωe·Ld·Id 项 */
  if (controller->ident_test.flux_psi > 0.0f) {
    float iq_a = (float)controller->I_q_ref_filterd * (1.0f / 1024.0f);
    float id_a = (float)controller->I_d_ref         * (1.0f / 1024.0f);
    float Vq_ff_pred =  omega_e_filt * (controller->ident_test.Ld * id_a + controller->ident_test.flux_psi);
    float Vd_ff_pred = -omega_e_filt *  controller->ident_test.Lq * iq_a;
    float bemf_mag_q10 = sqrtf(Vd_ff_pred * Vd_ff_pred + Vq_ff_pred * Vq_ff_pred) * 1024.0f;
    int32_t pi_lim = (int32_t)g_vs_limit - (int32_t)bemf_mag_q10 - 1024;
    if (pi_lim < 4096)  pi_lim = 4096;
    if (pi_lim > 20480) pi_lim = 20480;
    controller->IncPID_QAxis.OutputMax = pi_lim;
    controller->IncPID_DAxis.OutputMax = pi_lim;
  }
#endif

  // PID-Id
  controller->IncPID_DAxis.NowValue = controller->I_d;

  #if USE_WEAK_MAGN
    //弱磁
    controller->IncPID_DAxis.AimValue = controller_eyou.compensation_weak;
  #else
    controller->IncPID_DAxis.AimValue = controller->I_d_ref;
  #endif
  controller->IncPID_DAxis.PidRun(&controller->IncPID_DAxis);
  controller->V_d = controller->IncPID_DAxis.OutPut;

  // PID-Iq
  controller->IncPID_QAxis.NowValue = controller->I_q;
  controller->IncPID_QAxis.AimValue = controller->I_q_ref_filterd;
  controller->IncPID_QAxis.PidRun(&controller->IncPID_QAxis);
  controller->V_q = controller->IncPID_QAxis.OutPut;

  /* 弱磁已在 PI 之前调用 (本拍同步), 此处不再重复调用 */

  #if USE_BEMF_FF
    if (controller->ident_test.flux_psi > 0.0f &&
        controller->ident_test.Lq > 0.0f) {
      /* 解耦项用指令侧电流, 避免反馈 PWM 纹波/ADC 噪声经 ωe·L 放大灌进 V_dq.
       * Vff 用滤波后的 omega_e_filt, 跟动态 pi_lim 共用同一份, 保证两边模型一致
       * Id 来源: 弱磁工作时用 compensation_weak, 否则用 I_d_ref (通常为 0) */
      int32_t id_for_ff = controller->I_d_ref;
      #if USE_WEAK_MAGN
        if (controller->compensation_weak < 0) id_for_ff = controller->compensation_weak;
      #endif
      float Vd_ff = -omega_e_filt * controller->ident_test.Lq * (float)controller->I_q_ref_filterd;
      float Vq_ff =  omega_e_filt * (controller->ident_test.Ld * (float)id_for_ff
                                     + controller->ident_test.flux_psi * 1024.0f);

      /* 软限幅: |Vff| ≤ 0.85·g_vs_limit, 不让前馈一项就吃光全部余量
       * 高速突发 ωe 抖动时给 PI 留至少 15% 矢量预算, 避免 limit_norm 切相位 */
      float ff_mag = sqrtf(Vd_ff * Vd_ff + Vq_ff * Vq_ff);
      float ff_max = (float)g_vs_limit * 0.85f;
      if (ff_mag > ff_max && ff_mag > 1.0f) {
        float k = ff_max / ff_mag;
        Vd_ff *= k;  Vq_ff *= k;
      }

      controller->V_d += (int32_t)Vd_ff;
      controller->V_q += (int32_t)Vq_ff;
    }
  #endif

  // uvw
  check_phases_overcurrent_timesliced(controller);

  #if USE_DEADTIME_COMPENSATION
    // 死区补偿（三相电流方向补偿，精度优于dq轴方式）
    deadtime_compensation_3phase(controller);
    #endif

  /* 超速保护: 实际速度 > OVERSPD_LOW 电机端时, 按比例削减电压输出
   * OVERSPD_LOW~HIGH: 线性从 100% 削到 0%
   * >HIGH: 输出归零
   * scale 经一阶低通滤波, 避免阈值附近电压跳变产生噪音
   * 阈值对齐: 弱磁开启时给弱磁工作区让出空间 (高 25%), 避免与弱磁削压打架 */
#if 1
  #if USE_WEAK_MAGN
    /* 弱磁开启: 阈值上抬, 允许超基速运行至 ~140rpm 输出端
     * 注意: 与弱磁削压有冗余, 主要给 OVERSPD 兜底, 让弱磁先工作 */
    #define OVERSPD_LOW   (3800 * 1024)
    #define OVERSPD_HIGH  (4000 * 1024)
  #else
    /* 不弱磁: 阈值与 DEFAULT_MAX_SPEED=110rpm 输出端对齐, 留 100rpm 裕量 */
    #define OVERSPD_LOW   (2800 * 1024)
    #define OVERSPD_HIGH  (2950 * 1024)
  #endif
  {
    static int32_t scale_filt = 1024;
    int32_t spd_abs = controller->dtheta_mech;
    if (spd_abs < 0) spd_abs = -spd_abs;

    int32_t scale;
    if (spd_abs <= OVERSPD_LOW) {
      scale = 1024;
    } else if (spd_abs >= OVERSPD_HIGH) {
      scale = 0;
    } else {
      scale = (OVERSPD_HIGH - spd_abs) * 1024 / (OVERSPD_HIGH - OVERSPD_LOW);
    }

    scale_filt += (scale - scale_filt) / 128;

    if (scale_filt < 1024) {
      controller->V_d = controller->V_d * scale_filt / 1024;
      controller->V_q = controller->V_q * scale_filt / 1024;
    }
  }
#endif

  //
  int32_t Vd_before = controller->V_d;
  int32_t Vq_before = controller->V_q;
  limit_norm(&controller->V_d, &controller->V_q, g_vs_limit);

  // anti-windup: 检测电压是否被截断，回写饱和标志给 PID
  uint8_t vs_saturated = (controller->V_d != Vd_before || controller->V_q != Vq_before) ? 1 : 0;
  controller->IncPID_QAxis.saturated = vs_saturated;
  controller->IncPID_DAxis.saturated = vs_saturated;

  set_phase_voltage(controller, controller->V_d, controller->V_q, controller->theta_elec);

  /* CAN 0x2F05 cmd=1 触发的逐拍数据回传 (0x7FD), 上传 (Iq_ref_filterd, I_q) */
  can_wly_test_isr_post(controller->I_q_ref_filterd, controller->I_q);
  /* 同步刷新 Iq 反馈 LPF, 供 CAN 状态帧上报使用 (控制环用原始 I_q) */
  can_wly_iq_fb_filter_update(controller->I_q);
  /********************************************************************/
}

/*******************************************************************************
  : deadtime_compensation
    : 死区补偿函数，基于dq轴电流方向进行补偿
    : controller - 控制器结构体指针
  :
    : 根据电流方向对Vd、Vq进行死区补偿
          补偿原理：Vcomp = sign(I) × Vdc × Td / Ts
          过零区处理：电流小于阈值时，线性过渡补偿量
********************************************************************************/
void deadtime_compensation(ControllerStruct* controller) {
#if USE_DEADTIME_COMPENSATION
    int32_t comp_d = 0;
    int32_t comp_q = 0;

    // Q轴补偿（转矩电流）
    if (controller->I_q > DEADTIME_CURRENT_THRESHOLD) {
        // 正向电流，补偿正电压
        comp_q = DEADTIME_COMP_VOLTAGE;
    } else if (controller->I_q < -DEADTIME_CURRENT_THRESHOLD) {
        // 负向电流，补偿负电压
        comp_q = -DEADTIME_COMP_VOLTAGE;
    } else {
        // 过零区：线性插值，平滑过渡
        comp_q = (controller->I_q * DEADTIME_COMP_VOLTAGE) / DEADTIME_CURRENT_THRESHOLD;
    }

    // D轴补偿（励磁电流）
    if (controller->I_d > DEADTIME_CURRENT_THRESHOLD) {
        comp_d = DEADTIME_COMP_VOLTAGE;
    } else if (controller->I_d < -DEADTIME_CURRENT_THRESHOLD) {
        comp_d = -DEADTIME_COMP_VOLTAGE;
    } else {
        // 过零区：线性插值
        comp_d = (controller->I_d * DEADTIME_COMP_VOLTAGE) / DEADTIME_CURRENT_THRESHOLD;
    }

    // 应用补偿到输出电压
    controller->V_d += comp_d;
    controller->V_q += comp_q;
#endif
}

#if USE_DEADTIME_COMPENSATION
/*******************************************************************************
  死区补偿查表 (纯理论默认值, 标定后用 bwtest10 输出替换)
  物理模型: V_dt(I) = V_base + Rds_on × |I|
    V_base = Vdc × Td/Ts + V_F × Td/Ts
           = 48V × 500ns/50µs + 0.7V × 500ns/50µs = 0.487V
    Rds_on ≈ 5 mΩ (典型 60V MOSFET)
    小电流 (<1A): 体二极管 V_F 亚阈值非线性, V_dt 比 V_base 小
    LUT 只存正半轴, 负值 sign 对称; 端点外按相邻段斜率线性外推
********************************************************************************/
typedef struct { int32_t i_q10; int32_t v_q10; } dt_pt_t;
static const dt_pt_t s_dt_lut[] = {
    {     0,    0 },   /* 0A:    0V (过零) */
    {   256,  205 },   /* 0.25A: 0.20V (二极管亚阈值) */
    {   512,  307 },   /* 0.5A:  0.30V */
    {  1024,  410 },   /* 1.0A:  0.40V (接近饱和) */
    {  3072,  512 },   /* 3.0A:  0.50V ≈ V_base */
    {  5120,  522 },   /* 5.0A:  0.51V */
    { 10240,  553 },   /* 10A:   0.54V */
    { 20480,  604 },   /* 20A:   0.59V */
    { 30720,  655 },   /* 30A:   0.64V */
    { 51200,  758 },   /* 50A:   0.74V */
};
#define DT_LUT_N (sizeof(s_dt_lut) / sizeof(s_dt_lut[0]))

static int32_t deadtime_comp_lookup(int32_t i_q10) {
    int32_t s = (i_q10 < 0) ? -1 : 1;
    int32_t a = (i_q10 < 0) ? -i_q10 : i_q10;
    int32_t v;
    if (a >= s_dt_lut[DT_LUT_N - 1].i_q10) {
        uint32_t n = DT_LUT_N - 1;
        int32_t di = s_dt_lut[n].i_q10 - s_dt_lut[n - 1].i_q10;
        int32_t dv = s_dt_lut[n].v_q10 - s_dt_lut[n - 1].v_q10;
        v = s_dt_lut[n].v_q10 + (int32_t)((int64_t)(a - s_dt_lut[n].i_q10) * dv / di);
    } else {
        uint32_t i;
        for (i = 1; i < DT_LUT_N; i++) {
            if (a <= s_dt_lut[i].i_q10) break;
        }
        int32_t di = s_dt_lut[i].i_q10 - s_dt_lut[i - 1].i_q10;
        int32_t dv = s_dt_lut[i].v_q10 - s_dt_lut[i - 1].v_q10;
        v = s_dt_lut[i - 1].v_q10 + (int32_t)((int64_t)(a - s_dt_lut[i - 1].i_q10) * dv / di);
    }
    return s * v;
}
#endif  /* USE_DEADTIME_COMPENSATION */

/*******************************************************************************
  : deadtime_compensation_3phase
    : 死区补偿，基于三相电流幅值查表 (精度优于固定值 sign 方式)
    : controller - 控制器结构体指针
  :
    : 根据三相电流查 s_dt_lut 得补偿电压，经 Clarke+Park 变换到 dq 坐标系叠加
          Ic = -(Ia+Ib)，三相平衡，直接使用两相 Clarke 变换
          标定: bwtest10 (锁转子开环扫表)
********************************************************************************/
void deadtime_compensation_3phase(ControllerStruct* controller) {
#if USE_DEADTIME_COMPENSATION
    int32_t Va_comp, Vb_comp;
    int32_t Valpha_comp, Vbeta_comp;
    int32_t Vd_comp, Vq_comp;

    Va_comp = deadtime_comp_lookup(controller->I_a);
    Vb_comp = deadtime_comp_lookup(controller->I_b);

    // Clarke变换：abc → αβ（Ic=-Ia-Ib，三相平衡）
    clarke_transf(Va_comp, Vb_comp, &Valpha_comp, &Vbeta_comp);

    // Park变换：αβ → dq
    park_transf(Valpha_comp, Vbeta_comp, &Vd_comp, &Vq_comp, controller->theta_elec);

    // 叠加补偿到输出电压
    controller->V_d += Vd_comp;
    controller->V_q += Vq_comp;
#endif
}

/*******************************************************************************
weak_magn_control - 弱磁控制 (方案 A: 电压反馈式)
controller - 控制器结构体指针
原理:
  1. 检测 Vs = √(V_d² + V_q²) 是否撞顶 (>95% g_vs_limit)
  2. 撞顶则注负 Id (PI 控制), 削弱永磁磁场, 降低反电动势, 给 PI 让出电压
  3. 余量充足时缓慢恢复 Id → 0 (退出弱磁)
  4. 退磁保护: Id ∈ [WMAG_ID_MIN_Q10, 0]
  5. 低速 (|ωe| < WMAG_OMEGA_E_MIN) 直接禁用, 避免噪声误触发
时序:
  本拍读 V_d/V_q (PI 输出, BEMF 之前) → 算出 id_weak → 下拍 PI-Id 用
  即 PI-Id 用上一拍的 id_weak (1 拍延迟, 100µs, 远小于弱磁动态)
ωe 来源:
  与 BEMF FF 一致, 都用指令侧 velocity_ref_filterd / 25 (载端→电机端 Q10),
  避免反馈侧 dtheta_mech 滞后 1.6~2.3ms 导致反向切换瞬间决策不一致
PI 实现:
  增量式 PI, delta = -(Kp×Δexcess + Ki×excess) / Div
  Kp 项响应 vs 变化率, Ki 项消除稳态偏差
注意:
  static 状态变量必须保留, 否则每拍重置失效
  与 BEMF FF 共存: BEMF 占 80% 电压圆, 弱磁在剩 15% 内动作
********************************************************************************/
void weak_magn_control(ControllerStruct* controller){
  /* 持久化状态 (函数静态变量, ISR 间保留) */
  static int32_t id_weak_q10   = 0;        /* 当前弱磁 Id 指令 Q10 */
  static int32_t Us_filt_q10   = 0;        /* Vs 一阶低通后的值 Q10 */
  static int32_t vs_excess_prev = 0;       /* 上一拍 vs_excess, 增量 PI 用 */

  /* === 1. 计算 Vs = √(V_d² + V_q²) ===
   * V_d/V_q 是 PI 输出, |V_dq| 受 PI OutputMax 限制 < 26000 Q10,
   * 平方和 < 1.4e9 < 2^31, qsqrt 输入是 uint32, 直接转换无需钳幅 */
  uint32_t vs_sq = (uint32_t)((int64_t)controller->V_d * controller->V_d
                            + (int64_t)controller->V_q * controller->V_q);
  int32_t Us_raw = (int32_t)qsqrt(vs_sq);    /* Q10 */

  /* 一阶低通 α=1/16 (~100Hz @10kHz), 抑制 PI 输出抖动触发误判 */
  Us_filt_q10 += (Us_raw - Us_filt_q10) >> 4;
  controller_eyou.Us     = (uint32_t)Us_filt_q10;
  controller_eyou.Us_raw = (uint32_t)Us_raw;

  /* === 2. 低速禁用 (|ωe| < OMEGA_E_MIN) ===
   * ωe 来源 = 指令侧 velocity_ref_filterd (电机端 rpm×1024, 与速度环 PID 同单位),
   * 避免反馈侧 dtheta_mech 滞后导致反向切换决策不一致 */
  int32_t v_motor_q10 = controller->velocity_ref_filterd;   /* 电机端 rpm×1024 */
  float omega_e_abs = (float)v_motor_q10 * controller->bemf_omega_e_k;
  if (omega_e_abs < 0.0f) omega_e_abs = -omega_e_abs;
  if (omega_e_abs < WMAG_OMEGA_E_MIN) {
    /* 低速时立即清零 Id (避免静止时残留弱磁导致额外铜耗) */
    id_weak_q10    = 0;
    vs_excess_prev = 0;
    controller_eyou.compensation_weak = 0;
    controller_eyou.voltage_error     = 0;
    return;
  }

  /* === 3. 计算 Vs 触发阈值 (95% × g_vs_limit) === */
  int32_t vs_threshold = (int32_t)g_vs_limit * WMAG_VS_TRIGGER_RATIO / 100;
  int32_t vs_excess    = Us_filt_q10 - vs_threshold;   /* >0 撞顶, <0 有余量 */
  controller_eyou.voltage_error = vs_excess;

  /* === 4. 弱磁 PI 控制 (增量式) === */
  if (vs_excess > 0) {
    /* 电压撞顶 → 加大弱磁深度 (Id 更负)
     * 增量 PI: delta = -(Kp×Δexcess + Ki×excess) / Div
     * - Kp 项 (P): 响应 excess 变化率 (高频, 抑制冲击)
     * - Ki 项 (I): 累积消除稳态偏差 (低频, 保证 Vs 稳到阈值) */
    int32_t d_excess = vs_excess - vs_excess_prev;
    int32_t delta    = -(WMAG_KP * d_excess + WMAG_KI * vs_excess) / WMAG_DIV;
    id_weak_q10     += delta;
  } else {
    /* 电压有余量 → 缓慢恢复 Id 到 0 */
    if (id_weak_q10 < -WMAG_LEAK_OUT_STEP)      id_weak_q10 += WMAG_LEAK_OUT_STEP;
    else if (id_weak_q10 > WMAG_LEAK_OUT_STEP)  id_weak_q10 -= WMAG_LEAK_OUT_STEP;
    else                                         id_weak_q10  = 0;
  }
  vs_excess_prev = vs_excess;

  /* === 5. 退磁保护限幅 [ID_MIN, 0] === */
  if (id_weak_q10 < WMAG_ID_MIN_Q10) id_weak_q10 = WMAG_ID_MIN_Q10;
  if (id_weak_q10 > 0)               id_weak_q10 = 0;

  controller_eyou.compensation_weak = id_weak_q10;
}

//滤波
float sliding_avg_filter(float *buf, uint8_t depth, uint8_t *idx, float new_val, uint8_t filter_valid_cnt) {
    buf[*idx] = new_val;    *idx = (*idx + 1) % depth;
    // 有效计数：只累加至滤波深度
    if(filter_valid_cnt < depth) {
        filter_valid_cnt++;
    }
    // 只对有效数据求和
    float sum = 0;
    for(uint8_t i=0; i<filter_valid_cnt; i++) {
        sum += buf[i];
    }
    return sum / filter_valid_cnt;
}

/*******************************************************************************
  : phase_current_sample
    :
    :
  :
    :
********************************************************************************/
uint8_t phase_current_sample(ControllerStruct* controller) {
  /* PhaseOrder 镜像：NEGATIVE 下 PWM 已交换 B/C，ADC B 通道物理对应 C 相，
     所以把 Ib_raw 解释为 I_c，再用 KCL 反推 I_b = -I_a - I_c。
     与 set_phase_voltage 的 B/C 交换对称，整体效果 = Iβ→-Iβ。 */
  if (controller->FlashData.PhaseOrder == PHASE_ORDER_POSITIVE) {
    controller->I_a = (int32_t)(controller->Ia_raw - controller->FlashData.Ia_offset) * CURRENT_TRANS_NUMERATOR /
        CURRENT_TRANS_DENOMINATOR;
    controller->I_b = (int32_t)(controller->Ib_raw - controller->FlashData.Ib_offset) * CURRENT_TRANS_NUMERATOR /
        CURRENT_TRANS_DENOMINATOR;
    controller->I_c = (int32_t)(-controller->I_b - controller->I_a);
  } else {
    controller->I_a = (int32_t)(controller->Ia_raw - controller->FlashData.Ia_offset) * CURRENT_TRANS_NUMERATOR /
        CURRENT_TRANS_DENOMINATOR;
    controller->I_c = (int32_t)(controller->Ib_raw - controller->FlashData.Ib_offset) * CURRENT_TRANS_NUMERATOR /
        CURRENT_TRANS_DENOMINATOR;
    controller->I_b = (int32_t)(-controller->I_a - controller->I_c);
  }

  // 三相过流保护数据源 (简单一阶低通滤波, α=0.1)
  controller->I_a_Filter += (controller->I_a - controller->I_a_Filter) / 10;
  controller->I_b_Filter += (controller->I_b - controller->I_b_Filter) / 10;
  controller->I_c_Filter += (controller->I_c - controller->I_c_Filter) / 10;

  return 0;
}

/*******************************************************************************
  : phase_current_sample_Check
    :
    :
  :
    :
********************************************************************************/
uint8_t phase_current_sample_Check(ControllerStruct* controller,
                                                uint16_t IaSampleValue,
                                                uint16_t IbSampleValue) {
  controller->Ia_raw = IaSampleValue;
  controller->Ib_raw = IbSampleValue;
  return 0;
}

/*******************************************************************************
  :InitCurrentShowFilter
    :
    :
  :
    :
********************************************************************************/
uint8_t InitCurrentShowFilter(ControllerStruct* controller) {
  // 1msus-----------IS620 1ms
  controller->IqShowFilter.Ts = 1000L;
  // 50msus---------IS620 50ms
  controller->IqShowFilter.Tc = 50000L;
  // ka,kb
  controller->IqShowFilter.Filter1_Init(&controller->IqShowFilter);
  return 0;
}

/*******************************************************************************
  :CurrentShowFilterGoing
    :

    :
  :
    :
********************************************************************************/
int32_t ShowFilterGoing(ControllerStruct* controller, str_FILTER1* ShowFilter) {
  ShowFilter->InPut = controller->I_q;
  ShowFilter->Filter1(ShowFilter);
  return (int32_t)ShowFilter->OutPut;    //
}

/*******************************************************************************
  : CurrentLoopSmoothInit
    : CurrentLoopSmooth* CurrentSmooth
    :
  :
    :
********************************************************************************/
void CurrentLoopSmoothInit(CurrentLoopSmooth* CurrentSmooth) {
  CurrentSmooth->MaxCurAccEveryPrd = INC_PID_SPEED_LIMIT / (CURRENT_LOOP_MIN_ACC_TIME * 10);
  CurrentSmooth->NowCurrentRef     = 0;
  CurrentSmooth->OldCurrentRef     = 0;
  return;
}

/*******************************************************************************
  :CurrentLoopSmoothRun
    : int16_t IqRef 1/1024 A
    :
  :
    :
********************************************************************************/
int32_t CurrentLoopSmoothRun(int32_t IqRef, CurrentLoopSmooth* CurrentSmooth) {
  int32_t Temp = IqRef - CurrentSmooth->NowCurrentRef;

  if (ABS(Temp) <= CurrentSmooth->MaxCurAccEveryPrd)
    CurrentSmooth->NowCurrentRef = IqRef;

  if (IqRef > CurrentSmooth->NowCurrentRef) {
    CurrentSmooth->NowCurrentRef = CurrentSmooth->MaxCurAccEveryPrd + CurrentSmooth->OldCurrentRef;
  } else if (IqRef < CurrentSmooth->NowCurrentRef) {
    CurrentSmooth->NowCurrentRef = -CurrentSmooth->MaxCurAccEveryPrd + CurrentSmooth->OldCurrentRef;
  }

  CurrentSmooth->OldCurrentRef = CurrentSmooth->NowCurrentRef;
  return CurrentSmooth->NowCurrentRef;
}
/*******************************************************************************
  :process_single_phase
    :
    :
  :
    :uvw
********************************************************************************/
void process_single_phase(SimpleOverCurrentDetector* detector, float current, int phase) {
  detector->sample_buffer[detector->sample_index] = fabsf(current);
  detector->sample_index++;

  if (detector->sample_index >= 50) {
    float peak_sum     = 0.0f;
    uint8_t peak_count = 0;

    //
    for (uint8_t i = 1; i < 49; i++) {
      if (detector->sample_buffer[i] > detector->sample_buffer[i - 1] &&
          detector->sample_buffer[i] > detector->sample_buffer[i + 1]) {
        peak_sum += detector->sample_buffer[i];
        peak_count++;
      }
    }

    // RMS
    float rms_value                                   = peak_count > 0 ? (peak_sum / peak_count) / 1.414f : 0;
    detector->rms_buffer[detector->window_count % 20] = rms_value;
    detector->window_count++;
    detector->sample_index = 0;

    //
    if (detector->window_count >= 20) {
      bool all_over_threshold = true;
      for (uint8_t i = 0; i < 20; i++) {
        if (detector->rms_buffer[i] < Threshold.UVWCurrentLimit) {
          all_over_threshold = false;
          break;
        }
      }

      //  - phase
      if (all_over_threshold) {
        detector->fault_count++;
        if (detector->fault_count >= 20) {
          // phase
          if (phase == 0) {
            controller_eyou.ServoErrFlag.Bit.PhaseUVolErr = 1;
          } else if (phase == 1) {
            controller_eyou.ServoErrFlag.Bit.PhaseVVolErr = 1;
          } else if (phase == 2) {
            controller_eyou.ServoErrFlag.Bit.PhaseWVolErr = 1;
          }
        }
      } else {
        detector->fault_count = 0;
      }
    }
  }
}
/*******************************************************************************
  :check_phases_overcurrent_timesliced
    :
    :
  :
    :uvw -
********************************************************************************/
void check_phases_overcurrent_timesliced(ControllerStruct* controller) {
  static SimpleOverCurrentDetector detectors[3] = {0};
  static uint16_t phase_selector                = 0;    // 0-U, 1-V, 2-W
  static uint32_t call_count                    = 0;

  if (call_count % 4 == 0) {
    //  phase_selector
    switch (phase_selector) {
    case 0:                                                              // U
      process_single_phase(&detectors[0], controller->I_a_Filter, 0);    // 0 U
      break;
    case 1:                                                              // V
      process_single_phase(&detectors[1], controller->I_b_Filter, 1);    // 1 V
      break;
    case 2:                                                              // W
      process_single_phase(&detectors[2], controller->I_c_Filter, 2);    // 2 W
      break;
    }

    //
    phase_selector = (phase_selector + 1) % 3;
  }
  call_count++;
}

/*******************************************************************************
 * 电流环带宽测试 - 正弦扫频 + 同步检测
 * 原理: 在 Iq_ref 上叠加单频正弦信号，通过同步检测提取响应幅值和相位，
 *       逐点扫频得到 Bode 图数据。
 * 使用: 串口发送 "logtest120" 启动，测试完成后自动打印结果。
 *******************************************************************************/

#define BW_FS ((float)FOC_FREQUENCY)  // 电流环采样率，跟随 FOC_FREQUENCY
#define BW_SETTLE_CYCLES 10    // 每个频率点前等待稳态的周期数
#define BW_MEASURE_CYCLES 20   // 每个频率点测量的周期数

void bw_test_init(CurrentLoopBWTest* test, float freq_start, float freq_end, float amplitude) {
  test->freq_start = freq_start;
  test->freq_end   = freq_end;
  test->amplitude  = amplitude;  // Q10 格式, 1024 = 1A
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
  test->phase_step = (uint32_t)(test->current_freq / BW_FS * 4294967296.0f);
  test->phase_accum = 0;

  // 每个频率点的采样数 = (settle + measure) 周期 * 每周期采样数
  float samples_per_cycle = BW_FS / test->current_freq;
  test->samples_needed = (uint32_t)(samples_per_cycle * (BW_SETTLE_CYCLES + BW_MEASURE_CYCLES));
  test->settle_samples = (uint32_t)(samples_per_cycle * BW_SETTLE_CYCLES);

  test->sum_sin = 0;
  test->sum_cos = 0;
  test->ref_sin = 0;
  test->ref_cos = 0;
  test->sample_count = 0;

  test->enable = 1;
}

/*******************************************************************************
  bw_test_run - 每个电流环周期调用一次 (10kHz)
  返回值: 叠加到 I_q_ref 上的注入信号 (Q10 格式)
*******************************************************************************/
int16_t bw_test_run(CurrentLoopBWTest* test, int32_t iq_feedback) {
  if (!test->enable) {
    // 斜坡停机阶段
    if (test->stopping) {
      // 每周期递减 1 Q10，10kHz下 2A 约需 2s 降完
      if (test->ramp_ref > 0) {
        test->ramp_ref -= 1;
        if (test->ramp_ref < 0) test->ramp_ref = 0;
      } else if (test->ramp_ref < 0) {
        test->ramp_ref += 1;
        if (test->ramp_ref > 0) test->ramp_ref = 0;
      }
      controller_eyou.I_q_ref = test->ramp_ref;
      if (test->ramp_ref == 0) {
        test->stopping = 0;
        // 不设 foc_run=0，保持 FOC 环路运行（I_q_ref=0 主动制动）
        // 避免 PWM 突然停止导致反电动势冲击母线
      }
    }
    return 0;
  }

  // 利用查表 sin/cos: phase_accum 高 16 位映射到 0~65535 (uq16)
  // get_sincos_value 输入 uq16 角度, 输出 Q15 sin/cos
  uint16_t angle = (uint16_t)(test->phase_accum >> 16);
  Trig_Components sc = get_sincos_value((int32_t)angle);
  // sc.hSin, sc.hCos 为 Q15 格式 (-32768 ~ 32767)

  // 注入信号: inject = amplitude * sin(angle), Q10 输出
  int16_t inject = (int16_t)(((int32_t)test->amplitude * sc.hSin) >> 15);

  // 稳态等待阶段不累加，只在测量阶段累加
  if (test->sample_count >= test->settle_samples) {
    // 同步检测: 累加 response * sin/cos 和 reference * sin/cos
    test->sum_sin += (int64_t)iq_feedback * sc.hSin;
    test->sum_cos += (int64_t)iq_feedback * sc.hCos;
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
      // 启动斜坡停机，从当前 I_q_ref 缓慢降到 0
      test->stopping = 1;
      test->ramp_ref = controller_eyou.I_q_ref;
      return 0;
    }

    // 对数扫频
    test->current_freq = test->freq_start *
        powf(10.0f, (float)test->current_point / test->points_per_decade);
    // 限制不超过 Nyquist
    if (test->current_freq > BW_FS / 2.0f)
      test->current_freq = BW_FS / 2.0f;

    test->phase_step = (uint32_t)(test->current_freq / BW_FS * 4294967296.0f);
    test->phase_accum = 0;

    float spc = BW_FS / test->current_freq;
    test->samples_needed = (uint32_t)(spc * (BW_SETTLE_CYCLES + BW_MEASURE_CYCLES));
    test->settle_samples = (uint32_t)(spc * BW_SETTLE_CYCLES);

    test->sum_sin = 0;
    test->sum_cos = 0;
    test->ref_sin = 0;
    test->ref_cos = 0;
    test->sample_count = 0;
  }

  return inject;
}

/*******************************************************************************
  bw_test_print_results - 通过串口打印 Bode 图数据
*******************************************************************************/
void bw_test_print_results(CurrentLoopBWTest* test) {
  printf("\r\n===== Current Loop Bandwidth Test Results =====\r\n");
  printf("Kp=%d Ki=%d Kd=%d PID_Div=%u Iq_ref=%d Iq_inject=%.0f TargetBW=%dHz\r\n",
         (int)controller_eyou.IncPID_QAxis.P,
         (int)controller_eyou.IncPID_QAxis.I,
         (int)controller_eyou.IncPID_QAxis.D,
         (unsigned)controller_eyou.IncPID_QAxis.PID_Div,
         (int)controller_eyou.I_q_ref,
         test->amplitude,
         CURRENT_LOOP_TARGET_BW_HZ);
  printf("Freq(Hz)\tGain(dB)\tPhase(deg)\r\n");
  for (uint16_t i = 0; i < test->current_point; i++) {
    printf("%.1f\t\t%.2f\t\t%.1f\r\n",
           test->freq_list[i],
           test->gain_db[i],
           test->phase_deg[i]);
  }

  /* ---- 分析各项指标 ---- */

  /* 1. 找谐振峰 */
  float gain_low = test->gain_db[0];
  float gain_peak = gain_low;
  uint16_t peak_idx = 0;
  for (uint16_t i = 0; i < test->current_point; i++) {
    if (test->gain_db[i] > gain_peak) {
      gain_peak = test->gain_db[i];
      peak_idx = i;
    }
  }
  uint8_t has_peak = (gain_peak - gain_low > 1.0f);

  /* 2. -3dB 带宽（有峰从峰值算，无峰从低频基准算） */
  float bw = 0;
  float bw_target = has_peak ? (gain_peak - 3.0f) : (gain_low - 3.0f);
  uint16_t bw_start = has_peak ? (peak_idx + 1) : 1;
  for (uint16_t i = bw_start; i < test->current_point; i++) {
    if (test->gain_db[i] < bw_target) {
      float f0 = test->freq_list[i - 1];
      float f1 = test->freq_list[i];
      float g0 = test->gain_db[i - 1];
      float g1 = test->gain_db[i];
      bw = f0 + (f1 - f0) * (bw_target - g0) / (g1 - g0);
      break;
    }
  }

  /* 3. 0dB 穿越频率（增益从负穿到正） */
  float f_0dB = 0;
  for (uint16_t i = 1; i < test->current_point; i++) {
    if (test->gain_db[i - 1] < 0.0f && test->gain_db[i] >= 0.0f) {
      float f0 = test->freq_list[i - 1];
      float f1 = test->freq_list[i];
      float g0 = test->gain_db[i - 1];
      float g1 = test->gain_db[i];
      f_0dB = f0 + (f1 - f0) * (0.0f - g0) / (g1 - g0);
      break;
    }
  }

  /* 4. 阻尼比（从峰值反推，Mp = 1 / (2ζ√(1-ζ²))，Mp > 1 时近似 ζ ≈ 1/(2Mp)） */
  float zeta = 0.707f;  /* 无峰默认值 */
  if (has_peak) {
    float Mp = powf(10.0f, gain_peak / 20.0f);
    if (Mp > 1.0f) {
      /* 精确解 ζ = √((1 - √(1 - 1/Mp²)) / 2) */
      float inside = 1.0f - 1.0f / (Mp * Mp);
      if (inside > 0.0f) {
        zeta = sqrtf((1.0f - sqrtf(inside)) * 0.5f);
      } else {
        zeta = 0.5f / Mp;  /* fallback */
      }
    }
  }

  /* 5. 相位裕度（经验公式 PM ≈ 100 × ζ，ζ < 0.7 时有效） */
  float pm = 100.0f * zeta;
  if (pm > 70.0f) pm = 70.0f;

  /* 6. 阶跃超调（%） */
  float overshoot = 0.0f;
  if (zeta < 1.0f) {
    float denom = sqrtf(1.0f - zeta * zeta);
    if (denom > 0.01f) {
      overshoot = expf(-3.14159f * zeta / denom) * 100.0f;
    }
  }

  /* ---- 打印汇总 ---- */
  printf("\r\n---- Performance Summary ----\r\n");
  if (has_peak) {
    printf("Resonance peak:     %6.2f dB @ %6.1f Hz  [recommend <3dB]\r\n",
           gain_peak, test->freq_list[peak_idx]);
    if (bw > 0) {
      printf("-3dB Bandwidth:    %7.1f Hz (from peak)  [recommend fs/10~fs/5 = 1000~2000Hz]\r\n", bw);
    } else {
      printf("-3dB Bandwidth:      >%.1f Hz (beyond test range)  [recommend fs/10~fs/5]\r\n",
             test->freq_list[test->current_point - 1]);
    }
  } else {
    printf("Low-freq gain:      %6.2f dB (flat, no resonance)\r\n", gain_low);
    if (bw > 0) {
      printf("-3dB Bandwidth:    %7.1f Hz  [recommend fs/10~fs/5 = 1000~2000Hz]\r\n", bw);
    } else {
      printf("-3dB Bandwidth:      >%.1f Hz (beyond test range)  [recommend fs/10~fs/5]\r\n",
             test->freq_list[test->current_point - 1]);
    }
  }
  if (f_0dB > 0) {
    printf("0dB crossover:     %7.1f Hz  [closed-loop BW ~ 1.3x this]\r\n", f_0dB);
  }
  printf("Damping ratio:      %6.2f       [recommend 0.4~0.7]\r\n", zeta);
  printf("Phase margin (est): %5.0f deg     [recommend 45~60 deg, >30 min]\r\n", pm);
  printf("Overshoot (est):    %5.0f %%       [recommend 5~25%%]\r\n", overshoot);
  printf("\r\nReferences: Krishnan \"Electric Motor Drives\", Ogata \"Modern Control\"\r\n");
  printf("================================================\r\n");
}

/*******************************************************************************
 * 电感辨识 - ISR同步测量 (跟随bw_test模式)
 * 原理: 在FOC中断内注入单频正弦电压，旁路PI，同步检测提取电流响应，
 *       计算阻抗后扣除Rs得到电感值。
 *******************************************************************************/

#define IDENT_SETTLE_CYCLES  10   // 每次测量前稳态等待周期数
#define IDENT_MEASURE_CYCLES 20   // 每次测量的周期数
#define IDENT_FS ((float)FOC_FREQUENCY)

void ident_inductance_init(InductanceIdent* ident, uint8_t axis, float freq, float amplitude, float Rs) {
  ident->axis      = axis;
  ident->inj_freq  = freq;
  ident->amplitude = amplitude;
  ident->Rs        = Rs;

  // 相位步进: phase_step = freq / fs * 2^32
  ident->phase_step  = (uint32_t)(freq / IDENT_FS * 4294967296.0f);
  ident->phase_accum = 0;

  // 采样数 = 周期数 × 每周期采样数
  float spc = IDENT_FS / freq;
  ident->settle_samples  = (uint32_t)(spc * IDENT_SETTLE_CYCLES);
  ident->measure_samples = (uint32_t)(spc * IDENT_MEASURE_CYCLES);

  ident->v_sin = 0;  ident->v_cos = 0;
  ident->i_sin = 0;  ident->i_cos = 0;
  ident->sample_count = 0;

  ident->done   = 0;
  ident->enable = 1;
}

void ident_inductance_compute(InductanceIdent* ident) {
  // 电压幅值 (同步检测)
  double v_amp = sqrt((double)ident->v_sin * ident->v_sin +
                      (double)ident->v_cos * ident->v_cos);
  // 电流幅值
  double i_amp = sqrt((double)ident->i_sin * ident->i_sin +
                      (double)ident->i_cos * ident->i_cos);

  float L = 0.0f;
  if (i_amp > 0 && v_amp > 0) {
    // 阻抗幅值: Z = V_amp / I_amp (Q10电压 / Q10电流 = 实际Ohm)
    float Z = (float)(v_amp / i_amp);
    float omega = 2.0f * M_PIf * ident->inj_freq;

    if (ident->Rs > 0.0f && Z > ident->Rs) {
      // Rs补偿: L = sqrt(Z^2 - Rs^2) / omega
      float X = sqrtf(Z * Z - ident->Rs * ident->Rs);
      L = X / omega;
    } else {
      // 无Rs补偿 (fallback)
      L = Z / omega;
    }
  }

  if (ident->axis == 0)
    ident->Ld = L;
  else
    ident->Lq = L;
}
