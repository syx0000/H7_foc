/**
 * @file    foc_current_loop.c
 * @brief
 * @author  yxsui
 * @date    2025-07-31
 * @version 1.0
 */

#include "foc_current_loop.h"
#include "foc_api.h"
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

    limit_norm(&controller->V_d, &controller->V_q, INC_PID_CURRENT_LIMIT);
    set_phase_voltage(controller, controller->V_d, controller->V_q, controller->theta_elec);
    return;
  }

  set_torque_ref_loop(controller->I_q_ref);

  // 带宽测试信号注入
  int16_t sweep_signal = bw_test_run(&controller->bw_test, controller->I_q);

  // 基础电流指令先经过斜坡滤波，扫频信号后叠加，避免被滤波器吃掉
#if USE_CURRENT_LOOP_FILTER
  controller->I_q_ref_filterd = CurrentLoopSmoothRun(controller->I_q_ref, &controller->CurrentSmooth) + sweep_signal;
#else
  controller->I_q_ref_filterd = controller->I_q_ref + sweep_signal;
#endif

  /****************************辨识参数时，关闭该段，避免影响****************************************/
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

  //弱磁
  #if USE_WEAK_MAGN
    weak_magn_control(controller);
  #endif

  // uvw
  check_phases_overcurrent_timesliced(controller);

  #if USE_DEADTIME_COMPENSATION
    // 死区补偿（三相电流方向补偿）
    deadtime_compensation(controller);//_3phase
    #endif

  //
  limit_norm(&controller->V_d, &controller->V_q, INC_PID_CURRENT_LIMIT);    //

  set_phase_voltage(controller, controller->V_d, controller->V_q, controller->theta_elec);
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

/*******************************************************************************
  : deadtime_compensation_3phase
    : 死区补偿函数，基于三相电流方向进行补偿（精度优于dq轴方式）
    : controller - 控制器结构体指针
  :
    : 根据三相电流方向计算各相补偿电压，经Clarke+Park变换到dq坐标系叠加
          补偿原理：Vx_comp = sign(Ix) × Vdc × Td / Ts
          过零区处理：电流小于阈值时，线性过渡补偿量
          Ic = -(Ia+Ib)，三相平衡，直接使用两相Clarke变换
********************************************************************************/
void deadtime_compensation_3phase(ControllerStruct* controller) {
#if USE_DEADTIME_COMPENSATION
    int32_t Va_comp, Vb_comp;
    int32_t Valpha_comp, Vbeta_comp;
    int32_t Vd_comp, Vq_comp;

    // A相补偿（根据Ia方向）
    if (controller->I_a > DEADTIME_CURRENT_THRESHOLD) {
        Va_comp = DEADTIME_COMP_VOLTAGE;
    } else if (controller->I_a < -DEADTIME_CURRENT_THRESHOLD) {
        Va_comp = -DEADTIME_COMP_VOLTAGE;
    } else {
        // 过零区：线性插值，平滑过渡
        Va_comp = (controller->I_a * DEADTIME_COMP_VOLTAGE) / DEADTIME_CURRENT_THRESHOLD;
    }

    // B相补偿（根据Ib方向）
    if (controller->I_b > DEADTIME_CURRENT_THRESHOLD) {
        Vb_comp = DEADTIME_COMP_VOLTAGE;
    } else if (controller->I_b < -DEADTIME_CURRENT_THRESHOLD) {
        Vb_comp = -DEADTIME_COMP_VOLTAGE;
    } else {
        // 过零区：线性插值，平滑过渡
        Vb_comp = (controller->I_b * DEADTIME_COMP_VOLTAGE) / DEADTIME_CURRENT_THRESHOLD;
    }

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
weak_magn_control - 弱磁控制函数
controller - 控制器结构体指针
作用 - 通过削弱d轴磁场提升电机极限转速
********************************************************************************/
void weak_magn_control(ControllerStruct* controller){
  static int32_t integral = 0;
  const uint8_t filter_depth  = 32; // 滤波深度
  float Us_buf[32]  = {0};  // Vq滤波缓存
  uint8_t filter_idx          = 0; // 缓存索引
  uint8_t filter_valid_cnt    = 0;
  int32_t pid_weakmagn;
  uint8_t Kp_weakmagn   = 3;        //弱磁PID-Kp
  uint8_t Ki_weakmagn   = 1;        //弱磁PID-Ki
  uint8_t Pid_div       = 100;      //弱磁PID-除频
  int32_t integral_max = 1000000;  //弱磁PID-积分上限
  int32_t weakmagn_max;            //弱磁深度限幅
  controller_eyou.Us_raw = sqrt( controller->V_d *  controller->V_d +  controller->V_q *  controller->V_q);
  controller_eyou.Us = sliding_avg_filter(Us_buf, filter_depth, &filter_idx, controller_eyou.Us_raw, filter_valid_cnt);
  controller_eyou.voltage_error = 5*1024 - controller_eyou.Us;
  //example:20关节速度上限设置为31rpm, 指令速度27rpm时, 27-31+5 = 1 = speed_error,此时弱磁单位为 1，弱磁深度为1*(-500)
  controller_eyou.speed_error = (controller_eyou.velocity_ref_filterd - controller_eyou.FlashData.MaxSpeed)/25/1024 + WEAK_MAGN_MARGIN;
    if(controller_eyou.voltage_error < 0){
      if(controller_eyou.speed_error >= 0){
        integral += controller_eyou.voltage_error;
        if(integral < -integral_max) //积分限幅防止溢出
          integral =  -integral_max;
        pid_weakmagn = (Kp_weakmagn * controller_eyou.voltage_error + (Ki_weakmagn * integral))/Pid_div;//Kp取3, Ki取1, 结果除以100
        weakmagn_max = WEAK_MAGN_DEPTH * controller_eyou.speed_error;
        if(pid_weakmagn < -weakmagn_max)
          pid_weakmagn = -weakmagn_max;
        if(pid_weakmagn > 0)
          pid_weakmagn = 0;
        controller_eyou.compensation_weak = pid_weakmagn;
      }
      else
        controller_eyou.compensation_weak = 0;
    }
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
  CurrentSmooth->MaxCurAccEveryPrd = INC_PID_SPEED_LIMIT / (CURRENT_LOOP_MIN_ACC_TIME * 4);
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
int16_t CurrentLoopSmoothRun(int16_t IqRef, CurrentLoopSmooth* CurrentSmooth) {
  int16_t Temp = IqRef - CurrentSmooth->NowCurrentRef;

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
  bw_test_run - 每个电流环周期调用一次 (4kHz)
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
