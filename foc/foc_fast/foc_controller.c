/**
 * @file    foc_controller.h
 * @brief
 * @author  yxsui
 * @date    2025-08-01
 * @version 1.1
 */

#include "foc_controller.h"

uint32_t MAX_CURRENT_PRE        = 0;
uint32_t DEFAULT_MAX_SPEED      = 0;
uint32_t INC_PID_POSITION_LIMIT = 0;
// 电机PID参数
uint32_t INC_PID_POSITION_KP = 0;
uint32_t INC_PID_POSITION_KI = 0;
uint32_t INC_PID_POSITION_KD = 0;

uint32_t INC_PID_SPEED_KP = 0;
uint32_t INC_PID_SPEED_KI = 0;
uint32_t INC_PID_SPEED_KD = 0;
uint32_t POSERRFF_KP = 0;

uint32_t INC_PID_CURRENT_KP = 0;
uint32_t INC_PID_CURRENT_KI = 0;
uint32_t INC_PID_CURRENT_KD = 0;

/* Runtime-controllable USART control flag (1=off, 0=on) */
volatile uint8_t USART_CONTROL = 1; /* default off */

void set_usart_control(uint8_t val)
{
  USART_CONTROL = (val ? 1 : 0);
}

uint8_t get_usart_control(void)
{
  return (uint8_t)USART_CONTROL;
}

/**********************************************
 * *void set_ver_par(uint8_t id)
 * * set_ver_par: 根据硬件id设置电机参数
 * *@Input: 硬件id
 * *#Output：None
 * ***************************************/
void set_ver_par(uint8_t id) {
  if (id == 90) {
    // motor_h7_0426 配套：pole_pairs=8，25:1减速，初始保守PID，后续再调
    NPP               = 8;
    DEFAULT_MAX_SPEED = 100 * 25 * 1024;       // 40rpm * 减速比25 * Q10

    INC_PID_POSITION_KP = 800;
    INC_PID_POSITION_KI = 5;
    INC_PID_POSITION_KD = 0;
    INC_PID_SPEED_KP    = 500;
    INC_PID_SPEED_KI    = 4;
    INC_PID_SPEED_KD    = 0;
    POSERRFF_KP         = 300;
    INC_PID_CURRENT_KP  = 55;                // 保守起步
    INC_PID_CURRENT_KI  = 3;
    INC_PID_CURRENT_KD  = 0;
  }
  INC_PID_POSITION_LIMIT = DEFAULT_MAX_SPEED;
}

FlashSavedData flash_data = {
    .StructVersion   = FLASH_STRUCT_VERSION,
    .CurrentFlag     = 0,
    .Ia_offset       = 0,
    .Ib_offset       = 0,
    .Ic_offset       = 0,
    .AngleOffsetFlag = 0,
    .elec_offset     = 0,
    .PhaseOrder      = PHASE_ORDER_POSITIVE,
    .mech_offest     = 0,
    .PidFlag         = 0,
    //.Position_Kp     = INC_PID_POSITION_KP,
    //.Position_Ki     = INC_PID_POSITION_KI,
    //.Position_Kd     = INC_PID_POSITION_KD,
    //.Pid_PositionLimit = INC_PID_POSITION_LIMIT,
    //.Speed_Kp            = INC_PID_SPEED_KP,
    //.Speed_Ki            = INC_PID_SPEED_KI,
    //.Speed_Kd            = INC_PID_SPEED_KD,
    .Pid_SpeedLimit = INC_PID_SPEED_LIMIT,
    //.PosErrFF_Kp = 0, /* no feedforward by default */
    //.Current_Kp          = INC_PID_CURRENT_KP,
    //.Current_Ki          = INC_PID_CURRENT_KI,
    //.Current_Kd          = INC_PID_CURRENT_KD,
    .Pid_CurrentLimit     = INC_PID_CURRENT_LIMIT,
    .ArrivedFlag          = 0,
    .PositionArrivedValue = POSITION_ARRIVED_RANGE,
    .SpeedArrivedValue    = SPEED_ARRIVED_RANGE,
    .CurrentArrivedValue  = CURRENT_ARRIVED_RANGE,
    .RunDataFlag          = 0,
    .RunMode              = DEFAULT_RUN_MODE,
    //.MaxSpeed = DEFAULT_MAX_SPEED,
    .MaxCurrent            = DEFAULT_MAX_CURRENT,
    .MaxPositionLimit      = 0,
    .MinPositionLimit      = 0,
    .ProteckKeyFlag        = 0,
    .BusVolProteckKey      = DEFAULT_BUS_VOL_PROTECT_KEY,
    .LockedRotorProtectKey = DEFAULT_LOCKED_MOTOR_PROTECT_KEY,
    .brake_time            = BRAKE_TIME,
    .mech_offest_out       = 0,
};

/*******************************************************************************
  :controller_init
    :

    :
  :
    : ControllerStruct
********************************************************************************/
void controller_init(ControllerStruct* controller) {
  controller->PositionRefFilter  = Filter1_Defaults;
  controller->PositionShowFilter = Filter1_Defaults;
  controller->SpeedShowFilter    = Filter1_Defaults;
  controller->IqShowFilter       = Filter1_Defaults;
  flash_data.Pid_PositionLimit   = INC_PID_POSITION_LIMIT;
  flash_data.MaxSpeed            = DEFAULT_MAX_SPEED;
  controller->FlashData          = flash_data;
}

/*******************************************************************************
  :set_phase_voltage
    :

    :
  :
    : pwm ccr
********************************************************************************/
void set_phase_voltage(ControllerStruct* controller, int32_t d, int32_t q, int32_t Theta) {
  int32_t valpha, vbeta = 0;
  uint32_t ccr2, ccr3, ccr4 = 0;

  rev_park(d, q, Theta, &valpha, &vbeta);
  svpwm_calc(valpha, vbeta, &ccr2, &ccr3, &ccr4);
  controller->CCR2 = ccr2;
  controller->CCR3 = ccr3;
  controller->CCR4 = ccr4;

  /* PhaseOrder 镜像：NEGATIVE 下交换 B/C 相 PWM (CCR3↔CCR4)，等价于 Vβ→-Vβ。
     与 phase_current_sample 的 Ib_raw 当作 I_c 解释保持对称，整体效果 = 电角度取负。
     注意：A/B 交换不等价于 Vβ 翻转（αβ 平面会混合变换），必须用 B/C 交换。 */
  if (controller->FlashData.PhaseOrder == PHASE_ORDER_POSITIVE) {
    pwm_ccr_set(controller->CCR2, controller->CCR3, controller->CCR4);
  } else {
    pwm_ccr_set(controller->CCR2, controller->CCR4, controller->CCR3);
  }
}
