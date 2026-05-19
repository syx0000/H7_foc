/**
  **************************************************************************
  * @file     foc_data.c
  * @brief    电机参数及保存相关函数
  * author    cjwang14
  * data      20251230
  *
  **************************************************************************

  **************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "foc_data.h"
// #include "cia402appl.h"  /* EtherCAT removed */
#include "func_subprogram.h"
#include "func_pid.h"
#include "foc_controller.h"
#include "foc_api.h"
#include "flash_port.h"
#include "encoder.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

Portection_Value Threshold_buffer;

extern Portection_Value Threshold;
extern ControllerStruct controller_eyou;
// extern TCiA402Axis LocalAxes[MAX_AXES];

/*******************************************************************************
  函数名: ResetControlData
  描  述: 恢复默认PMSM控制结构体参数 - 把FlashData里的PID参数塞进PID结构体
********************************************************************************/
void ResetControlData(ControllerStruct* controller) {
    controller->controller_mode = DEFAULT_RUN_MODE;
    controller->I_q_ref         = 0;

    // Id PID
    controller->IncPID_DAxis.PidInit = Init_IncPID;
    controller->IncPID_DAxis.PidInit(&controller->IncPID_DAxis,
                                     controller->FlashData.Current_Kp,
                                     controller->FlashData.Current_Ki,
                                     controller->FlashData.Current_Kd,
                                     DEFAULT_PID_DIV,
                                     controller->FlashData.Pid_CurrentLimit);
    controller->IncPID_DAxis.PidRun = IncPIDCal;

    // Iq PID
    controller->IncPID_QAxis.PidInit = Init_IncPID;
    controller->IncPID_QAxis.PidInit(&controller->IncPID_QAxis,
                                     controller->FlashData.Current_Kp,
                                     controller->FlashData.Current_Ki,
                                     controller->FlashData.Current_Kd,
                                     DEFAULT_PID_DIV,
                                     controller->FlashData.Pid_CurrentLimit);
    controller->IncPID_QAxis.PidRun = IncPIDCal;

    // Speed PID
    controller->IncPID_Speed.PidInit = Init_IncPID;
    controller->IncPID_Speed.PidInit(&controller->IncPID_Speed,
                                     controller->FlashData.Speed_Kp,
                                     controller->FlashData.Speed_Ki,
                                     controller->FlashData.Speed_Kd,
                                     DEFAULT_PID_SPEED_DIV,
                                     controller->FlashData.Pid_SpeedLimit);
    controller->IncPID_Speed.PidRun = IncPIDCal;

    // 位置误差前馈增益
    controller->pos_err_ff_gain = controller->FlashData.PosErrFF_Kp;

    // Position PID
    controller->IncPID_Position.PidInit = Init_IncPID;
    controller->IncPID_Position.PidInit(&controller->IncPID_Position,
                                        controller->FlashData.Position_Kp,
                                        controller->FlashData.Position_Ki,
                                        controller->FlashData.Position_Kd,
                                        DEFAULT_PID_POSITION_DIV,
                                        controller->FlashData.Pid_PositionLimit);
    controller->IncPID_Position.PidRun = PositionPID;

    extern uint8_t NPP;
    controller->bemf_omega_e_k = (float)NPP * 2.0f * 3.14159265f / (1024.0f * 60.0f);

    #if USE_SPEED_NOTCH
    biquadFilterInitNotch(&controller->speed_notch,
                          SPEED_NOTCH_PERIOD_US, SPEED_NOTCH_FREQ_HZ, SPEED_NOTCH_BW_HZ);
    #endif
}

/*******************************************************************************
  函数名: InitFlashData
  描  述: 上电初始化FLASH中的运行所需数据（对齐PHU实现）
          按 Flag 分阶段处理：Current/AngleOffset/Pid/Arrived/RunData/ProteckKey/brake_time/sto
          首次上电或版本不匹配时，各段用对应的 Defualt* 函数填充默认值
          改动过的段累计到 Temp，最后写回 Flash
********************************************************************************/
uint8_t InitFlashData(ControllerStruct* controller) {
    uint8_t Temp = 0;

    /* 1. 先从Flash读 */
    ReadDataFromAddress(controller, MOTORID0_RUN_DATA_ADDRESS);

    /* 2. 版本检查：不匹配则强制重新初始化所有段 */
    if (controller->FlashData.StructVersion != FLASH_STRUCT_VERSION) {
        printf("Flash struct version mismatch (got %u, expect %u), force reinit\r\n",
               (unsigned)controller->FlashData.StructVersion, FLASH_STRUCT_VERSION);
        controller->FlashData.StructVersion   = FLASH_STRUCT_VERSION;
        controller->FlashData.CurrentFlag     = 0xFFFF;
        controller->FlashData.AngleOffsetFlag = 0xFFFF;
        controller->FlashData.PidFlag         = 0xFFFF;
        controller->FlashData.ArrivedFlag     = 0xFFFF;
        controller->FlashData.RunDataFlag     = 0xFFFF;
        controller->FlashData.ProteckKeyFlag  = 0xFFFF;
        controller->FlashData.MotorParamFlag  = 0xFFFF;
        controller->FlashData.FluxIdentFlag   = 0xFFFF;
        controller->FlashData.InertiaIdentFlag = 0xFFFF;
        InitReservedFields(&controller->FlashData);
        Temp = 0xFF;
    }

    printf("Flash: PhaseOrder=%u, mech_offest_out=%d, elec_offset=%u\r\n",
           (unsigned)controller->FlashData.PhaseOrder,
           (int)controller->FlashData.mech_offest_out,
           (unsigned)controller->FlashData.elec_offset);

    /* 3. 电流零偏：未校准或为空时重新获取 */
    if ((controller->FlashData.CurrentFlag == 0xFFFF) ||
        (controller->FlashData.CurrentFlag == 0x0000)) {
        controller->FlashData.CurrentFlag = OFFEST_IS_CORRECTED_FLAG;
        PhaseCurrentOffsetEstimate(controller);
        Temp++;
    }

    /* 4. 电角度偏移：失败或首次上电时做一次电角度辨识（需要无抱闸+电机能转动） */
    if ((controller->FlashData.AngleOffsetFlag == 0xFFFF) ||
        (controller->FlashData.AngleOffsetFlag == 0x0000) ||
        (controller->FlashData.AngleOffsetFlag == ELEC_ANGLE_ESTIMATE_FAILED))
		{
        controller->FlashData.AngleOffsetFlag = OFFEST_IS_CORRECTED_FLAG;
        ElecAngleEstimate(controller);

        /* 辨识失败则返回 */
        if (controller->ServoErrFlag.Bit.LockedRotorErr ||
            controller->ServoErrFlag.Bit.PhaseOrderErr) {
            return 0xFF;
        }
        Temp++;
    }
//		else if (controller->FlashData.AngleOffsetFlag == MECH_OFFSET_ANGLE_IS_UPDATA_FLAG) {
//        /* Flash中已有用户定义的零点位置，不做处理 */
//    } else {
//        /* 初次定位后再次上电，以当前位置为零点 */
//        controller->FlashData.AngleOffsetFlag = FLASH_DATA_IS_UPDATA_FLAG;
//        MechAngleOffsetEstimata(controller, 0);
//        Temp++;
//    }

    /* 5. PID 参数：每次上电都按 set_ver_par 设置的全局变量填充（允许改源码后覆盖旧Flash） */
		//if((controller->FlashData.PidFlag == 0xFFFF) || (controller->FlashData.PidFlag == 0x0000))
		{
				controller->FlashData.PidFlag = OFFEST_IS_CORRECTED_FLAG;
				DefualtPidValue(&controller->FlashData);
				Temp++;
		}
    /* 6. 指令到达阈值 */
    if ((controller->FlashData.ArrivedFlag == 0xFFFF) ||
        (controller->FlashData.ArrivedFlag == 0x0000)) {
        controller->FlashData.ArrivedFlag = OFFEST_IS_CORRECTED_FLAG;
        DefualtArrivedValue(controller);
        Temp++;
    }

    /* 7. 运行限幅（位置/速度/电流/限位） */
    if ((controller->FlashData.RunDataFlag == 0xFFFF) ||
        (controller->FlashData.RunDataFlag == 0x0000)) {
        controller->FlashData.RunDataFlag = OFFEST_IS_CORRECTED_FLAG;
        DefualtRunDataValue(controller);
        Temp++;
    }
    /* MaxCurrent 每次上电从源码强制覆盖, 允许改宏后立即生效 (不依赖 Flash version) */
    controller->FlashData.MaxCurrent = DEFAULT_MAX_CURRENT;

    /* 8. 保护功能开关 */
    if ((controller->FlashData.ProteckKeyFlag == 0xFFFF) ||
        (controller->FlashData.ProteckKeyFlag == 0x0000)) {
        controller->FlashData.ProteckKeyFlag = OFFEST_IS_CORRECTED_FLAG;
        DefualtProteckKeyValue(controller);
        Temp++;
    }

    /* 9. 抱闸时间等对象字典字段 */
    if ((controller->FlashData.brake_time == 0xFFFF) ||
        (controller->FlashData.brake_time == 0x0000)) {
        DefualtObjectToFlash(controller);
        Temp++;
    }

    /* 10. STO 状态标志 */
    if (controller->FlashData.stoStateFlag == 0xFFFFFFFF) {
        controller->FlashData.stoStateFlag = 0;
        Temp++;
    }

    /* 11. 如有改动则写回 Flash */
    if (Temp) {
        WriteRunDataToFlash(controller, MOTORID0_RUN_DATA_ADDRESS);
    }

    printf("Flash: MaxPosLim=%d, MinPosLim=%d\r\n",
           (int)controller->FlashData.MaxPositionLimit,
           (int)controller->FlashData.MinPositionLimit);

    MAX_CURRENT_PRE = controller->FlashData.MaxCurrent;

    printf("FlashData: CurPID=%u/%u/%u SpdPID=%u/%u/%u PosPID=%u/%u/%u FF=%u\r\n",
           (unsigned)controller->FlashData.Current_Kp,
           (unsigned)controller->FlashData.Current_Ki,
           (unsigned)controller->FlashData.Current_Kd,
           (unsigned)controller->FlashData.Speed_Kp,
           (unsigned)controller->FlashData.Speed_Ki,
           (unsigned)controller->FlashData.Speed_Kd,
           (unsigned)controller->FlashData.Position_Kp,
           (unsigned)controller->FlashData.Position_Ki,
           (unsigned)controller->FlashData.Position_Kd,
           (unsigned)controller->FlashData.PosErrFF_Kp);
    return 0;
}

uint8_t PhaseCurrentOffsetEstimate(ControllerStruct* controller) {
    /* STM32 实现：直接用 g_adc_offset_a/b（已在 ADC_CalibrateOffsets 中采得）
       这里保留函数形参，不再做硬件采样，避免与 ADC 校准重复 */
    extern volatile int32_t g_adc_offset_a;
    extern volatile int32_t g_adc_offset_b;
    controller->FlashData.Ia_offset = (uint16_t)g_adc_offset_a;
    controller->FlashData.Ib_offset = (uint16_t)g_adc_offset_b;
    controller->FlashData.Ic_offset = 0;
    return 0;
}

void get_offest(uint16_t* offset_1, uint16_t* offset_2) {
    extern volatile int32_t g_adc_offset_a;
    extern volatile int32_t g_adc_offset_b;
    *offset_1 = (uint16_t)g_adc_offset_b;
    *offset_2 = (uint16_t)g_adc_offset_a;
}

uint8_t DefualtPidValue(FlashSavedData* FlashData) {
    FlashData->Position_Kp       = INC_PID_POSITION_KP;
    FlashData->Position_Ki       = INC_PID_POSITION_KI;
    FlashData->Position_Kd       = INC_PID_POSITION_KD;
    FlashData->Pid_PositionLimit = INC_PID_POSITION_LIMIT;

    FlashData->Speed_Kp       = INC_PID_SPEED_KP;
    FlashData->Speed_Ki       = INC_PID_SPEED_KI;
    FlashData->Speed_Kd       = INC_PID_SPEED_KD;
    FlashData->PosErrFF_Kp    = POSERRFF_KP;
    FlashData->Pid_SpeedLimit = INC_PID_SPEED_LIMIT;

    FlashData->Current_Kp       = INC_PID_CURRENT_KP;
    FlashData->Current_Ki       = INC_PID_CURRENT_KI;
    FlashData->Current_Kd       = INC_PID_CURRENT_KD;
    FlashData->Pid_CurrentLimit = INC_PID_CURRENT_LIMIT;
    return 0;
}

uint8_t ReadDataFromAddress(ControllerStruct* controller, unsigned int Address) {
    Flash_ReadData(Address, &controller->FlashData, sizeof(FlashSavedData));
    return 0;
}

uint8_t WriteRunDataToFlash(ControllerStruct* controller, unsigned int Address) {
    /* H743整扇区擦除，所以本扇区内其他数据会一起丢 —— 参数写回时要保证此扇区只存FlashData */
    if (Flash_EraseSector() != HAL_OK) {
        printf("Flash erase failed\r\n");
        return 1;
    }
    if (Flash_WriteData(Address, &controller->FlashData, sizeof(FlashSavedData)) != HAL_OK) {
        printf("Flash write failed\r\n");
        return 1;
    }
    printf("Flash write OK @0x%08X, %u bytes\r\n",
           (unsigned)Address, (unsigned)sizeof(FlashSavedData));
    return 0;
}

void WriteDataToFlash(void) {
    extern ControllerStruct controller_eyou;
    WriteRunDataToFlash(&controller_eyou, MOTORID0_RUN_DATA_ADDRESS);
}

uint8_t WriteFaultThreshold(Portection_Value* Threshold, uint32_t Address) {
    return 0;
}

uint8_t ReadFaultThreshold(Portection_Value* Threshold_buffer, uint32_t Address) {
    return 0;
}

uint8_t FlashLimit_Check(Portection_Value* Threshold_buffer) {
    return 0;
}

/* PHU 风格四点开环定位（只校准一个方向）：
   1. 先用 POSITIVE 模式跑四点（0°/90°/180°/270° 电角度），判断编码器方向 → 定 PhaseOrder
   2. 用确定后的 PhaseOrder 再跑一次四点，取第一个点的编码器值算 elec_offset
   这样 elec_offset 和运行时的 PhaseOrder 严格对应，不会有坐标系不一致的问题 */
void ElecAngleEstimate(ControllerStruct* controller) {
    uint16_t theta_open[4] = {0, 16383, 32767, 49151};  // 0° 90° 180° 270° 电角度
    int16_t v_d            = 1024;                       // 1V d轴电压
    int16_t v_q            = 0;
    uint32_t TempPosition[4] = {0};
    int16_t i;
    uint8_t dir = 0;
    uint8_t ErrTemp = 0;

    /* === 第一轮：用 POSITIVE 模式跑四点，判断编码器方向 === */
    controller->FlashData.PhaseOrder = PHASE_ORDER_POSITIVE;
    for (i = 0; i < 4; i++) {
        set_phase_voltage(controller, v_d, v_q, theta_open[i]);
        HAL_Delay(1200);

        DPT_Angles angles;
        DPT_GetLatestAngles(&angles);
        TempPosition[i] = angles.outer_raw;
        printf("scan[%d]: %u (%.2f deg)\r\n", i, (unsigned)TempPosition[i],
               TempPosition[i] * 360.0f / ENCODER_BIT);
    }
    set_phase_voltage(controller, 0, 0, 0);

    /* 检查方向：相邻差值归一化后 > 0 为正向 */
    dir = 0;
    for (i = 0; i < 3; i++) {
        int32_t temp = TempPosition[i + 1] - TempPosition[i];
        if (temp > (int32_t)(ENCODER_BIT / 2))       temp -= ENCODER_BIT;
        else if (temp < -(int32_t)(ENCODER_BIT / 2)) temp += ENCODER_BIT;

        printf("  delta[%d]: %.2f deg\r\n", i, temp * 360.0f / ENCODER_BIT);

        if (ABS(temp) < (ENCODER_BIT / (NPP * 4 * 2)))
            ErrTemp++;
        if (temp > 0)
            dir++;
    }

    if (ErrTemp >= 1) {
        controller->ServoErrFlag.Bit.LockedRotorErr = 1;
        controller->FlashData.AngleOffsetFlag = ELEC_ANGLE_ESTIMATE_FAILED;
        printf("ElecAngle FAIL: locked rotor\r\n");
        return;
    }

    /* 多数差值 > 0 → POSITIVE，否则 NEGATIVE */
    if (dir >= 2) {
        controller->FlashData.PhaseOrder = PHASE_ORDER_POSITIVE;
        printf("PhaseOrder = POSITIVE\r\n");
    } else {
        controller->FlashData.PhaseOrder = PHASE_ORDER_NEGATIVE;
        printf("PhaseOrder = NEGATIVE (swap B/C)\r\n");
    }

    /* === 第二轮：用确定后的 PhaseOrder 跑四点，算 elec_offset === */
    ErrTemp = 0;
    for (i = 0; i < 4; i++) {
        set_phase_voltage(controller, v_d, v_q, theta_open[i]);
        HAL_Delay(1200);

        DPT_Angles angles;
        DPT_GetLatestAngles(&angles);
        TempPosition[i] = angles.outer_raw;
        printf("cali[%d]: %u (%.2f deg)\r\n", i, (unsigned)TempPosition[i],
               TempPosition[i] * 360.0f / ENCODER_BIT);
    }
    set_phase_voltage(controller, 0, 0, 0);

    /* 验证第二轮方向正确（应该都是正向） */
    for (i = 0; i < 3; i++) {
        int32_t temp = TempPosition[i + 1] - TempPosition[i];
        if (temp > (int32_t)(ENCODER_BIT / 2))       temp -= ENCODER_BIT;
        else if (temp < -(int32_t)(ENCODER_BIT / 2)) temp += ENCODER_BIT;

        if (ABS(temp) < (ENCODER_BIT / (NPP * 4 * 2)))
            ErrTemp++;
        if (temp < 0) {
            controller->ServoErrFlag.Bit.PhaseOrderErr = 1;
            controller->FlashData.AngleOffsetFlag = ELEC_ANGLE_ESTIMATE_FAILED;
            printf("ElecAngle FAIL: phase order mismatch in round 2\r\n");
            return;
        }
    }

    if (ErrTemp >= 1) {
        controller->ServoErrFlag.Bit.LockedRotorErr = 1;
        controller->FlashData.AngleOffsetFlag = ELEC_ANGLE_ESTIMATE_FAILED;
        printf("ElecAngle FAIL: locked rotor in round 2\r\n");
        return;
    }

    /* 取第一个点（theta=0°）的编码器值算 elec_offset */
    controller->FlashData.elec_offset =
        (uint16_t)(((NPP * TempPosition[0]) % ENCODER_BIT) >> ENCODER_16BIT_DIV);
    controller->FlashData.mech_offest = TempPosition[3] * 360 >> ENCODER_10BIT_DIV;

    printf("elec_offset = %u, mech_offest = %d, PhaseOrder = %u\r\n",
           (unsigned)controller->FlashData.elec_offset,
           (int)controller->FlashData.mech_offest,
           (unsigned)controller->FlashData.PhaseOrder);
}

uint8_t MechAngleOffsetEstimata(ControllerStruct* controller, int32_t UserAngle) {
    DPT_Angles angles;
    DPT_GetLatestAngles(&angles);
    uint32_t TempPosition = angles.outer_raw;

    if (UserAngle == 0)
        controller->FlashData.mech_offest = TempPosition * 360 >> ENCODER_10BIT_DIV;

    return 0;
}

uint8_t DefualtArrivedValue(ControllerStruct* controller) {
    controller->FlashData.CurrentArrivedValue  = CURRENT_ARRIVED_RANGE;
    controller->FlashData.SpeedArrivedValue    = SPEED_ARRIVED_RANGE;
    controller->FlashData.PositionArrivedValue = POSITION_ARRIVED_RANGE;
    return 0;
}

uint8_t DefualtRunDataValue(ControllerStruct* controller) {
    controller->FlashData.RunMode    = DEFAULT_RUN_MODE;
    controller->FlashData.MaxCurrent = DEFAULT_MAX_CURRENT;
    controller->FlashData.MaxSpeed   = DEFAULT_MAX_SPEED;

    controller->FlashData.PositionLimitFlag = POSITION_FLAG;
    controller->FlashData.MaxPositionLimit  = DEFAULT_MAX_POSITION;
    controller->FlashData.MinPositionLimit  = DEFAULT_MIN_POSITION;
    return 0;
}

uint8_t DefualtProteckKeyValue(ControllerStruct* controller) {
    controller->FlashData.BusVolProteckKey      = DEFAULT_BUS_VOL_PROTECT_KEY;
    controller->FlashData.Sto_1_protectKey      = DEFAULT_STO_1_PROTECT_KEY;
    controller->FlashData.Sto_2_protectKey      = DEFAULT_STO_2_PROTECT_KEY;
    controller->FlashData.LockedRotorProtectKey = DEFAULT_LOCKED_MOTOR_PROTECT_KEY;
    return 0;
}

uint8_t DefualtObjectToFlash(ControllerStruct* controller) {
    controller->FlashData.brake_time      = BRAKE_TIME;
    controller->FlashData.PhaseOrder      = PHASE_ORDER_POSITIVE;
    controller->FlashData.mech_offest_out = 0;
    return 0;
}

void InitReservedFields(FlashSavedData* FlashData) {
    FlashData->temp1 = 0;
    FlashData->temp2 = 0;
    FlashData->temp3 = 0;
    FlashData->temp4 = 0;
    FlashData->temp5 = 0;
    FlashData->temp6 = 0;
    FlashData->temp7 = 0;
    FlashData->temp8 = 0;
}

uint16_t User_Data_Save(uint16_t control) {
    return 0;
}

int8_t Contorl_motor_dir(int8_t dir) {
    return 0;
}

uint16_t set_brake_Time(uint16_t time) {
    return 0;
}

int32_t get_elecoffest_value() {
    return 0;
}

int32_t set_velocity_lim(int32_t VelLim) {
    return 0;
}

int16_t set_max_torque(int16_t maxTorruqRef) {
    return 0;
}

int16_t get_max_torque(void) {
    return 0;
}

int16_t get_actual_torque(void) {
    return 0;
}

int32_t set_min_pos_lim(int32_t MinPosLim) {
    return 0;
}

int32_t set_max_pos_lim(int32_t MaxPosLim) {
    return 0;
}

int32_t set_current_loop_kp(int32_t Kp) {
    return 0;
}

int32_t get_current_loop_kp(void) {
    return 0;
}

int32_t set_current_loop_ki(int32_t Ki) {
    return 0;
}

int32_t get_current_loop_ki(void) {
    return 0;
}

int32_t set_speed_loop_kp(int32_t Kp) {
    return 0;
}

int32_t get_speed_loop_kp(void) {
    return 0;
}

int32_t set_speed_loop_ki(int32_t Ki) {
    return 0;
}

int32_t get_speed_loop_ki(void) {
    return 0;
}

int32_t set_position_loop_kp(int32_t Kp) {
    return 0;
}

int32_t get_position_loop_kp(void) {
    return 0;
}

int32_t set_position_loop_ki(int32_t Ki) {
    return 0;
}

int32_t get_position_loop_ki(void) {
    return 0;
}

int32_t set_position_loop_kd(int32_t Kd) {
    return 0;
}

int32_t get_position_loop_kd(void) {
    return 0;
}
