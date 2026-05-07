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
#include "flash_port.h"
#include <stdio.h>
#include <string.h>

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
}

/*******************************************************************************
  函数名: InitFlashData
  描  述: 上电初始化FLASH中的运行所需数据。
          1) 先从Flash读取，若版本匹配则采用Flash值
          2) 版本不匹配/首次上电，用 set_ver_par 的默认值填充并写回Flash
********************************************************************************/
uint8_t InitFlashData(ControllerStruct* controller) {
    /* 1. 尝试从Flash读取 */
    FlashSavedData flash_backup;
    ReadDataFromAddress(controller, MOTORID0_RUN_DATA_ADDRESS);

    if (controller->FlashData.StructVersion == FLASH_STRUCT_VERSION) {
        printf("Flash: version OK (%u), use stored params\r\n",
               (unsigned)controller->FlashData.StructVersion);
        printf("FlashData: CurPID=%u/%u/%u  SpdPID=%u/%u/%u  PosPID=%u/%u/%u  FF=%u\r\n",
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

    printf("Flash: version mismatch (got 0x%08X, expect %u), re-init\r\n",
           (unsigned)controller->FlashData.StructVersion, FLASH_STRUCT_VERSION);

    /* 2. 用 set_ver_par 设置的全局默认值填充 FlashData */
    (void)flash_backup;
    controller->FlashData.StructVersion = FLASH_STRUCT_VERSION;
    controller->FlashData.Current_Kp = INC_PID_CURRENT_KP;
    controller->FlashData.Current_Ki = INC_PID_CURRENT_KI;
    controller->FlashData.Current_Kd = INC_PID_CURRENT_KD;
    controller->FlashData.Speed_Kp   = INC_PID_SPEED_KP;
    controller->FlashData.Speed_Ki   = INC_PID_SPEED_KI;
    controller->FlashData.Speed_Kd   = INC_PID_SPEED_KD;
    controller->FlashData.Position_Kp = INC_PID_POSITION_KP;
    controller->FlashData.Position_Ki = INC_PID_POSITION_KI;
    controller->FlashData.Position_Kd = INC_PID_POSITION_KD;
    controller->FlashData.PosErrFF_Kp = POSERRFF_KP;
    controller->FlashData.Pid_PositionLimit = INC_PID_POSITION_LIMIT;
    controller->FlashData.MaxSpeed = DEFAULT_MAX_SPEED;

    /* 3. 写回Flash */
    if (WriteRunDataToFlash(controller, MOTORID0_RUN_DATA_ADDRESS) == 0) {
        printf("Flash: default params saved\r\n");
    }

    printf("FlashData: CurPID=%u/%u/%u  SpdPID=%u/%u/%u  PosPID=%u/%u/%u  FF=%u\r\n",
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
    return 0;
}

void get_offest(uint16_t* offset_1, uint16_t* offset_2) {
}

uint8_t DefualtPidValue(FlashSavedData* FlashData) {
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

void ElecAngleEstimate(ControllerStruct* controller) {
}

uint8_t MechAngleOffsetEstimata(ControllerStruct* controller, int32_t UserAngle) {
    return 0;
}

uint8_t DefualtArrivedValue(ControllerStruct* controller) {
    return 0;
}

uint8_t DefualtRunDataValue(ControllerStruct* controller) {
    return 0;
}

uint8_t DefualtProteckKeyValue(ControllerStruct* controller) {
    return 0;
}

uint8_t DefualtObjectToFlash(ControllerStruct* controller) {
    return 0;
}

void InitReservedFields(FlashSavedData* FlashData) {
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
