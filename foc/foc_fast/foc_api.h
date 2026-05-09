/**
 * @file    foc_api.h
 * @brief
 * @author  yxsui
 * @date    2025-07-31
 * @version 1.0
 */

#ifndef _FOC_API_H_
#define _FOC_API_H_

#include "encoder.h"
#include "foc_bsp.h"
#include "foc_controller.h"
#include "foc_current_loop.h"
#include "foc_position_loop.h"
#include "foc_speed_loop.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

uint8_t DefualtPidValue(FlashSavedData* FlashData);
void motorRunStateUpdate(void);
void motorRun(void);
void motorStop(void);
void Init_foc(ControllerStruct* controller);

void FocOpenTest(ControllerStruct* controller,
                 uint8_t ModelChoose,
                 int16_t v_d,
                 int16_t v_q,
                 uint16_t IaSampleValue,
                 uint16_t IbSampleValue);

typedef struct WheelControl {
  uint8_t MotorID;
  uint8_t StateCode;
  uint8_t ErrorCode;

  int32_t TargetSpeed;
  int32_t NowSpeed;

} WheelControlStr;

#define WHEEL_MOTOR_NUM MOTOR_NUM
#define WHEEL_MOTOR_LEFT MOTOR_ID0
#define WHEEL_MOTOR_RIGHT MOTOR_ID1

/* ControlWord */
#define READY_TO_SWITCH_ON 0x06
#define SWITCH_ON 0x07
#define OPERATION_ENALBE 0x0f
#define QUICK_STOP 0x02
/* StatusWord */
#define START 0x0000
#define NOT_READY_TO_SWITCH_ON 0x0250
#define SWITCH_ON_DISABLE 0x0270
#define TO_READY_TO_SWITCH_ON 0x0231
#define TO_SWITCH_ON 0x0233
#define TO_OPERATION_ENALBE 0x0237
#define QUICK_STOP_ACTIVE 0x0217
#define FALUT_REACTION_ACTIVE 0x021F
#define FAULT 0x0218

#define WHEEL_SPD_MOTOR_KP 150
#define WHEEL_SPD_MOTOR_KI 2
#define WHEEL_SPD_MOTOR_KD 10
#define WHEEL_SPD_MOTOR_DIV 100
#define WHEEL_SPD_MOTOR_LIMIT 1000

#define RPM_TO_ANGLE_RATE (float)0.00586    //   1rpm1/s(6/1024)

#define DEFAULT_WHEEL_CONTROL_LEFT {WHEEL_MOTOR_LEFT, 0, 0, 0, 0}
#define DEFAULT_WHEEL_CONTROL_RIGHT {WHEEL_MOTOR_RIGHT, 0, 0, 0, 0}

/* Private_Constants ---------------------------------------------------------*/

/* Private_TypesDefinitions --------------------------------------------------*/

/* Exported_Functions --------------------------------------------------------*/

/* Exported_Functions --------------------------------------------------------*/
extern ControllerStruct controller_eyou;
extern IncPID PidSpdWheel;    // PID

void control_to_Obj(void);
// foc-api
uint32_t getControlErrFlag(void);
uint32_t getControlStatusFlag(void);
uint8_t getControlRunFlag(void);
int8_t set_operation_mode(int8_t Mode);
int8_t get_operation_mode(void);
int32_t get_actual_position(void);
int32_t get_actual_velocity(void);
uint8_t set_velocity_ref(int32_t VelRef);
uint8_t set_velocity_ref_loop(int32_t VelRef);
int32_t get_velocity_ref(void);
int16_t set_torque_ref(int16_t TorRef);
int16_t set_torque_ref_loop(int16_t TorRef);

int32_t postion_limit_check(int32_t PostionRef);
int32_t set_postion_ref(int32_t PostionRef);

uint8_t motorPhaseCurrentStatesGet(void);
uint8_t motorZeroPointStaGet(void);
uint8_t motorSpeedOffsetGet(void);
uint8_t motorSpeedOverGet(void);
uint8_t motorPosOffsetGet(void);
uint8_t getMotorRunState(void);
uint8_t setMotorPhaseUErr(void);
uint8_t setMotorPhaseVErr(void);
uint8_t setMotorPhaseWErr(void);
uint8_t resetMotorPhaseUErr(void);
uint8_t resetMotorPhaseVErr(void);
uint8_t resetMotorPhaseWErr(void);
uint8_t getMotorPhaseUErr(void);
uint8_t getMotorPhaseVErr(void);
uint8_t getMotorPhaseWErr(void);
uint8_t zeroPointLostCheck(void);
uint32_t get_max_speed(void);
int16_t get_max_current(void);
uint8_t motorErrReset(void);
uint8_t setCommnicationErr(void);
void controlword_switch(uint16_t controlword);
float wheel_motor_spd_get(uint8_t motorId);
void wheel_motor_spd_set(uint8_t motorId, float spd);
float wheel_motor_pos_get(uint8_t motorId);
// flag_status wheel_motor_enable(uint8_t motorId, uint8_t EN);
uint32_t get_wheel_motor_status(uint8_t motorId);
uint32_t get_wheel_motor_ErrCode(uint8_t motorId);
int32_t set_error_eeprom(void);
BOOL get_motor_arrived_flag(void);
void wheel_motor_reset(uint8_t motorId);
void SetCurrentLoopPid(uint32_t Kp, uint32_t Ki, uint32_t Kd);
void SetSpeedLoopPid(uint32_t Kp, uint32_t Ki, uint32_t Kd);
void WheelControlInit(WheelControlStr* p);
uint8_t Re_Cali_Wheel_Elec_Off(uint8_t motorId);
void MotorFuncRun(ControllerStruct* controller);
void StartStopMotor(ControllerStruct* controller);
uint8_t MotorRunModeControl(ControllerStruct* controller, uint8_t AimMode);
uint8_t ModelChangeSolve(ControllerStruct* controller);
uint8_t ServoStateFlagGudge(ControllerStruct* controller);
uint8_t MC_Loop_Schedule(ControllerStruct* controller);
// static uint8_t WheelMotorReCali(ControllerStruct* controller);
uint8_t ResetAllErrorBit(ControllerStruct* controller);

uint8_t LockedRotorErrSolve(ControllerStruct* controller);
uint8_t InitFlashData(ControllerStruct* controller);

// flash

void ElecAngleEstimate(ControllerStruct* controller);
void alignDAxis(ControllerStruct* controller);
float measurePhaseResistance(ControllerStruct* controller);
void measurePhaseInductanceAC(ControllerStruct* controller, float Rs);
void autoTuneCurrentLoopPI(float Rs, float Ld, float Lq);
void autoTuneSpeedLoopPI(float J, float psi_f, uint8_t pp);
void WriteDataToFlash(void);
uint32_t Set_Position_Limit(uint32_t Limit);
uint16_t Pid_Control_Enable(uint16_t pid_key);
int32_t Get_error_speed(void);
int16_t Quick_stop_option(int16_t code);
int16_t Shutdown_code(int16_t key);
int16_t Disable_Operation(int16_t stop);
int16_t Halt_Option(int16_t option);
int16_t Fault_React(int16_t react);
int16_t Modes_Display(void);
int16_t Sensor_select(int16_t select);
uint16_t Speed_max_warn(void);
int32_t Home_Offset(void);
uint32_t Set_QuickStop_deceleration(uint32_t deceleration);
uint32_t Set_Torque_slope(uint32_t slope);
uint8_t Set_Homing_Method(uint8_t Method);
uint32_t Set_Homing_Speeds(uint32_t speed);
uint32_t Set_Homing_Speeds_zero(uint32_t speed);
uint32_t Set_Homing_Acceleration(uint32_t Acceleration);
uint32_t Set_Torque_Offset(uint32_t offset);
uint32_t Set_Maximum_acceleration(uint32_t accel);
uint32_t Set_Maximum_deceleration(uint32_t decele);
uint16_t Set_Positive_torque_limit(uint16_t limit);
uint32_t Set_Negative_torque_limit(uint32_t limit);
uint16_t Set_Position_Option_Code(uint16_t code);
uint16_t Get_Follow_offset(void);
int32_t Get_Pos_Demand(void);
uint16_t Set_Break_Dealy_Time(uint16_t BreakTime);
uint16_t get_board_temp(void);
uint32_t Get_Motor_Encoder_Value(void);
uint32_t Get_Double_Encoder_Value(void);
uint32_t Reset_objReset_Output_Encoder(uint32_t reset);
uint32_t get_position_ref(void);
int16_t Motion_profile_type(int16_t type);
int16_t Interpolatedsubmodeselect(int16_t mode);
int8_t InterpolationIndex(int8_t Index);
uint8_t Inter_polation_Period(uint8_t Period);
uint32_t Get_Supported_Drive_Modes(void);
uint16_t set_brake_Time(uint16_t time);

void fault_handle(uint8_t mod);
uint8_t communicationErrReset(void);
void get_Hardware_Version(void);
void get_Software_Version(void);
uint16_t set_STO_controll(uint16_t STO);
uint16_t get_STO_status(void);
void motorStopProgress(uint8_t mod);
uint16_t set_system_restart(uint16_t reset);
uint16_t get_Historical_Fault(uint16_t infor);
int32_t set_Minposition_Limit(int32_t Limit);
int32_t set_Maxposition_Limit(int32_t Limit);
void FlashData_recover_default(ControllerStruct* controller);
uint8_t reset_default_value(uint8_t value);

int32_t get_current_ref(void);
int16_t get_actual_current(void);
int32_t set_speed_loop_kp_ec(int32_t kp);
int32_t set_speed_loop_ki_ec(int32_t ki);
int32_t set_speed_loop_kd_ec(int32_t kd);
int32_t set_position_loop_kp_ec(int32_t kp);
int32_t set_position_loop_ki_ec(int32_t ki);
int32_t set_position_loop_kd_ec(int32_t kd);
int32_t set_current_loop_kp_ec(int32_t kp);
int32_t set_current_loop_ki_ec(int32_t ki);
int32_t set_current_loop_kd_ec(int32_t kd);
int32_t set_pos_error_ff_gain(int32_t gain);
#endif
