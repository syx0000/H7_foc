/**
 * @file    foc_controller.c
 * @brief   模块功能描述
 * @author  yxsui
 * @date    2025-08-01
 * @version 1.1
 */

#ifndef _FOC_CONTROLLER_H_
#define _FOC_CONTROLLER_H_

#include "foc_kernel.h"
#include "func_filter.h"
#include "func_pid.h"
#include <stddef.h>
#include <stdint.h>
/*******************************版本信息*************************************************/
#define SOFT_VERSION "20260331.1"
#define HARD_VERSION "20250902"
/*******************************功能配置*************************************************/
/* Runtime-controllable serial debug flag (0=off, 1=on). Replaces compile-time macro USART_CONTROL. */
extern volatile uint8_t USART_CONTROL;
#define USE_RTOS_LOG_PRINT 0       // RTOS的log打印功能
#define CHANGE_PHASE_ORDER_UV 0    // 调换U V相序；
                                   //当电机相序从UVW变为VUW时，电机的电角度相对于机械角度发生120°的相位偏移，因为每相之间的电角度差从120°变为-120°
#define USEFOC_OPEN_TEST 0         // 使用FOC开环测试
#define USE_DMA_SEND 0             // 使用dma发送
#define CIA_402_AXIS               // 使用CIA的轴宏
/*电机参数*/
#define PHU20 1
#define PHU17 0
#define PHU14 0
/*位置环指令滤波功能*/
#define USE_COMMEND_LINE_FILTER 0    //
#define USE_COMMEND_RC_FILTER 0      //
/*速度环指令滤波功能*/
#define USE_SPEED_LOOP_SMOOTH 1    // 使用速度环斜坡滤波
#define MIN_ACC_TIME 200            // 最小加减速时间,ms

/*电流环指令滤波功能*/
#define USE_CURRENT_LOOP_FILTER 1       // 使用电流环斜坡指令滤波0
#define CURRENT_LOOP_MIN_ACC_TIME 10    // 电流环滤波时间ms

//弱磁控制
#define USE_WEAK_MAGN   0     // 弱磁控制使能开关：1开启，0关闭
#define WEAK_MAGN_DEPTH 500   //弱磁深度单位
#define WEAK_MAGN_MARGIN 5      //弱磁速度超前裕度

/*死区补偿功能配置*/
#define USE_DEADTIME_COMPENSATION 1     // 死区补偿使能开关：1开启，0关闭
#define DEADTIME_TICKS 50               // 死区时间（时钟周期数）
#define PWM_CLOCK_HZ 200000000          // PWM时钟频率 200MHz
#define DEADTIME_COMP_VOLTAGE 246       // 死区补偿电压 Q10格式 (0.24V * 1024)
#define DEADTIME_CURRENT_THRESHOLD 512  // 电流过零区阈值 Q10格式 (0.5A * 1024)

/*保护功能开关*/
#define DEFAULT_BUS_VOL_PROTECT_KEY 10         // 20开  10关
#define DEFAULT_STO_1_PROTECT_KEY 10           // 20开  10关
#define DEFAULT_STO_2_PROTECT_KEY 10           // 20开  10关
#define DEFAULT_LOCKED_MOTOR_PROTECT_KEY 10    // 20开  10关

/*电机同方向旋转*/
#define MOTOR_DIRECT_SAME -1
#define MOTOR_DIRECT_SAME_F 0   //1默认参数使用宏定义，0默认参数使用flash值
/*抱闸延时时间默认值*/
#define BRAKE_TIME 500

/*地址*/
#define FLASH_USER_COMMON_ADDR  0x081E0000U   // STM32H743 Bank2 Sector 7
#define MOTORID0_RUN_DATA_ADDRESS (FLASH_USER_COMMON_ADDR)         // 0x081E0000
#define ERR_MESSGE_DATA_BASER (FLASH_USER_COMMON_ADDR + 0x1000)    // 0x081E1000
#define TEST_LIMIT_DATA (FLASH_USER_COMMON_ADDR + 0x5000)          // 0x081E5000

#define ERRMESSGECOUNT 5                                           // 历史故障记录数据组数

// #define PWM_FREQUENCY                               20000
#define FOC_FREQUENCY 10000    // 20k整数倍
#define VELOCETY_CALCULATE_DIV 2
#define POSITION_CALCULATE_DIV 4
#define SPEED_LOOP_FRE (FOC_FREQUENCY / VELOCETY_CALCULATE_DIV)
#define SPEED_FILTER_DEPTH 16
#define SPEED_UNIT 1024

#define OFFEST_IS_CORRECTED_FLAG 50
#define FLASH_DATA_IS_UPDATA_FLAG 60
#define ELEC_ANGLE_ESTIMATE_FAILED 70                          // 上一次电角度辨识失败
#define MECH_OFFSET_ANGLE_IS_UPDATA_FLAG ((uint16_t)0x0064)    // 用户定义零点
#define FLASH_STRUCT_VERSION 2                                 // FlashSavedData 结构体版本（uint32_t PID）

#define LOCKED_MOTOR_CURRENT (15 * 1024)                       // 10A
#define DE_LOCKED_CURRENT (LOCKED_MOTOR_CURRENT / 6)
#define LOCKED_MOTOR_SPEED_VALUE (30 * 1024)                   // 9rpm
#define LOCKED_MOTOR_RECOVER_TIME 10000                        // 10s

#define COMMAND_ARRIVED_Value 128
#define POSITION_ARRIVED_RANGE 256
#define SPEED_ARRIVED_RANGE 512
#define CURRENT_ARRIVED_RANGE 256

#define POSITION_ARRIVED_TIME 16    // 位置环机械臂位置到达判定
#define SPEED_ARRIVED_TIME 32       // 速度环机械臂位置到达判定
#define CURRENT_ARRIVED_TIME 32     // 电流环机械臂位置到达判定

// 高频注入配置 (用于电感辨识)
#define INJ_FREQ_HZ 2000.0f         // 注入频率（Hz）
#define INJ_VOLTAGE_AMPL 5.0f       // 注入电压幅值（V）

// 运行模式相关
#ifdef CIA_402_AXIS
#define NO_MODE 0               /**< \brief No mode*/
#define PROFILE_POSITION_MODE 1 /**< \brief Position Profile mode*/
// #define VELOCITY_MODE               2 /**< \brief Velocity mode*/
#define PROFILE_VELOCITY_MOCE 3 /**< \brief Velocity Profile mode*/
#define PROFILE_TORQUE_MODE 4   /**< \brief Torque Profile mode*/
// 5 reserved
#define HOMING_MODE 6                 /**< \brief Homing mode*/
#define INTERPOLATION_POSITION_MODE 7 /**< \brief Interpolation Position mode*/
#define CYCLIC_SYNC_POSITION_MODE 8   /**< \brief Cyclic Synchronous Position mode*/
#define CYCLIC_SYNC_VELOCITY_MODE 9   /**< \brief Cyclic Synchronous Velocity mode*/
#define CYCLIC_SYNC_TORQUE_MODE 10    /**< \brief Cyclic Synchronous Torque mode*/
#endif

#define DEFAULT_RUN_MODE NO_MODE

extern uint32_t MAX_CURRENT_PRE;
#define DEFAULT_MAX_CURRENT (25 * 1024)    // 120w/48v*38nm/14nm /sqrtf(3)*2 = 7.85A

// #define DEFAULT_MAX_SPEED                       (30 * 101*1024)   //30rpm
extern uint32_t DEFAULT_MAX_SPEED;

// #define DEFAULT_MAX_SPEED_LOAD                  (30*1024)
#define POSITION_FLAG 0
#define DEFAULT_MAX_POSITION (180 * 1024)     // 360度
#define DEFAULT_MIN_POSITION (-180 * 1024)    //-360度

// 电机PID参数
extern uint32_t INC_PID_POSITION_KP;
extern uint32_t INC_PID_POSITION_KI;
extern uint32_t INC_PID_POSITION_KD;
// #define INC_PID_POSITION_LIMIT                  DEFAULT_MAX_SPEED
extern uint32_t INC_PID_POSITION_LIMIT;
#define POS_STATE_SOFT_LIMIT 16 * 1024
#define DEFAULT_PID_POSITION_DIV 100

extern uint32_t INC_PID_SPEED_KP;
extern uint32_t INC_PID_SPEED_KI;
extern uint32_t INC_PID_SPEED_KD;
extern uint32_t POSERRFF_KP;
#define INC_PID_SPEED_LIMIT DEFAULT_MAX_CURRENT
#define DEFAULT_PID_SPEED_DIV 65000

extern uint32_t INC_PID_CURRENT_KP;
extern uint32_t INC_PID_CURRENT_KI;
extern uint32_t INC_PID_CURRENT_KD;
#define INC_PID_CURRENT_LIMIT (28467)    //(27.8*1024)
// #define VQD_LIMIT                               (27*1024)       //48v*sqrtf(3)/2 = 41.5v
#define DEFAULT_PID_DIV 100

/*******************************************************************************
 * 电感辨识结构体 (ISR同步, 跟随bw_test模式)
 *******************************************************************************/
typedef struct {
  uint8_t  enable;           // 测试使能
  uint8_t  done;             // 测试完成标志
  uint8_t  axis;             // 0=d轴, 1=q轴
  float    inj_freq;         // 注入频率 (Hz)
  float    amplitude;        // 注入电压幅值 (Q10)
  float    Rs;               // 已知相电阻用于补偿 (Ohm), 0则不补偿
  uint32_t phase_accum;      // 相位累加器
  uint32_t phase_step;       // 相位步进
  int64_t  v_sin, v_cos;     // 电压参考同步检测累加
  int64_t  i_sin, i_cos;     // 电流响应同步检测累加
  uint32_t sample_count;     // 当前已采样数
  uint32_t settle_samples;   // 稳态等待采样数
  uint32_t measure_samples;  // 测量阶段采样数
  float    Ld;               // d轴电感结果 (H)
  float    Lq;               // q轴电感结果 (H)
  float    flux_psi;         // 磁链 ψ_f [Wb], 由 TestFluxIdent 写入
} InductanceIdent;

/*******************************************************************************
 * 电流环带宽测试结构体
 *******************************************************************************/
#define BW_TEST_MAX_POINTS 40  // 最大测试频率点数

typedef struct {
  uint8_t  enable;              // 测试使能
  uint8_t  done;                // 测试完成标志
  uint8_t  stopping;            // 停机斜坡阶段
  int16_t  ramp_ref;            // 斜坡停机当前电流指令
  float    freq_start;          // 起始频率 (Hz)
  float    freq_end;            // 结束频率 (Hz)
  uint8_t  points_per_decade;   // 每十倍频程点数
  float    amplitude;           // 注入幅值 (Q10, 即 1024 = 1A)

  // 运行时状态
  uint16_t current_point;       // 当前测试点索引
  uint16_t total_points;        // 总测试点数
  float    current_freq;        // 当前频率 (Hz)
  uint32_t phase_accum;         // 相位累加器 (Q16.16)
  uint32_t phase_step;          // 相位步进

  // 同步检测累加器
  int64_t  sum_sin;             // sin 分量
  int64_t  sum_cos;             // cos 分量
  int64_t  ref_sin;             // 参考信号 sin 分量
  int64_t  ref_cos;             // 参考信号 cos 分量
  uint32_t sample_count;        // 当前频率点已采样数
  uint32_t samples_needed;      // 当前频率点需要的采样数
  uint32_t settle_samples;      // 稳态等待采样数（预计算）

  // 结果
  float    freq_list[BW_TEST_MAX_POINTS];
  float    gain_db[BW_TEST_MAX_POINTS];
  float    phase_deg[BW_TEST_MAX_POINTS];
} CurrentLoopBWTest;

/*******************************************************************************
 * 速度环带宽测试结构体
 *******************************************************************************/
typedef struct {
  uint8_t  enable;              // 测试使能
  uint8_t  done;                // 测试完成标志
  uint8_t  stopping;            // 停机斜坡阶段
  int32_t  ramp_ref;            // 斜坡停机当前速度指令（int32_t，速度值范围大）
  float    freq_start;          // 起始频率 (Hz)
  float    freq_end;            // 结束频率 (Hz)
  uint8_t  points_per_decade;   // 每十倍频程点数
  float    amplitude_base;      // 低频段恒定注入幅值 (内部速度单位, 1rpm = 1024*101)
  float    f_break;             // 拐点频率 (Hz), f>f_break 后注入幅值按 f_break/f 衰减
  int32_t  current_amplitude;   // 当前频点实际注入幅值, 切频时按 amp_base*f_break/f 重算

  // 运行时状态
  uint16_t current_point;       // 当前测试点索引
  uint16_t total_points;        // 总测试点数
  float    current_freq;        // 当前频率 (Hz)
  uint32_t phase_accum;         // 相位累加器 (Q16.16)
  uint32_t phase_step;          // 相位步进

  // 同步检测累加器
  int64_t  sum_sin;             // sin 分量
  int64_t  sum_cos;             // cos 分量
  int64_t  ref_sin;             // 参考信号 sin 分量
  int64_t  ref_cos;             // 参考信号 cos 分量
  uint32_t sample_count;        // 当前频率点已采样数
  uint32_t samples_needed;      // 当前频率点需要的采样数
  uint32_t settle_samples;      // 稳态等待采样数（预计算）

  // 结果
  float    freq_list[BW_TEST_MAX_POINTS];
  float    gain_db[BW_TEST_MAX_POINTS];
  float    phase_deg[BW_TEST_MAX_POINTS];
  uint32_t udc_peak[BW_TEST_MAX_POINTS];   // 每个频点测量阶段 Udc 峰值 (0.1V)

  // 异常中止信息
  uint8_t  abort_reason;        // 0=正常完成, 1=Udc 接近过压
  uint32_t abort_udc;           // 中止瞬间的 Udc (0.1V)
  float    abort_freq;          // 中止瞬间的扫频频率 (Hz)
  // 测试期间临时修改的阈值, 用于结束时还原
  uint8_t  saved_velocity_coe;
} SpeedLoopBWTest;

/*******************************************************************************
 * 位置环带宽测试结构体
 *******************************************************************************/
typedef struct {
  uint8_t  enable;              // 测试使能
  uint8_t  done;                // 测试完成标志
  uint8_t  stopping;            // 停机阶段
  float    freq_start;          // 起始频率 (Hz)
  float    freq_end;            // 结束频率 (Hz)
  uint8_t  points_per_decade;   // 每十倍频程点数
  float    amplitude_base;      // 低频段恒定注入幅值 (位置单位, 1°=1024)
  float    f_break;             // 拐点频率 (Hz), f>f_break 后注入幅值按 f_break/f 衰减
  int32_t  current_amplitude;   // 当前频点实际注入幅值

  // 运行时状态
  uint16_t current_point;       // 当前测试点索引
  uint16_t total_points;        // 总测试点数
  float    current_freq;        // 当前频率 (Hz)
  uint32_t phase_accum;         // 相位累加器
  uint32_t phase_step;          // 相位步进

  // 同步检测累加器
  int64_t  sum_sin;
  int64_t  sum_cos;
  int64_t  ref_sin;
  int64_t  ref_cos;
  uint32_t sample_count;
  uint32_t samples_needed;
  uint32_t settle_samples;

  // 结果
  float    freq_list[BW_TEST_MAX_POINTS];
  float    gain_db[BW_TEST_MAX_POINTS];
  float    phase_deg[BW_TEST_MAX_POINTS];

  // 异常中止
  uint8_t  abort_reason;
  float    abort_freq;
  uint16_t saved_velocity_coe;
} PositionLoopBWTest;

/*******************************************************************************
 * 电机错误结构体********************************************************************************/
typedef struct {
  uint32_t OverBusVolErr : 1;            // 错误标志位,母线过压1
  uint32_t LowBusVolErr : 1;             // 错误标志位,母线欠压2
  uint32_t OverBusCurrentErr : 1;        // 错误标志位,过流3
  uint32_t HighBoardTempErr : 1;         // 错误标志位,板子过热4

  uint32_t HighMotorTempErr : 1;         // 错误标志位,电机过热5
  uint32_t LockedRotorErr : 1;           // 错误标志位,堵转 6
  uint32_t EncoderErr : 1;               // 错误标志位,编码器故障7
  uint32_t DriverChipNfault : 1;         // 错误标志位,驱动芯片故障8

  uint32_t sto_activated : 1;            // STO触发9
  uint32_t MosFault : 1;                 // 错误标志位,mos管失效
  uint32_t CommunicateErr : 1;           // 错误标志位,通讯故障11
  uint32_t OverSpeedErr : 1;             // 错误标志位,速度过大12

  uint32_t OverPositionErr : 1;          // 错误标志位，位置过大13
  uint32_t PhaseUVolErr : 1;             // 错误标志位，U相故障14
  uint32_t PhaseVVolErr : 1;             // 错误标志位，v相故障15
  uint32_t PhaseWVolErr : 1;             // 错误标志位，w相故障16

  uint32_t PhaseCurrentSampleErr : 1;    // 相电流采样故障17
  uint32_t CommunicateFlag : 1;          // 1bit通讯状态标志位 0通讯正常 1通讯失败 18
  // 以下为软件内部标志位，不对外通讯
  uint32_t DCBusSampleErr : 1;                         // 1bit母线电压采集标志位 0正常 1异常 19
  uint32_t BoradTemSampleErr : 1;                      // 1bit板子温度采集标志位 0正常 1异常 20

  uint32_t MotorTemSampleErr : 1;                      // 1bit电机问题采集标志位 0正常 1异常 21
  uint32_t PhaseOrderErr : 1;                          // 电机三相线序错误
  uint32_t UserCommendValueErr : 1;                    // 用户传入的参数指令错误
  uint32_t MotorMaxAccErr : 1;                         // 电机加速度过大

  uint32_t MotorMaxJerkErr : 1;                        // 电机加加速度过大
  uint32_t speedOffsetErr : 1;                         // 速度偏差过大
  uint32_t eepromDataErr : 1;                          // ecat的eeprom参数异常
  uint32_t posOffsetErr : 1;                           // 位置偏差过大

  uint32_t zeroPointErr : 1;                           // 零点故障
  uint32_t currentOffsetErr : 1;                       // 30
  uint32_t flashReadErr : 1;                           // flash读取故障
  uint32_t Temp11 : 1;                                 // 32
} Servo_Flag;

#define MOTOR_PHASE_CURRENT_SAMPLE_FAULT 0x00010000    // 错误标志位,电流采样错误
#define MOTOR_PHASE_ORDER_FAULT 0x00080000             // 错误标志位,三相线序错误
#define MOTOR_DRIVER_CHIP_FAULT 0x00000080             // 错误标志位,驱动芯片故障8
#define MOTOR_LOCKED_FAULT 0x00000020                  // 错误标志位,堵转 6
// #define MOTOR_POS_OFFSET_VALUE            2048//1degree
// #define MOTOR_POS_OFFSET_COUNT            3
#define MOTOR_SPEED_OFFSET_VALUE 10240    // 10rpm  10*101*1024
#define MOTOR_SPEED_OVER_VALUE 3102720    // 30rpm
#define MOTOR_SPEED_OFFSET_COUNT 2048
typedef union {
  uint32_t All_Flag;
  Servo_Flag Bit;
} Servo_Flag_Unin;

/*******************************************************************************
 * 电机状态结构体********************************************************************************/
typedef struct {
  uint32_t ServoRunning : 1;           // 伺服运行状态反馈，1运行中，0停止
  uint32_t ServoRunDriction : 1;       // 伺服运行方向，1角度增加，0角度减少
  uint32_t SerRunMode : 2;             // 运行模式
  uint32_t CurrentArrivedFlag : 1;     // 电流模式下电流反馈到达指令要求标志位
  uint32_t SpeedArrivedFlag : 1;       // 速度模式下速度反馈到达指令要求标志位
  uint32_t PositionArrivedFlag : 1;    // 位置模式下位置反馈到达指令要求标志位
  uint32_t BoardTempWarning : 1;
  uint32_t PdoRefreshing : 1;
} Servo_State;

typedef union {
  uint32_t All_Flag;
  Servo_State Bit;
} Servo_State_Unin;

/*******************************************************************************FLASH保存参数结构体********************************************************************************/
typedef struct {
  uint16_t StructVersion;  // 结构体版本号，用于检测 Flash 数据兼容性（当前版本=2，uint32_t PID）
  uint16_t CurrentFlag;    // 00||FF为无 70为辨识失败重新辨识，其余为有
  uint16_t Ia_offset;      // ADC零位偏差 停机设定
  uint16_t Ib_offset;
  uint16_t Ic_offset;

  uint16_t AngleOffsetFlag;    // 00||FF为无 其余为有
  uint16_t elec_offest_0;      // 电气角度偏移 停机设定,方向-1
  uint16_t elec_offest_1;      // 电气角度偏移 停机设定，方向1
  int32_t mech_offest;         // 机械角度偏移 停机设定，立即生效
  int32_t temp1;               // 预留成员1
  int32_t temp2;               // 预留成员2

  uint16_t PidFlag;            // PID数据 00||FF为无 其余为有 运行设定，立即生效
  uint32_t Position_Kp;        // 升级到 uint32_t 支持大增益（如 85000）
  uint32_t Position_Ki;
  uint32_t Position_Kd;
  int32_t Pid_PositionLimit;
  uint32_t Speed_Kp;
  uint32_t Speed_Ki;
  uint32_t Speed_Kd;
  int32_t Pid_SpeedLimit;
  /* proportional gain for position-error feedforward (units rpm/deg, scaled Q10) */
  int32_t PosErrFF_Kp;
  uint32_t Current_Kp;
  uint32_t Current_Ki;
  uint32_t Current_Kd;
  int32_t Pid_CurrentLimit;
  int32_t temp3;               // 预留成员3
  int32_t temp4;               // 预留成员4

  uint16_t ArrivedFlag;             // 指令到达阈值 00||FF为无 其余为有 运行设定，立即生效
  uint16_t PositionArrivedValue;    // 0.1度
  uint16_t SpeedArrivedValue;       // 0.1rpm
  uint16_t CurrentArrivedValue;     // 0.1A

  uint16_t RunDataFlag;             // 运行数据限制 00||FF为无 其余为有 运行设定，立即生效
  uint16_t RunMode;                 // 运行模式 CIA AXIS
  int32_t MaxSpeed;                 // 单位0.1rpm
  uint16_t MaxCurrent;              // 单位0.1A
  uint16_t PositionLimitFlag;
  int32_t MaxPositionLimit;         // 单位0.1°
  int32_t MinPositionLimit;         // 单位0.1°
  int32_t temp5;               // 预留成员5
  int32_t temp6;               // 预留成员6

  uint16_t ProteckKeyFlag;          // 保护功能开关  00||FF为无 其余为有 运行设定，立即生效
  uint16_t Sto_1_protectKey;        // 10关 20开
  uint16_t Sto_2_protectKey;        // 10关 20开
  uint16_t BusVolProteckKey;        // 10关 20开
  uint16_t LockedRotorProtectKey;

  int8_t InvertDirflag;             // 电机正反转方向
  uint16_t brake_time;              // 抱闸延时时间
  __IO int32_t mech_offest_out;     // 输出端偏移值
  uint32_t stoStateFlag;            // STO标志位，0-sto不启用，1-stoa启用，2-stob启用，3-stoab启用
  int32_t temp7;               // 预留成员7
  int32_t temp8;               // 预留成员8

  uint32_t Crc;                      // Crc校验位
} FlashSavedData;

/*******************************************************************************
 * 电机控制结构体********************************************************************************/
typedef struct {
  /*电机基本数据*/
  uint8_t controller_mode;     // 控制模式
  uint8_t foc_run;             // 启动运行命令
  uint8_t stoCommend;          // sto控制参数
  uint8_t UserDataSaveFlag;    // 用户参数保存写入flash命令

  /*电机FOC计算数据*/
  uint32_t Ia_raw, Ib_raw, Ic_raw;               // ADC采样值
  int32_t I_a, I_b, I_c;                         // 相电流
  int32_t I_a_Filter, I_b_Filter, I_c_Filter;    // 滤波后的相电流

  uint8_t count_loop;                            // 电流环执行次数计数值
  int32_t theta_mech;                            // 转子机械角度，Q18
  int32_t theta_elec;                            // 转子电角度，uq16
  int32_t theta_elec_raw;                        // 转子电角度，单位°
  int32_t old_mechposition;                      // 上一次的机械位置
  int32_t now_mechposition;                      // 当前机械位置

  int32_t dtheta_mech;                           // 转子机械角速度，单位1rpm/1024
  int32_t dtheta_mech_out;
  int32_t velocity_ref;                          // 速度参考值
  int32_t velocity_ref_filterd;                  // 滤波后的速度参考值
  /* feedforward term computed from position error (same units as velocity_ref) */
  int32_t pos_err_ff_gain;                       // 位置误差前馈增益，单位rpm/deg,电机端
  int32_t I_d, I_q;                              // D/Q 电流
  int32_t I_d_ref, I_q_ref;                      // DQ轴电流参考值，单位1A/1024
  int32_t I_q_ref_filterd;                       // 滤波后的参考电流
  int32_t V_d, V_q;                              // D/Q 电压
  int32_t V_alpha, V_beta;                       // alpha beta 轴电压
  int32_t I_alpha, I_beta;                       // alpha beta 轴电流

  uint32_t CCR2, CCR3, CCR4;                     // 定时器比较值
  int32_t circle_count;                          // 选择物理圈数计数
  __IO int32_t circle_count_out;                 // 选择物理圈数计数
  uint32_t old_angle_count;                      // 上一次循环角度值
  __IO uint32_t old_angle_count_out;             // 上一次循环角度值
  __IO uint32_t old_angle_count_out_raw;         // 上一次循环角度原始值

  int32_t real_position;                         // 减去偏差之后的实际位置，
  int32_t real_position_out;                     // 输出端编码器值
  int32_t real_position_out_pre;                 // 输出端编码器上次值
  int32_t real_position_raw;                     // 未减去偏差的位置
  int32_t position_ref;                          // 参考位置     单位1°/1024
  int32_t position_ref_filterd;                  // 滤波后参考位置     单位1°/1024

   /*弱磁*/
  uint32_t Us;
  uint32_t Us_raw;
  int32_t compensation_weak;
  int32_t voltage_error;
  int32_t speed_error;

  /*电机PID*/
  IncPID IncPID_QAxis;
  IncPID IncPID_Speed;
  IncPID IncPID_Position;
  IncPID IncPID_DAxis;
  /*电机掉电保存数据*/
  FlashSavedData FlashData;
  /*电流环滤波器*/
  CurrentLoopSmooth CurrentSmooth;    // 电流环电流指令斜坡滤波结构体
  str_FILTER1 IqShowFilter;           // Iq显示数字滤波器
  /*速度环滤波器*/
  movingAverage_s32t Speed_Filter;    // 速度计算滑动均值滤波器
  SpeedLoopSmooth SpeedSmooth;        // 速度环斜坡滤波器
  str_FILTER1 SpeedShowFilter;        // 速度显示数字滤波器
  /*位置环滤波器*/
  PositionRefSmooth SmoothPosRef;    // 位置指令斜坡结构体
  str_FILTER1 PositionRefFilter;     // 位置指令RC数字滤波器
  str_FILTER1 PositionShowFilter;    // 位置显示数字滤波器
  /*电机状态及错误信息数据*/
  Servo_Flag_Unin ServoErrFlag;    // 伺服错误标志位
  Servo_State_Unin ServoState;     // 伺服状态标志位

  /*电流环带宽测试*/
  CurrentLoopBWTest bw_test;
  /*速度环带宽测试*/
  SpeedLoopBWTest spd_bw_test;
  /*位置环带宽测试*/
  PositionLoopBWTest pos_bw_test;

  /*电感辨识*/
  InductanceIdent ident_test;

} ControllerStruct;

void set_ver_par(uint8_t id);
void set_phase_voltage(ControllerStruct* controller, int32_t d, int32_t q, int32_t Theta);
void controller_init(ControllerStruct* controller);
void set_usart_control(uint8_t val);
uint8_t get_usart_control(void);

#endif
