#ifndef _FOC_BSP_H_
#define _FOC_BSP_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Stub type definitions for cross-platform compatibility (HPMicro -> STM32) */
typedef struct { uint32_t dummy; } PWMV2_Type;
typedef struct { uint32_t dummy; } ADC16_Type;
typedef struct { uint32_t dummy; } TRGM_Type;
typedef struct { uint32_t dummy; } adc_type;

/* Windows-style type definitions for compatibility */
typedef uint8_t  BOOL;
typedef uint16_t UINT16;
typedef uint32_t UINT32;
typedef int16_t  INT16;
typedef int32_t  INT32;

/* STM32 __IO macro (volatile) */
#ifndef __IO
#define __IO volatile
#endif

/* RAM function attribute (for time-critical functions) */
/* ARM Compiler 5 doesn't support __attribute__ section syntax well, so disable it */
#ifndef ATTR_RAMFUNC
#define ATTR_RAMFUNC
#endif

/* Fast RAM placement attribute */
#ifndef ATTR_PLACE_AT_FAST_RAM_INIT
#define ATTR_PLACE_AT_FAST_RAM_INIT
#endif

/* Math constant M_PI */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* MIN/MAX macros */
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/* PWM period constant (should be defined based on actual PWM configuration) */
#ifndef PWM_T
#define PWM_T 12000  /* Default PWM period value */
#endif

/**********************************************************************************************/
// STM32H7 16位ADC + 10x运放 + 0.0025Ω采样电阻
// I(A) = (raw - offset) * 3.3 / 65535 / 10 / 0.0025
// I_Q10 = I * 1024 = (raw - offset) * 3.3 * 1024 / 65535 / 10 / 0.0025
//       ≈ (raw - offset) * 33 / 16   (误差 <0.01%)
#define CURRENT_TRANS_NUMERATOR 33
#define CURRENT_TRANS_DENOMINATOR 16

//
// 编码器分辨率常量 - DPT 双磁编码器 24位
#define ENCODER_BIT 16777216       // 24位 = 1<<24
#define ENCODER_BIT_HALF 8388608   // 1<<23
//
#define ENCODER_BIT_OUT 16777216       // 24位
#define ENCODER_BIT_HALF_OUT 8388608   // 1<<23

//
#define ENCODER_16BIT_DIV 8    // 24-16=8，(NPP*raw)%2^24 >> 8 → theta_elec 0~65536
#define ENCODER_10BIT_DIV 14   // 24-10=14，raw*360 >> 14 → real_position 1°/1024

/**********************************************************************************************/
#define UDC 48    //=1699.6%24
extern uint8_t NPP;

#define BUFF_SIZE (1024U)

/*pwm cmp set*/
#define PWM_IRQEN_CMPIRQEX_MASK (0xFFFFFFUL)
#define PWM_IRQEN_CMPIRQEX_SHIFT (0U)
#define PWM_IRQEN_CMPIRQEX_SET(x) (((uint32_t)(x) << PWM_IRQEN_CMPIRQEX_SHIFT) & PWM_IRQEN_CMPIRQEX_MASK)
#define PWM_IRQEN_CMPIRQEX_GET(x) (((uint32_t)(x) & PWM_IRQEN_CMPIRQEX_MASK) >> PWM_IRQEN_CMPIRQEX_SHIFT)

/*control mode*/
#define MOTORCONTROL_EC_OR_STUDIO 0    // 0: by ethercat 402; 1: by studio of uart
#define CIA402_PSITION_COEFFICIENT (10000)
#define CIA402_SPEED_COEFFICIENT (0.10922)
#define MCL_2PI (6.28318530717958647692)

/**
 * @brief    PWM frequency
 */
#define PWM_FREQUENCY (20000)

/**
 * @brief PWM output parameters
 *
 */
typedef struct bldc_control_pwmout_par {
    uint32_t dummy;
} BLDC_CONTROL_PWMOUT_PARA;
#define BLDC_CONTROL_PWMOUT_PARA_DEFAULTS {0, 0, 0, 0, 0, 0, NULL}

extern uint8_t dbgRecvBuf[1024];
extern volatile uint16_t usart_rx_len;
extern volatile uint16_t logPriodMs;

void pwmv2_duty_init(PWMV2_Type *ptr,
                     uint32_t PWM_PRD,
                     uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE,
                     uint8_t CMP_PWM_REGISTER_UPDATE_TYPE,
                     uint8_t CMP_SOURCE);
void adc_isr_enable(void);
void init_trigger_cfg(
    ADC16_Type *ptr, uint8_t trig_ch, uint8_t channel, bool inten, uint32_t ADC_MODULE, uint8_t ADC_PREEMPT_TRIG_LEN);
void init_trigger_mux(TRGM_Type *ptr, uint8_t TRAG_INPUT, uint8_t TRAG_INPUT_FOR_ADC);
void pwmv2_trigfor_adc_init(PWMV2_Type *ptr,
                            uint32_t PWM_PRD,
                            uint32_t PWM_CNT,
                            uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE,
                            uint8_t CMP_PWM_REGISTER_UPDATE_TYPE,
                            uint8_t PWM_TRIGOUT_CH_ADC,
                            uint8_t CMP_SOURCE,
                            uint8_t PWM_CH_TRIG_ADC);
void adc_module_cfg(adc_type *adc_typ, uint8_t adc_module, ADC16_Type *adc_base_ptr);
void pwmv2_trigfor_sei_init(PWMV2_Type *ptr,
                            uint32_t PWM_PRD,
                            uint32_t PWM_CNT,
                            uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE,
                            uint8_t CMP_PWM_REGISTER_UPDATE_TYPE,
                            uint8_t PWM_TRIGOUT_CH_SEI,
                            uint8_t CMP_SOURCE,
                            uint8_t PWM_CH_TRIG_SEI);
void led_init(void);
void break_motor_operation_init(void);
void dbg_cmd_set(void);
void dbg_log_print(void);
void adc_pins_init(void);
void adc_init_udc_temp(ADC16_Type *ptr, uint8_t udc_channel, uint8_t temp_channel, uint32_t sample_cycle);
void adc_cfg_init(ADC16_Type *ptr, uint8_t channel, uint32_t sample_cycle, uint32_t ADC_MODULE, uint32_t ADC_TRG);
void pwm_pins_init(void);
void pwm_ccr_set(uint32_t ccr1, uint32_t ccr2, uint32_t ccr3);
uint32_t motor_encoder_spi(uint8_t in_out);
uint64_t get_clock_cpu_ms(void);
uint8_t get_ver_id(void);
void sto_motor_operation_init(void);
void seiInterruptReset(void);
#endif
