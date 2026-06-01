/**
 * @file    can_debug.c
 * @brief   CAN-FD debug channel (0x7E0~0x7EF), 32B payload limit
 */
#include "can_debug.h"
#include "fdcan.h"
#include "foc_controller.h"
#include "ifly_fault.h"
#include "stm32h7xx_hal.h"
#include <string.h>

extern ControllerStruct controller_eyou;
extern volatile uint16_t dbgLogFlag;
extern volatile uint16_t logPriodMs;

/* ===== RX queue (ISR -> main) ===== */
#define Q_DEPTH 8u
typedef struct { uint8_t len; uint8_t data[32]; } rx_slot_t;
static volatile rx_slot_t s_q[Q_DEPTH];
static volatile uint8_t s_wr = 0, s_rd = 0;

/* ===== helpers ===== */
static void send_resp(const uint8_t *d, uint32_t n) { fdcan_send(CAN_DBG_ID_RESP, d, n); }
static void send_err(uint8_t cmd, can_dbg_err_t e) {
    uint8_t r[3] = {CAN_DBG_ERR_FLAG, cmd, (uint8_t)e};
    send_resp(r, 3);
}

static void w_u16(uint8_t *p, uint16_t v) { p[0]=v; p[1]=v>>8; }
static void w_i32(uint8_t *p, int32_t v) {
    uint32_t u=(uint32_t)v; p[0]=u; p[1]=u>>8; p[2]=u>>16; p[3]=u>>24;
}

/* ===== command handlers ===== */
static void h_ping(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    uint8_t r[4]={CAN_DBG_CMD_PING, CAN_DBG_OK, CAN_DBG_PROTO_VER, 0};
    send_resp(r, 4);
}

static void h_version(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    /* [CMD][OK][soft:10][hw:8][build:11] = 31B <= 32B */
    uint8_t r[32]={0};
    r[0]=CAN_DBG_CMD_VERSION; r[1]=CAN_DBG_OK;
    strncpy((char*)&r[2], SOFT_VERSION, 10);
    strncpy((char*)&r[12], HARD_VERSION, 8);
    strncpy((char*)&r[20], __DATE__, 11);
    send_resp(r, 31);
}

static void h_reset(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    uint8_t r[2]={CAN_DBG_CMD_RESET, CAN_DBG_OK};
    send_resp(r, 2);
    HAL_Delay(20);
    fault_safe_shutdown();
    HAL_Delay(50);
    NVIC_SystemReset();
}

static void h_logid(const uint8_t *d, uint32_t n) {
    if (n<3){send_err(CAN_DBG_CMD_LOGID_SET,CAN_DBG_ERR_BAD_LEN);return;}
    uint16_t id=d[1]|(d[2]<<8);
    dbgLogFlag=id;
    uint8_t r[4]={CAN_DBG_CMD_LOGID_SET, CAN_DBG_OK, d[1], d[2]};
    send_resp(r,4);
}

static void h_logfreq(const uint8_t *d, uint32_t n) {
    if (n<3){send_err(CAN_DBG_CMD_LOGFREQ_SET,CAN_DBG_ERR_BAD_LEN);return;}
    uint16_t p=d[1]|(d[2]<<8);
    logPriodMs=p;
    uint8_t r[4]={CAN_DBG_CMD_LOGFREQ_SET, CAN_DBG_OK, d[1], d[2]};
    send_resp(r,4);
}

static void h_pid(const uint8_t *d, uint32_t n, uint8_t cmd) {
    if (n<13){send_err(cmd,CAN_DBG_ERR_BAD_LEN);return;}
    uint32_t kp=d[1]|(d[2]<<8)|(d[3]<<16)|(d[4]<<24);
    uint32_t ki=d[5]|(d[6]<<8)|(d[7]<<16)|(d[8]<<24);
    uint32_t kd=d[9]|(d[10]<<8)|(d[11]<<16)|(d[12]<<24);
    if (cmd==CAN_DBG_CMD_CUR_PID_SET) {
        controller_eyou.IncPID_QAxis.P=kp; controller_eyou.FlashData.Current_Kp=kp;
        controller_eyou.IncPID_QAxis.I=ki; controller_eyou.FlashData.Current_Ki=ki;
        controller_eyou.IncPID_QAxis.D=kd; controller_eyou.FlashData.Current_Kd=kd;
        controller_eyou.IncPID_DAxis.P=kp;
        controller_eyou.IncPID_DAxis.I=ki;
        controller_eyou.IncPID_DAxis.D=kd;
    } else if (cmd==CAN_DBG_CMD_SPD_PID_SET) {
        controller_eyou.IncPID_Speed.P=kp; controller_eyou.FlashData.Speed_Kp=kp;
        controller_eyou.IncPID_Speed.I=ki; controller_eyou.FlashData.Speed_Ki=ki;
        controller_eyou.IncPID_Speed.D=kd; controller_eyou.FlashData.Speed_Kd=kd;
    } else {
        controller_eyou.IncPID_Position.P=kp; controller_eyou.FlashData.Position_Kp=kp;
        controller_eyou.IncPID_Position.I=ki; controller_eyou.FlashData.Position_Ki=ki;
        controller_eyou.IncPID_Position.D=kd; controller_eyou.FlashData.Position_Kd=kd;
    }
    uint8_t r[14]; r[0]=cmd; r[1]=CAN_DBG_OK;
    memcpy(&r[2], &d[1], 12);
    send_resp(r,14);
}

static void h_flash_write(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    extern void WriteDataToFlash(void);
    WriteDataToFlash();
    uint8_t r[2]={CAN_DBG_CMD_FLASH_WRITE, CAN_DBG_OK};
    send_resp(r,2);
}

static void h_flash_erase(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    extern HAL_StatusTypeDef Flash_EraseSector(void);
    uint8_t r[2]={CAN_DBG_CMD_FLASH_ERASE, CAN_DBG_OK};
    if (Flash_EraseSector()!=HAL_OK) { send_err(CAN_DBG_CMD_FLASH_ERASE,CAN_DBG_ERR_BUSY); return; }
    send_resp(r,2);
}

static void h_fault_clr(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    ClearFaults(1);
    uint8_t r[2]={CAN_DBG_CMD_FAULT_CLR, CAN_DBG_OK};
    send_resp(r,2);
}

static void h_enable(const uint8_t *d, uint32_t n) {
    if (n<2){send_err(CAN_DBG_CMD_ENABLE,CAN_DBG_ERR_BAD_LEN);return;}
    if (d[1]) {
        if (fault_brake_is_active()){send_err(CAN_DBG_CMD_ENABLE,CAN_DBG_ERR_BUSY);return;}
        if (controller_eyou.ServoErrFlag.All_Flag!=0){send_err(CAN_DBG_CMD_ENABLE,CAN_DBG_ERR_FAULT);return;}
        extern void ResetControlData(ControllerStruct*);
        ResetControlData(&controller_eyou);
        controller_eyou.I_q_ref=0;
        controller_eyou.velocity_ref=0;
        controller_eyou.position_ref=controller_eyou.real_position_out;
        controller_eyou.controller_mode=PROFILE_TORQUE_MODE;
        controller_eyou.foc_run=2;
        TIM1->BDTR|=TIM_BDTR_MOE;
        TIM1->CCER|=0x0555u;
    } else {
        fault_safe_shutdown();
    }
    uint8_t r[2]={CAN_DBG_CMD_ENABLE, CAN_DBG_OK};
    send_resp(r,2);
}

/* ===== dispatch ===== */
static void dispatch(const uint8_t *d, uint32_t n) {
    if (!n) return;
    switch(d[0]) {
    case CAN_DBG_CMD_PING:       h_ping(d,n); break;
    case CAN_DBG_CMD_VERSION:    h_version(d,n); break;
    case CAN_DBG_CMD_RESET:      h_reset(d,n); break;
    case CAN_DBG_CMD_LOGID_SET:  h_logid(d,n); break;
    case CAN_DBG_CMD_LOGFREQ_SET:h_logfreq(d,n); break;
    case CAN_DBG_CMD_CUR_PID_SET:
    case CAN_DBG_CMD_SPD_PID_SET:
    case CAN_DBG_CMD_POS_PID_SET:h_pid(d,n,d[0]); break;
    case CAN_DBG_CMD_FLASH_WRITE:h_flash_write(d,n); break;
    case CAN_DBG_CMD_FLASH_ERASE:h_flash_erase(d,n); break;
    case CAN_DBG_CMD_FAULT_CLR:  h_fault_clr(d,n); break;
    case CAN_DBG_CMD_ENABLE:     h_enable(d,n); break;
    default: send_err(d[0], CAN_DBG_ERR_UNKNOWN_CMD); break;
    }
}

/* ===== public API ===== */
void can_debug_init(void) { s_wr=0; s_rd=0; }

void can_debug_rx_isr(uint32_t id, const uint8_t *data, uint32_t len) {
    (void)id;
    if (len>32) len=32;
    uint8_t next=(s_wr+1)&(Q_DEPTH-1);
    if (next==s_rd) return;
    s_q[s_wr].len=(uint8_t)len;
    memcpy((void*)s_q[s_wr].data, data, len);
    s_wr=next;
}

void can_debug_poll(void) {
    while (s_rd!=s_wr) {
        dispatch((const uint8_t*)s_q[s_rd].data, s_q[s_rd].len);
        s_rd=(s_rd+1)&(Q_DEPTH-1);
    }
}

/* ===== 0x7E2 periodic log (all <= 32B) ===== */
static uint8_t s_seq=0;
void can_debug_send_log(void) {
    uint8_t buf[32];
    uint16_t id=dbgLogFlag;
    uint16_t ts=(uint16_t)(HAL_GetTick()&0xFFFF);
    buf[0]=(uint8_t)id; buf[1]=s_seq++; buf[2]=ts; buf[3]=ts>>8;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)<4) return;

    switch(id) {
    case 50:
        w_i32(&buf[4], controller_eyou.velocity_ref/1024);
        w_i32(&buf[8], controller_eyou.velocity_ref_filterd/1024);
        w_i32(&buf[12],controller_eyou.dtheta_mech/1024);
        w_i32(&buf[16],(controller_eyou.dtheta_mech/1024)/25);
        w_i32(&buf[20],(controller_eyou.velocity_ref-controller_eyou.dtheta_mech)/1024);
        fdcan_send(CAN_DBG_ID_LOG, buf, 24);
        break;
    case 40:
        w_i32(&buf[4], controller_eyou.I_q);
        w_i32(&buf[8], controller_eyou.I_d);
        w_i32(&buf[12],controller_eyou.V_q);
        w_i32(&buf[16],controller_eyou.V_d);
        w_i32(&buf[20],controller_eyou.I_q_ref);
        w_i32(&buf[24],controller_eyou.I_d_ref);
        w_i32(&buf[28],controller_eyou.I_q_ref_filterd);
        fdcan_send(CAN_DBG_ID_LOG, buf, 32);
        break;
    case 10:
        w_i32(&buf[4], controller_eyou.now_mechposition);
        w_u16(&buf[8], controller_eyou.theta_elec);
        w_i32(&buf[10],controller_eyou.real_position_out);
        w_i32(&buf[14],controller_eyou.real_position);
        w_i32(&buf[18],controller_eyou.dtheta_mech/1024);
        fdcan_send(CAN_DBG_ID_LOG, buf, 22);
        break;
    case 70:
        w_u16(&buf[4], (uint16_t)controller_eyou.CCR2);
        w_u16(&buf[6], (uint16_t)controller_eyou.CCR3);
        w_u16(&buf[8], (uint16_t)controller_eyou.CCR4);
        w_i32(&buf[10],controller_eyou.I_a);
        w_i32(&buf[14],controller_eyou.I_b);
        w_i32(&buf[18],controller_eyou.I_c);
        fdcan_send(CAN_DBG_ID_LOG, buf, 22);
        break;
    default: break;
    }
}

/* ===== 0x7E3 async event ===== */
void can_debug_send_event(uint8_t evt_id, const uint8_t *payload, uint32_t len) {
    uint8_t buf[32];
    if (len>31) len=31;
    buf[0]=evt_id;
    if (payload&&len) memcpy(&buf[1], payload, len);
    fdcan_send(CAN_DBG_ID_EVENT, buf, len+1);
}
