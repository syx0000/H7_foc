/**
 * @file    can_debug.c
 * @brief   CAN-FD debug channel (0x7E0~0x7EF), 32B payload limit
 */
#include "can_debug.h"
#include "fdcan.h"
#include "flash_port.h"
#include "foc_controller.h"
#include "ifly_fault.h"
#include "ota_app.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>

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
static void send_event(const uint8_t *d, uint32_t n) { fdcan_send(CAN_DBG_ID_EVENT, d, n); }
static void send_text(const char *str) {
    uint32_t len = strlen(str);
    if (len > 32) len = 32;  /* CAN 帧限制 */
    fdcan_send(CAN_DBG_ID_TEXT, (const uint8_t*)str, len);
}
static void send_ota_ack(uint8_t type, uint16_t seq, uint8_t reason) {
    /* type: 0x00=ACK, 0x01=NAK, 0xFF=DONE */
    uint8_t r[4] = {type, seq & 0xFF, (seq >> 8) & 0xFF, reason};
    fdcan_send(CAN_DBG_ID_OTA_ACK, r, 4);
}
static void send_err(uint8_t cmd, can_dbg_err_t e) {
    uint8_t r[3] = {CAN_DBG_ERR_FLAG, cmd, (uint8_t)e};
    send_resp(r, 3);
}

static void w_u16(uint8_t *p, uint16_t v) { p[0]=v; p[1]=v>>8; }
static void w_u32(uint8_t *p, uint32_t v) { p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }
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

static void h_get_params(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    /* 返回格式: [CMD][OK][CurKp:u16][CurKi:u16][CurKd:u16][SpdKp:u16][SpdKi:u16][SpdKd:u16]
                      [PosKp:u16][PosKi:u16][PosKd:u16][OffPos:i16][OffNeg:i16][CompPos:i16][CompNeg:i16]
       = 2 + 9×2 + 4×2 = 28B */
    uint8_t r[28] = {0};
    r[0] = CAN_DBG_CMD_GET_PARAMS;
    r[1] = CAN_DBG_OK;

    uint16_t cur_kp = (uint16_t)controller_eyou.IncPID_QAxis.P;
    uint16_t cur_ki = (uint16_t)controller_eyou.IncPID_QAxis.I;
    uint16_t cur_kd = (uint16_t)controller_eyou.IncPID_QAxis.D;
    uint16_t spd_kp = (uint16_t)controller_eyou.IncPID_Speed.P;
    uint16_t spd_ki = (uint16_t)controller_eyou.IncPID_Speed.I;
    uint16_t spd_kd = (uint16_t)controller_eyou.IncPID_Speed.D;
    uint16_t pos_kp = (uint16_t)controller_eyou.IncPID_Position.P;
    uint16_t pos_ki = (uint16_t)controller_eyou.IncPID_Position.I;
    uint16_t pos_kd = (uint16_t)controller_eyou.IncPID_Position.D;

    extern int16_t g_theta_offset_pos, g_theta_offset_neg, g_theta_comp_pos, g_theta_comp_neg;

    r[2]  = cur_kp & 0xFF; r[3]  = (cur_kp >> 8) & 0xFF;
    r[4]  = cur_ki & 0xFF; r[5]  = (cur_ki >> 8) & 0xFF;
    r[6]  = cur_kd & 0xFF; r[7]  = (cur_kd >> 8) & 0xFF;
    r[8]  = spd_kp & 0xFF; r[9]  = (spd_kp >> 8) & 0xFF;
    r[10] = spd_ki & 0xFF; r[11] = (spd_ki >> 8) & 0xFF;
    r[12] = spd_kd & 0xFF; r[13] = (spd_kd >> 8) & 0xFF;
    r[14] = pos_kp & 0xFF; r[15] = (pos_kp >> 8) & 0xFF;
    r[16] = pos_ki & 0xFF; r[17] = (pos_ki >> 8) & 0xFF;
    r[18] = pos_kd & 0xFF; r[19] = (pos_kd >> 8) & 0xFF;

    int16_t off_pos = g_theta_offset_pos;
    int16_t off_neg = g_theta_offset_neg;
    int16_t cmp_pos = g_theta_comp_pos;
    int16_t cmp_neg = g_theta_comp_neg;

    r[20] = off_pos & 0xFF; r[21] = (off_pos >> 8) & 0xFF;
    r[22] = off_neg & 0xFF; r[23] = (off_neg >> 8) & 0xFF;
    r[24] = cmp_pos & 0xFF; r[25] = (cmp_pos >> 8) & 0xFF;
    r[26] = cmp_neg & 0xFF; r[27] = (cmp_neg >> 8) & 0xFF;

    send_resp(r, 28);
}

static void h_logid(const uint8_t *d, uint32_t n) {
    if (n<3){send_err(CAN_DBG_CMD_LOGID_SET,CAN_DBG_ERR_BAD_LEN);return;}
    uint16_t id=d[1]|(d[2]<<8);
    dbgLogFlag=id;
    uint8_t r[4]={CAN_DBG_CMD_LOGID_SET, CAN_DBG_OK, d[1], d[2]};
    send_resp(r,4);
    char msg[32];
    snprintf(msg, sizeof(msg), "logid set to %u", id);
    send_text(msg);
}

static void h_logfreq(const uint8_t *d, uint32_t n) {
    if (n<3){send_err(CAN_DBG_CMD_LOGFREQ_SET,CAN_DBG_ERR_BAD_LEN);return;}
    uint16_t p=d[1]|(d[2]<<8);
    logPriodMs=p;
    uint8_t r[4]={CAN_DBG_CMD_LOGFREQ_SET, CAN_DBG_OK, d[1], d[2]};
    send_resp(r,4);
    char msg[32];
    snprintf(msg, sizeof(msg), "logfreq set to %u ms", p);
    send_text(msg);
}

static void h_pid(const uint8_t *d, uint32_t n, uint8_t cmd) {
    if (n<13){send_err(cmd,CAN_DBG_ERR_BAD_LEN);return;}
    uint32_t kp=d[1]|(d[2]<<8)|(d[3]<<16)|(d[4]<<24);
    uint32_t ki=d[5]|(d[6]<<8)|(d[7]<<16)|(d[8]<<24);
    uint32_t kd=d[9]|(d[10]<<8)|(d[11]<<16)|(d[12]<<24);
    const char *loop_name = "PID";
    if (cmd==CAN_DBG_CMD_CUR_PID_SET) {
        controller_eyou.IncPID_QAxis.P=kp; controller_eyou.FlashData.Current_Kp=kp;
        controller_eyou.IncPID_QAxis.I=ki; controller_eyou.FlashData.Current_Ki=ki;
        controller_eyou.IncPID_QAxis.D=kd; controller_eyou.FlashData.Current_Kd=kd;
        controller_eyou.IncPID_DAxis.P=kp;
        controller_eyou.IncPID_DAxis.I=ki;
        controller_eyou.IncPID_DAxis.D=kd;
        loop_name = "Current";
    } else if (cmd==CAN_DBG_CMD_SPD_PID_SET) {
        controller_eyou.IncPID_Speed.P=kp; controller_eyou.FlashData.Speed_Kp=kp;
        controller_eyou.IncPID_Speed.I=ki; controller_eyou.FlashData.Speed_Ki=ki;
        controller_eyou.IncPID_Speed.D=kd; controller_eyou.FlashData.Speed_Kd=kd;
        loop_name = "Speed";
    } else {
        controller_eyou.IncPID_Position.P=kp; controller_eyou.FlashData.Position_Kp=kp;
        controller_eyou.IncPID_Position.I=ki; controller_eyou.FlashData.Position_Ki=ki;
        controller_eyou.IncPID_Position.D=kd; controller_eyou.FlashData.Position_Kd=kd;
        loop_name = "Position";
    }
    uint8_t r[14]; r[0]=cmd; r[1]=CAN_DBG_OK;
    memcpy(&r[2], &d[1], 12);
    send_resp(r,14);
    char msg[32];
    snprintf(msg, sizeof(msg), "%s PID: %lu/%lu/%lu", loop_name,
             (unsigned long)kp, (unsigned long)ki, (unsigned long)kd);
    send_text(msg);
}

static void h_flash_write(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    extern void WriteDataToFlash(void);
    WriteDataToFlash();
    uint8_t r[2]={CAN_DBG_CMD_FLASH_WRITE, CAN_DBG_OK};
    send_resp(r,2);
    send_text("Flash write OK");
}

static void h_flash_erase(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    extern HAL_StatusTypeDef Flash_EraseSector(void);
    uint8_t r[2]={CAN_DBG_CMD_FLASH_ERASE, CAN_DBG_OK};
    if (Flash_EraseSector()!=HAL_OK) {
        send_err(CAN_DBG_CMD_FLASH_ERASE,CAN_DBG_ERR_BUSY);
        send_text("Flash erase FAIL");
        return;
    }
    send_resp(r,2);
    send_text("Flash erase OK");
}

static void h_flash_compare(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    /* 比较 RAM vs Flash: 逐字段对比, 输出文本结果 */
    extern void Flash_ReadData(uint32_t, void*, uint32_t);

    FlashSavedData flash_copy;
    Flash_ReadData(FLASH_USER_START_ADDR, &flash_copy, sizeof(FlashSavedData));

    FlashSavedData *ram = &controller_eyou.FlashData;
    FlashSavedData *fls = &flash_copy;

    uint8_t r[2]={CAN_DBG_CMD_FLASH_COMPARE, CAN_DBG_OK};
    send_resp(r, 2);

    /* 逐字段对比关键参数 */
    int diff_count = 0;
    char msg[32];

    if (ram->StructVersion != fls->StructVersion) {
        snprintf(msg, sizeof(msg), "DIFF Ver: %u vs %u",
                 ram->StructVersion, fls->StructVersion);
        send_text(msg);
        diff_count++;
    }
    if (ram->Current_Kp != fls->Current_Kp || ram->Current_Ki != fls->Current_Ki) {
        send_text("DIFF CurPID");
        diff_count++;
    }
    if (ram->Speed_Kp != fls->Speed_Kp || ram->Speed_Ki != fls->Speed_Ki) {
        send_text("DIFF SpdPID");
        diff_count++;
    }
    if (ram->Position_Kp != fls->Position_Kp || ram->Position_Ki != fls->Position_Ki) {
        send_text("DIFF PosPID");
        diff_count++;
    }
    if (ram->elec_offset != fls->elec_offset) {
        send_text("DIFF elec_offset");
        diff_count++;
    }
    if (ram->mech_offest_out != fls->mech_offest_out) {
        send_text("DIFF mech_offest_out");
        diff_count++;
    }
    if (ram->Ia_offset != fls->Ia_offset || ram->Ib_offset != fls->Ib_offset) {
        send_text("DIFF I_offset");
        diff_count++;
    }
    if (ram->MaxCurrent != fls->MaxCurrent) {
        send_text("DIFF MaxCurrent");
        diff_count++;
    }
    if (ram->MaxSpeed != fls->MaxSpeed) {
        send_text("DIFF MaxSpeed");
        diff_count++;
    }

    if (diff_count == 0) {
        send_text("Flash compare: ALL MATCH");
    } else {
        snprintf(msg, sizeof(msg), "Flash compare: %d diff", diff_count);
        send_text(msg);
    }
}

static void h_fault_clr(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    ClearFaults(1);
    uint8_t r[2]={CAN_DBG_CMD_FAULT_CLR, CAN_DBG_OK};
    send_resp(r,2);
    send_text("All faults cleared");
}

static void h_enable(const uint8_t *d, uint32_t n) {
    if (n<2){send_err(CAN_DBG_CMD_ENABLE,CAN_DBG_ERR_BAD_LEN);return;}
    if (d[1]) {
        if (fault_brake_is_active()){
            send_err(CAN_DBG_CMD_ENABLE,CAN_DBG_ERR_BUSY);
            send_text("Enable FAIL: brake active");
            return;
        }
        if (controller_eyou.ServoErrFlag.All_Flag!=0){
            send_err(CAN_DBG_CMD_ENABLE,CAN_DBG_ERR_FAULT);
            send_text("Enable FAIL: faults present");
            return;
        }
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
    send_text(d[1] ? "PWM enabled" : "PWM disabled");
}

static void h_phase_comp_set(const uint8_t *d, uint32_t n) {
    if (n<9) {send_err(d[0],CAN_DBG_ERR_BAD_LEN);return;}
    int16_t off_pos  = (int16_t)(d[1] | (d[2]<<8));
    int16_t off_neg  = (int16_t)(d[3] | (d[4]<<8));
    int16_t comp_pos = (int16_t)(d[5] | (d[6]<<8));
    int16_t comp_neg = (int16_t)(d[7] | (d[8]<<8));
    extern int16_t g_theta_offset_pos, g_theta_offset_neg;
    extern int16_t g_theta_comp_pos, g_theta_comp_neg;
    g_theta_offset_pos = off_pos;
    g_theta_offset_neg = off_neg;
    g_theta_comp_pos = comp_pos;    g_theta_comp_neg = comp_neg;
    uint8_t r[10]={CAN_DBG_CMD_PHASE_COMP_SET, CAN_DBG_OK};
    memcpy(&r[2], &d[1], 8);
    send_resp(r, 10);
    char msg[32];
    snprintf(msg, sizeof(msg), "PhaseComp: %d/%d/%d/%d",
             off_pos, off_neg, comp_pos, comp_neg);
    send_text(msg);
}

static void h_phase_comp_save(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    extern void SavePhaseCompToFlash(void);
    SavePhaseCompToFlash();
    uint8_t r[2]={CAN_DBG_CMD_PHASE_COMP_SAVE, CAN_DBG_OK};
    send_resp(r,2);
    send_text("PhaseComp saved to Flash");
}

static void h_cali(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    /* 电角度偏置辨识 + Flash 保存 */
    uint8_t old_run = controller_eyou.foc_run;
    controller_eyou.foc_run = 0;
    controller_eyou.ServoErrFlag.All_Flag = 0;
    HAL_Delay(10);

    /* 重新使能 PWM 输出 */
    extern TIM_HandleTypeDef htim1;
    TIM1->CCER |= 0x0555u;
    __HAL_TIM_MOE_ENABLE(&htim1);

    extern void ElecAngleEstimate(ControllerStruct*);
    ElecAngleEstimate(&controller_eyou);

    extern HAL_StatusTypeDef Flash_EraseSector(void);
    extern void WriteDataToFlash(void);

    uint8_t r[2]={CAN_DBG_CMD_CALI, CAN_DBG_OK};
    if (Flash_EraseSector() != HAL_OK) {
        send_err(CAN_DBG_CMD_CALI, CAN_DBG_ERR_BUSY);
        send_text("Cali: Flash erase FAIL");
    } else {
        WriteDataToFlash();
        send_resp(r, 2);
        send_text("Cali done");
    }

    controller_eyou.foc_run = old_run;
}

static void h_bwtest(const uint8_t *d, uint32_t n) {
    if (n<2) {send_err(CAN_DBG_CMD_BWTEST,CAN_DBG_ERR_BAD_LEN);return;}
    uint8_t test_id = d[1];

    /* 先回 ACK，测试完成后再发 EVENT */
    uint8_t r[2]={CAN_DBG_CMD_BWTEST, CAN_DBG_OK};
    send_resp(r,2);

    /* 调用对应的测试函数（阻塞执行）*/
    extern void TestCurrentLoopBandwidth(void);
    extern void TestSpeedLoopBandwidth(void);
    extern void TestMotorParamsIdent(void);
    extern void TestFluxIdent(void);
    extern void TestInertiaIdent(void);
    extern void TestAutoTuneCurrent(void);
    extern void TestAutoTuneSpeed(void);
    extern void TestAutoTunePosition(void);
    extern void TestPositionLoopBandwidth(void);
    extern void TestDeadtimeCalibration(void);

    switch(test_id) {
        case 1: TestCurrentLoopBandwidth(); break;
        case 2: TestSpeedLoopBandwidth(); break;
        case 3: TestMotorParamsIdent(); break;
        case 4: TestFluxIdent(); break;
        case 5: TestInertiaIdent(); break;
        case 6: TestAutoTuneCurrent(); break;
        case 7: TestAutoTuneSpeed(); break;
        case 8: TestAutoTunePosition(); break;
        case 9: TestPositionLoopBandwidth(); break;
        case 10: TestDeadtimeCalibration(); break;
        default:
            send_err(CAN_DBG_CMD_BWTEST, CAN_DBG_ERR_OUT_OF_RANGE);
            return;
    }

    /* 测试完成，发送 EVENT 通知 */
    uint8_t evt[2]={0x01, test_id};  /* event_type=0x01(bwtest_done) */
    send_event(evt, 2);

    /* 发送文本结果消息 */
    char msg[32];
    snprintf(msg, sizeof(msg), "bwtest%d done", test_id);
    send_text(msg);
}

static void h_canrxdbg(const uint8_t *d, uint32_t n) {
    if (n<2) {send_err(d[0],CAN_DBG_ERR_BAD_LEN);return;}
    extern volatile uint8_t g_can_rx_debug;
    g_can_rx_debug = d[1];
    uint8_t r[3]={CAN_DBG_CMD_CANRXDBG, CAN_DBG_OK, d[1]};
    send_resp(r,3);
}

/* ===== CAN OTA 状态 ===== */
static struct {
    uint8_t  active;        /* 1=接收中 */
    uint16_t next_seq;      /* 期望的下一片序号 */
} s_can_ota = {0, 0};

/* CRC16-MODBUS, 与 ota_app.c 同实现 */
static uint16_t crc16_modbus_local(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

static void h_ota_begin(const uint8_t *d, uint32_t n) {
    if (n < 13) {send_err(CAN_DBG_CMD_OTA_BEGIN, CAN_DBG_ERR_BAD_LEN); return;}
    uint32_t size    = d[1] | (d[2]<<8) | (d[3]<<16) | (d[4]<<24);
    uint32_t crc32   = d[5] | (d[6]<<8) | (d[7]<<16) | (d[8]<<24);
    uint32_t version = d[9] | (d[10]<<8) | (d[11]<<16) | (d[12]<<24);

    if (ota_begin(size, crc32, version) != 0) {
        send_err(CAN_DBG_CMD_OTA_BEGIN, CAN_DBG_ERR_BUSY);
        send_text("OTA begin FAIL");
        return;
    }
    s_can_ota.active = 1;
    s_can_ota.next_seq = 0;

    uint8_t r[2] = {CAN_DBG_CMD_OTA_BEGIN, CAN_DBG_OK};
    send_resp(r, 2);
    send_text("OTA ready");
}

static void h_ota_end(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    s_can_ota.active = 0;
    if (ota_end() != 0) {
        send_err(CAN_DBG_CMD_OTA_END, CAN_DBG_ERR_INTERNAL);
        send_text("OTA end FAIL");
        return;
    }
    uint8_t r[2] = {CAN_DBG_CMD_OTA_END, CAN_DBG_OK};
    send_resp(r, 2);
    send_text("OTA done");
}

static void h_ota_abort(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    s_can_ota.active = 0;
    ota_abort();
    uint8_t r[2] = {CAN_DBG_CMD_OTA_ABORT, CAN_DBG_OK};
    send_resp(r, 2);
    send_text("OTA aborted");
}

static void h_ota_swap(const uint8_t *d, uint32_t n) {
    (void)d;(void)n;
    uint8_t r[2] = {CAN_DBG_CMD_OTA_SWAP, CAN_DBG_OK};
    send_resp(r, 2);
    send_text("Rebooting to bootloader");
    HAL_Delay(50);
    NVIC_SystemReset();
}

/* ===== CAN OTA 数据帧处理 (从 fdcan_rx_user 直接调用, ISR 上下文)
 *
 * 帧格式 (CAN-FD 32B, 受 RX FIFO 限制):
 *   D[0..1] : seq (u16 LE)
 *   D[2..3] : len (u16 LE)
 *   D[4..4+len-1] : payload (最大 24B)
 *   D[4+len..4+len+1] : crc16 (over D[0..3+len])
 *
 * 复用 ota_app.c 的 ring buffer + ota_process(): 把 CAN 帧重组为
 * 串口 'OD' + seq + len + payload + crc16 的格式喂进去. */
void can_debug_ota_data_rx(const uint8_t *data, uint32_t len) {
    if (!s_can_ota.active) return;
    if (len < 6) {
        send_ota_ack(0x01, 0xFFFF, CAN_DBG_ERR_BAD_LEN);
        return;
    }

    uint16_t seq      = data[0] | (data[1] << 8);
    uint16_t pay_len  = data[2] | (data[3] << 8);

    if (pay_len > 24 || pay_len + 6 > len) {
        send_ota_ack(0x01, seq, CAN_DBG_ERR_BAD_LEN);
        return;
    }

    /* CRC16 over D[0..3+pay_len] */
    uint16_t expected = crc16_modbus_local(data, 4 + pay_len);
    uint16_t actual   = data[4 + pay_len] | (data[4 + pay_len + 1] << 8);

    if (expected != actual) {
        send_ota_ack(0x01, seq, CAN_DBG_ERR_INTERNAL);
        return;
    }

    /* 序号检查 */
    if (seq != s_can_ota.next_seq) {
        send_ota_ack(0x01, seq, CAN_DBG_ERR_OUT_OF_RANGE);
        return;
    }

    /* 重组为 ota_app.c 期望的格式: 'OD' + seq[2] + len[2] + payload + crc16[2] */
    uint8_t reframe[6 + 24 + 2];
    reframe[0] = 'O';
    reframe[1] = 'D';
    reframe[2] = data[0]; reframe[3] = data[1];
    reframe[4] = data[2]; reframe[5] = data[3];
    memcpy(&reframe[6], &data[4], pay_len);
    uint16_t new_crc = crc16_modbus_local(reframe, 6 + pay_len);
    reframe[6 + pay_len]     = new_crc & 0xFF;
    reframe[6 + pay_len + 1] = (new_crc >> 8) & 0xFF;

    ota_rx_feed(reframe, 6 + pay_len + 2);
    /* 不在 ISR 中 process (会阻塞 Flash 写入几十 ms), 由主循环 can_debug_poll 处理 */

    s_can_ota.next_seq = (seq + 1) & 0xFFFF;
    send_ota_ack(0x00, seq, 0);  /* ACK */
}

/* ===== dispatch ===== */
static void dispatch(const uint8_t *d, uint32_t n) {
    if (!n) return;
    switch(d[0]) {
    case CAN_DBG_CMD_PING:       h_ping(d,n); break;
    case CAN_DBG_CMD_VERSION:    h_version(d,n); break;
    case CAN_DBG_CMD_RESET:      h_reset(d,n); break;
    case CAN_DBG_CMD_GET_PARAMS: h_get_params(d,n); break;
    case CAN_DBG_CMD_LOGID_SET:  h_logid(d,n); break;
    case CAN_DBG_CMD_LOGFREQ_SET:h_logfreq(d,n); break;
    case CAN_DBG_CMD_CUR_PID_SET:
    case CAN_DBG_CMD_SPD_PID_SET:
    case CAN_DBG_CMD_POS_PID_SET:h_pid(d,n,d[0]); break;
    case CAN_DBG_CMD_FLASH_WRITE:h_flash_write(d,n); break;
    case CAN_DBG_CMD_FLASH_ERASE:h_flash_erase(d,n); break;
    case CAN_DBG_CMD_FLASH_COMPARE:h_flash_compare(d,n); break;
    case CAN_DBG_CMD_FAULT_CLR:  h_fault_clr(d,n); break;
    case CAN_DBG_CMD_ENABLE:     h_enable(d,n); break;
    case CAN_DBG_CMD_PHASE_COMP_SET:  h_phase_comp_set(d,n); break;
    case CAN_DBG_CMD_PHASE_COMP_SAVE: h_phase_comp_save(d,n); break;
    case CAN_DBG_CMD_CALI:       h_cali(d,n); break;
    case CAN_DBG_CMD_BWTEST:     h_bwtest(d,n); break;
    case CAN_DBG_CMD_CANRXDBG:   h_canrxdbg(d,n); break;
    case CAN_DBG_CMD_OTA_BEGIN:  h_ota_begin(d,n); break;
    case CAN_DBG_CMD_OTA_END:    h_ota_end(d,n); break;
    case CAN_DBG_CMD_OTA_ABORT:  h_ota_abort(d,n); break;
    case CAN_DBG_CMD_OTA_SWAP:   h_ota_swap(d,n); break;
    default: send_err(d[0], CAN_DBG_ERR_UNKNOWN_CMD); break;
    }
}

/* ===== public API ===== */
void can_debug_init(void) { s_wr=0; s_rd=0; }

void can_debug_rx_isr(uint32_t id, const uint8_t *data, uint32_t len) {
    /* 0x7E4 OTA 数据帧: 直接调用 OTA 处理 (绕过命令队列, 不走 32B 限制) */
    if (id == CAN_DBG_ID_OTA_DATA) {
        can_debug_ota_data_rx(data, len);
        return;
    }
    /* 其他命令进队列, 主循环 poll 处理 */
    if (len>32) len=32;
    uint8_t next=(s_wr+1)&(Q_DEPTH-1);
    if (next==s_rd) return;
    s_q[s_wr].len=(uint8_t)len;
    memcpy((void*)s_q[s_wr].data, data, len);
    s_wr=next;
}

void can_debug_poll(void) {
    /* OTA 数据处理 (从 ring buffer → Flash, 主循环非阻塞) */
    if (s_can_ota.active) {
        ota_process();
    }

    /* 命令派发 */
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
        w_i32(&buf[16],(controller_eyou.dtheta_mech_out * 10) / 1024);  /* 载端 0.1 rpm/LSB 16阶MA */
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
    case 30:  // V_q, V_d
        w_i32(&buf[4], controller_eyou.V_q);
        w_i32(&buf[8], controller_eyou.V_d);
        fdcan_send(CAN_DBG_ID_LOG, buf, 12);
        break;
    case 60:  // CCR2, CCR3, CCR4
        w_u32(&buf[4], (uint32_t)controller_eyou.CCR2);
        w_u32(&buf[8], (uint32_t)controller_eyou.CCR3);
        w_u32(&buf[12],(uint32_t)controller_eyou.CCR4);
        fdcan_send(CAN_DBG_ID_LOG, buf, 16);
        break;
    case 90:  // Ia_raw, Ib_raw, Ic_raw
        w_i32(&buf[4], controller_eyou.Ia_raw);
        w_i32(&buf[8], controller_eyou.Ib_raw);
        w_i32(&buf[12],controller_eyou.Ic_raw);
        fdcan_send(CAN_DBG_ID_LOG, buf, 16);
        break;
    case 100: // position
        w_i32(&buf[4], controller_eyou.position_ref);
        w_i32(&buf[8], controller_eyou.real_position_out);
        w_i32(&buf[12],controller_eyou.position_ref - controller_eyou.real_position_out);
        w_i32(&buf[16],controller_eyou.FlashData.mech_offest_out);
        fdcan_send(CAN_DBG_ID_LOG, buf, 20);
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
