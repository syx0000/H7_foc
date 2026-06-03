"""
can_debug_protocol.py — CAN-FD 调试协议帧打包/解包

镜像 Core/Inc/can_debug.h 的 CMD_ID + 错误码 + 帧 schema.
保持 C 端 / Python 端字典同步, 修改时两边同时改.

详见 tools/canfd/CAN_DEBUG_DESIGN.md §3
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


# ===== CAN ID =====
ID_CMD       = 0x7E0
ID_RESP      = 0x7E1
ID_LOG       = 0x7E2
ID_EVENT     = 0x7E3
ID_OTA_DATA  = 0x7E4
ID_OTA_ACK   = 0x7E5
ID_TEXT      = 0x7E6


# ===== 协议版本 (与 can_debug.h::CAN_DBG_PROTO_VER 同步) =====
PROTO_VER = 1


# ===== 错误响应头标志 =====
ERR_FLAG = 0xFF


# ===== CMD_ID =====
class CMD(IntEnum):
    PING                = 0x00
    VERSION             = 0x01
    RESET               = 0x02
    GET_PARAMS          = 0x03
    # 日志类
    LOGID_SET           = 0x10
    LOGFREQ_SET         = 0x11
    # PID 类
    CUR_PID_SET         = 0x20
    SPD_PID_SET         = 0x21
    POS_PID_SET         = 0x22
    # Flash 类
    FLASH_WRITE         = 0x40
    FLASH_ERASE         = 0x41
    FLASH_COMPARE       = 0x42
    FAULT_CLR           = 0x43
    # 运动控制类
    ENABLE              = 0x50
    PHASE_COMP_SET      = 0x52
    PHASE_COMP_SAVE     = 0x53
    CALI                = 0x5F
    BWTEST              = 0x60
    # CAN 状态类
    CANRXDBG            = 0x61


# ===== 错误码 =====
class ERR(IntEnum):
    OK              = 0x00
    UNKNOWN_CMD     = 0x01
    BAD_LEN         = 0x02
    OUT_OF_RANGE    = 0x03
    BUSY            = 0x04
    FAULT           = 0x05
    BRAKE           = 0x06
    INTERNAL        = 0x07


# ===== 响应解析结果 =====
@dataclass
class CmdResp:
    cmd: int
    ok: bool
    err: Optional[ERR]
    payload: bytes  # 不含头 2~3 字节


def parse_resp(data: bytes) -> CmdResp:
    """解析 0x7E1 帧.

    成功: data[0]=CMD_ID,  data[1]=0x00, data[2:]=payload
    错误: data[0]=0xFF,    data[1]=CMD_ID, data[2]=ERR
    """
    if not data:
        raise ValueError("empty resp")
    if data[0] == ERR_FLAG:
        if len(data) < 3:
            raise ValueError(f"err resp too short: {data.hex()}")
        return CmdResp(cmd=data[1], ok=False, err=ERR(data[2]), payload=data[3:])
    cmd = data[0]
    if len(data) < 2:
        raise ValueError(f"resp too short: {data.hex()}")
    return CmdResp(cmd=cmd, ok=(data[1] == 0), err=None if data[1] == 0 else ERR(data[1]),
                   payload=data[2:])


# ===== 帧打包辅助 =====
def pack_ping() -> bytes:
    return bytes([CMD.PING])


def pack_version() -> bytes:
    return bytes([CMD.VERSION])


def pack_reset() -> bytes:
    return bytes([CMD.RESET])


def pack_get_params() -> bytes:
    """查询所有 PID + 相位补偿参数"""
    return bytes([CMD.GET_PARAMS])


# ===== 日志类 =====
def pack_logid_set(log_id: int) -> bytes:
    return struct.pack('<BH', CMD.LOGID_SET, log_id)


def pack_logfreq_set(period_ms: int) -> bytes:
    return struct.pack('<BH', CMD.LOGFREQ_SET, period_ms)


# ===== PID 类 =====
def pack_pid_set(cmd: CMD, kp: int, ki: int, kd: int) -> bytes:
    return struct.pack('<BIII', cmd, kp, ki, kd)


# ===== Flash 类 =====
def pack_flash_write() -> bytes:
    return bytes([CMD.FLASH_WRITE])


def pack_flash_erase() -> bytes:
    return bytes([CMD.FLASH_ERASE])


def pack_flash_compare() -> bytes:
    return bytes([CMD.FLASH_COMPARE])


def pack_fault_clear() -> bytes:
    return bytes([CMD.FAULT_CLR])


# ===== 运动控制类 =====
def pack_enable(en: bool) -> bytes:
    return bytes([CMD.ENABLE, 1 if en else 0])


def pack_phase_comp_set(off_pos: int, off_neg: int, comp_pos: int, comp_neg: int) -> bytes:
    """相位补偿参数 (i16×4)"""
    return struct.pack('<Bhhhh', CMD.PHASE_COMP_SET, off_pos, off_neg, comp_pos, comp_neg)


def pack_phase_comp_save() -> bytes:
    return bytes([CMD.PHASE_COMP_SAVE])


# ===== CAN 状态类 =====
def pack_canrxdbg(enable: bool) -> bytes:
    return bytes([CMD.CANRXDBG, 1 if enable else 0])


def pack_bwtest(test_id: int) -> bytes:
    """带宽测试/辨识/autoTune

    Args:
        test_id: 1=电流环BW, 2=速度环BW, 3=Rs/Ld/Lq, 4=磁链, 5=惯量,
                 6=电流autoTune, 7=速度autoTune, 8=位置autoTune,
                 9=位置环BW, 10=死区标定
    """
    return bytes([CMD.BWTEST, test_id])


def pack_cali() -> bytes:
    """电角度偏置辨识 + Flash 保存"""
    return bytes([CMD.CALI])


# ===== 0x7E2 周期日志解析 =====
@dataclass
class LogFrame:
    log_id: int
    seq: int
    ts_ms: int          # HAL_GetTick() 低 16 位
    fields: dict        # 各 LOG_ID 解析后的字段


# 各 LOG_ID 的 payload schema (与 can_debug.c::can_debug_send_log 同步)
# 头 4B 已被 _parse_hdr 吃掉, 这里只描述 payload
_LOG_SCHEMAS = {
    10: ('<iHiii',  ['now_mech', 'theta_elec', 'pos_out', 'pos', 'dtheta_div1024']),
    30: ('<ii',     ['V_q', 'V_d']),
    40: ('<iiiiiii',['I_q', 'I_d', 'V_q', 'V_d', 'I_q_ref', 'I_d_ref', 'I_q_ref_filterd']),
    50: ('<iiiii',  ['v_ref_rpm', 'v_ref_filt_rpm', 'v_fb_motor_rpm', 'v_fb_load_rpm', 'v_err_rpm']),
    60: ('<III',    ['CCR2', 'CCR3', 'CCR4']),
    70: ('<HHHiii', ['CCR2', 'CCR3', 'CCR4', 'I_a', 'I_b', 'I_c']),
    90: ('<iii',    ['Ia_raw', 'Ib_raw', 'Ic_raw']),
    100:('<iiii',   ['pos_ref', 'pos_fb', 'pos_err', 'mech_offset_out']),
}


def parse_log(data: bytes) -> LogFrame:
    """解析 0x7E2 周期日志帧.
    头 4B = [LOG_ID:u8][SEQ:u8][TS_MS:u16_le], 后续为该 LOG_ID 的 payload
    """
    if len(data) < 4:
        raise ValueError(f"log frame too short: {data.hex()}")
    log_id = data[0]
    seq = data[1]
    ts_ms = data[2] | (data[3] << 8)
    payload = data[4:]

    schema = _LOG_SCHEMAS.get(log_id)
    if schema is None:
        # 未知 LOG_ID, 返回原始 payload
        return LogFrame(log_id=log_id, seq=seq, ts_ms=ts_ms, fields={'raw': payload.hex()})

    fmt, names = schema
    expected = struct.calcsize(fmt)
    if len(payload) < expected:
        raise ValueError(f"log_id={log_id} payload too short: got {len(payload)}, need {expected}")
    values = struct.unpack(fmt, payload[:expected])
    return LogFrame(log_id=log_id, seq=seq, ts_ms=ts_ms,
                    fields=dict(zip(names, values)))


# ===== 0x7E3 异步事件解析 =====
EVT_BWTEST_DONE = 0x30
EVT_CALI_DONE   = 0x31


@dataclass
class EventFrame:
    event_id: int
    payload: bytes


def parse_event(data: bytes) -> EventFrame:
    """解析 0x7E3 异步事件帧."""
    if not data:
        raise ValueError("empty event")
    return EventFrame(event_id=data[0], payload=data[1:])


# ===== 响应解析辅助 =====
@dataclass
class VersionInfo:
    soft: str
    hard: str
    build: str


def parse_version_payload(payload: bytes) -> VersionInfo:
    """VERSION 响应 payload: [soft:10][hw:8][build:11] = 29B (32B FIFO 约束, 见 CAN_DEBUG_DESIGN.md §3.2)"""
    if len(payload) < 29:
        raise ValueError(f"version payload too short: {len(payload)}")

    def _take(buf: bytes) -> str:
        # 截到第一个 0 字节, 去掉尾随 0/空格
        end = buf.find(b'\x00')
        if end == -1:
            end = len(buf)
        return buf[:end].decode('ascii', errors='replace').strip()

    return VersionInfo(
        soft=_take(payload[0:10]),
        hard=_take(payload[10:18]),
        build=_take(payload[18:29]),
    )


@dataclass
class PingInfo:
    proto_ver: int


def parse_ping_payload(payload: bytes) -> PingInfo:
    """PING 响应 payload: [proto_ver:u8][reserved:u8]"""
    if len(payload) < 2:
        raise ValueError(f"ping payload too short: {len(payload)}")
    return PingInfo(proto_ver=payload[0])


@dataclass
class ParamsInfo:
    """所有 PID + 相位补偿参数"""
    cur_kp: int
    cur_ki: int
    cur_kd: int
    spd_kp: int
    spd_ki: int
    spd_kd: int
    pos_kp: int
    pos_ki: int
    pos_kd: int
    off_pos: int
    off_neg: int
    comp_pos: int
    comp_neg: int


def parse_params_payload(payload: bytes) -> ParamsInfo:
    """GET_PARAMS 响应 payload: 9×u16 + 4×i16 = 26B"""
    if len(payload) < 26:
        raise ValueError(f"params payload too short: {len(payload)}")
    # 小端序解包
    vals = struct.unpack('<9H4h', payload[:26])
    return ParamsInfo(
        cur_kp=vals[0], cur_ki=vals[1], cur_kd=vals[2],
        spd_kp=vals[3], spd_ki=vals[4], spd_kd=vals[5],
        pos_kp=vals[6], pos_ki=vals[7], pos_kd=vals[8],
        off_pos=vals[9], off_neg=vals[10], comp_pos=vals[11], comp_neg=vals[12]
    )
