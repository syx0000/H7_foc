/**
 * @file    can_protocol_sel.h
 * @brief   CAN 协议编译期选择
 *
 * 通过 CAN_PROTOCOL_SEL 宏在编译期决定使用哪套 CAN 协议:
 *   CAN_PROTO_WLY      - 万里扬 V1.7 (CAN-FD + BRS, 1M+5M)
 *   CAN_PROTO_CYBEAST  - 守护兽 CAN Simple (Classic CAN, 1M)
 *
 * 两套协议互斥, 不共存, 零运行时开销.
 */
#ifndef __CAN_PROTOCOL_SEL_H__
#define __CAN_PROTOCOL_SEL_H__

#define CAN_PROTO_WLY       0
#define CAN_PROTO_CYBEAST   1

/*========== 切换这里 ==========*/
#define CAN_PROTOCOL_SEL    CAN_PROTO_CYBEAST
/*==============================*/

#endif /* __CAN_PROTOCOL_SEL_H__ */
