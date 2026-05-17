#include "vision.h"
#include "usart.h"

/* K230 简化协议：AA room side confidence checksum 55 */
#define K230_FRAME_HEAD 0xAAU
#define K230_FRAME_TAIL 0x55U

/* OpenMV 兼容协议：2C 12 Num LoR Finded_flag FindTask 5B */
#define OPENMV_FRAME_HEAD1 0x2CU
#define OPENMV_FRAME_HEAD2 0x12U
#define OPENMV_FRAME_TAIL 0x5BU

/* K230 置信度低于 55 时，不认为识别结果可靠 */
#define VISION_MIN_CONFIDENCE 55U

/* 连续识别到多少次相同数字后认为目标稳定 */
#define VISION_STABLE_COUNT 1U

/* 每 5 个 10ms 周期发送一次任务，也就是约 50ms 一次 */
#define VISION_SEND_PERIOD_TICKS 5U

/* K230 接在 USART3 上 */
extern UART_HandleTypeDef huart3;

/* 串口接收状态机临时变量 */
static uint8_t rx_state;
static uint8_t rx_room;
static uint8_t rx_side;
static uint8_t rx_confidence;
static uint8_t rx_checksum;
static uint8_t rx_find_flag;
static uint8_t rx_find_task;

/* 最近一帧视觉结果，使用 volatile 是因为它们在串口中断回调中更新 */
static volatile uint8_t vision_num;
static volatile uint8_t vision_lor;
static volatile uint8_t vision_finded_flag;
static volatile uint8_t vision_find_task;
static volatile uint8_t vision_confidence;
static volatile uint8_t vision_last_byte;
static volatile uint32_t vision_rx_byte_count;
static volatile uint32_t vision_packet_count;
static volatile uint32_t vision_checksum_error_count;

/* 起点数字稳定识别相关变量 */
static uint8_t stable_digit;
static uint8_t stable_ready;
static uint8_t last_candidate;
static uint8_t candidate_count;

/* 当前需要周期发送给 K230 的任务和目标数字 */
static uint8_t task_to_send = 1U;
static uint8_t target_to_send;
static uint8_t send_tick;

static uint8_t Vision_IsValidDigit(uint8_t digit)
{
    /* 比赛房间号只允许 1~8 */
    return digit >= 1U && digit <= 8U;
}

static uint8_t Vision_IsValidSide(uint8_t side)
{
    /* 左右位置只允许无、左、右三种状态 */
    return side == VISION_LOR_NONE || side == VISION_LOR_LEFT || side == VISION_LOR_RIGHT;
}

static void Vision_UpdateStable(uint8_t room)
{
    /* 如果连续识别到同一个数字，就增加稳定计数 */
    if (last_candidate == room) {
        if (candidate_count < 255U) {
            candidate_count++;
        }
    } else {
        /* 数字发生变化时，重新开始统计 */
        last_candidate = room;
        candidate_count = 1U;
    }

    /* 达到稳定次数后，认为起点目标数字已经确认 */
    if (candidate_count >= VISION_STABLE_COUNT) {
        stable_digit = room;
        stable_ready = 1U;
    }
}

static void Vision_AcceptFrame(uint8_t room, uint8_t side, uint8_t finded_flag, uint8_t find_task, uint8_t confidence)
{
    /* 保存最近一帧视觉数据，供状态机和 OLED 查询 */
    vision_num = room;
    vision_lor = side;
    vision_finded_flag = finded_flag;
    vision_find_task = find_task;
    vision_confidence = confidence;
    vision_packet_count++;

    /* 只有识别有效、数字合法、左右合法、置信度足够时，才更新稳定数字 */
    if (finded_flag == 1U && Vision_IsValidDigit(room) && Vision_IsValidSide(side) && confidence >= VISION_MIN_CONFIDENCE) {
        Vision_UpdateStable(room);
    } else {
        /* 本帧无效时清除候选连续计数 */
        last_candidate = 0U;
        candidate_count = 0U;
    }
}

void Vision_Init(void)
{
    /* 串口解析状态机从空闲状态开始 */
    rx_state = 0U;

    /* 清空起点稳定数字 */
    Vision_ResetStableDigit();
}

void Vision_OnRxByte(uint8_t byte)
{
    /* 保存最近收到的字节，并累计接收字节数 */
    vision_last_byte = byte;
    vision_rx_byte_count++;

    /* 按字节解析 K230 AA55 协议和 OpenMV 兼容协议 */
    switch (rx_state) {
    case 0:
        /* 空闲状态下等待两种协议的帧头 */
        if (byte == K230_FRAME_HEAD) {
            rx_state = 1U;
        } else if (byte == OPENMV_FRAME_HEAD1) {
            rx_state = 10U;
        }
        break;

    case 1:
        /* K230 协议第 2 字节：房间号 */
        rx_room = byte;
        rx_state = 2U;
        break;
    case 2:
        /* K230 协议第 3 字节：目标左右位置 */
        rx_side = byte;
        rx_state = 3U;
        break;
    case 3:
        /* K230 协议第 4 字节：置信度 */
        rx_confidence = byte;
        rx_state = 4U;
        break;
    case 4:
        /* K230 协议第 5 字节：校验和 */
        rx_checksum = byte;
        rx_state = 5U;
        break;
    case 5:
        /* K230 协议最后必须收到帧尾 0x55 */
        if (byte == K230_FRAME_TAIL) {
            uint8_t expected = (uint8_t)((K230_FRAME_HEAD + rx_room + rx_side + rx_confidence) & 0xFFU);
            if (rx_checksum == expected) {
                Vision_AcceptFrame(rx_room, rx_side, 1U, 0U, rx_confidence);
            } else {
                vision_checksum_error_count++;
            }
        }
        rx_state = 0U;
        break;

    case 10:
        /* OpenMV 协议需要连续收到 0x2C、0x12 两个帧头 */
        rx_state = (byte == OPENMV_FRAME_HEAD2) ? 11U : 0U;
        break;
    case 11:
        /* OpenMV 协议第 3 字节：房间号 */
        rx_room = byte;
        rx_state = 12U;
        break;
    case 12:
        /* OpenMV 协议第 4 字节：目标左右位置 */
        rx_side = byte;
        rx_state = 13U;
        break;
    case 13:
        /* OpenMV 协议第 5 字节：识别有效标志 */
        rx_find_flag = byte;
        rx_state = 14U;
        break;
    case 14:
        /* OpenMV 协议第 6 字节：当前视觉任务号 */
        rx_find_task = byte;
        rx_state = 15U;
        break;
    case 15:
        /* OpenMV 协议最后必须收到帧尾 0x5B */
        if (byte == OPENMV_FRAME_TAIL) {
            Vision_AcceptFrame(rx_room, rx_side, rx_find_flag, rx_find_task, 100U);
        }
        rx_state = 0U;
        break;

    default:
        /* 任何异常状态都回到空闲，等待下一帧 */
        rx_state = 0U;
        break;
    }
}

void Vision_SetTask(uint8_t task, uint8_t target_num)
{
    /* 保存需要周期发送给 K230 的任务编号和目标数字 */
    task_to_send = task;
    target_to_send = target_num;
}

void Vision_SendTask(uint8_t task, uint8_t target_num)
{
    uint8_t send_buf[4];

    /* 发送给 K230 的任务格式：* 任务数字 目标数字 & */
    send_buf[0] = '*';
    send_buf[1] = (uint8_t)('0' + (task % 10U));
    send_buf[2] = (uint8_t)('0' + (target_num % 10U));
    send_buf[3] = '&';

    /* 通过 USART3 发给 K230 */
    HAL_UART_Transmit(&huart3, send_buf, sizeof(send_buf), 10);
}

void Vision_Loop10ms(void)
{
    /* 每隔固定周期向 K230 重发当前任务，防止 K230 漏收 */
    if (++send_tick >= VISION_SEND_PERIOD_TICKS) {
        send_tick = 0U;
        Vision_SendTask(task_to_send, target_to_send);
    }
}

void Vision_ResetStableDigit(void)
{
    /* 清空稳定目标数字和候选计数 */
    stable_digit = 0U;
    stable_ready = 0U;
    last_candidate = 0U;
    candidate_count = 0U;
}

uint8_t Vision_HasStableDigit(void)
{
    return stable_ready;
}

uint8_t Vision_GetStableDigit(void)
{
    return stable_digit;
}

uint8_t Vision_GetDigit(void)
{
    return vision_num;
}

uint8_t Vision_GetLoR(void)
{
    return vision_lor;
}

uint8_t Vision_GetFindedFlag(void)
{
    return vision_finded_flag;
}

uint8_t Vision_GetFindTask(void)
{
    return vision_find_task;
}

uint8_t Vision_GetConfidence(void)
{
    return vision_confidence;
}

uint8_t Vision_GetLastByte(void)
{
    return vision_last_byte;
}

uint8_t Vision_TargetMatched(void)
{
    /* 当前视觉帧必须有效，且识别数字等于 STM32 下发的目标数字 */
    return vision_finded_flag == 1U && vision_num == target_to_send && Vision_IsValidDigit(vision_num);
}

uint32_t Vision_GetRxByteCount(void)
{
    return vision_rx_byte_count;
}

uint32_t Vision_GetPacketCount(void)
{
    return vision_packet_count;
}

uint32_t Vision_GetChecksumErrorCount(void)
{
    return vision_checksum_error_count;
}
