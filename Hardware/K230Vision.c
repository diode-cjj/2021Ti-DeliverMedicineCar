#include "K230Vision.h"
#include "Serial.h"
#include "usart.h"
#include <string.h>

/* K230 小球协议：AA find cxH cxL cyH cyL wH wL hH hL dH dL checksum 55 */
#define K230VISION_FRAME_HEAD 0xAAU
#define K230VISION_FRAME_TAIL 0x55U

/* K230 接在 USART3 上 */
extern UART_HandleTypeDef huart3;

/* 串口接收状态机临时变量 */
static uint8_t rx_state;
static uint8_t rx_find;
static uint8_t rx_cx_h;
static uint8_t rx_cx_l;
static uint8_t rx_cy_h;
static uint8_t rx_cy_l;
static uint8_t rx_width_h;
static uint8_t rx_width_l;
static uint8_t rx_height_h;
static uint8_t rx_height_l;
static uint8_t rx_distance_h;
static uint8_t rx_distance_l;
static uint8_t rx_checksum;
static uint8_t rx_sum;

/* 最近一帧 K230 小球数据 */
static volatile K230Vision_Target_t k230_target;
static volatile uint32_t k230_last_tick;

static uint16_t K230Vision_MakeU16(uint8_t high, uint8_t low)
{
    return ((uint16_t)high << 8) | low;
}

static void K230Vision_AcceptFrame(void)
{
    k230_target.found = (rx_find != 0U) ? 1U : 0U;
    k230_target.cx = (int16_t)K230Vision_MakeU16(rx_cx_h, rx_cx_l);
    k230_target.cy = (int16_t)K230Vision_MakeU16(rx_cy_h, rx_cy_l);
    k230_target.width = (int16_t)K230Vision_MakeU16(rx_width_h, rx_width_l);
    k230_target.height = (int16_t)K230Vision_MakeU16(rx_height_h, rx_height_l);
    k230_target.distance_cm = K230Vision_MakeU16(rx_distance_h, rx_distance_l);
    k230_last_tick = HAL_GetTick();
    k230_target.tick = k230_last_tick;
}

void K230Vision_Init(void)
{
    K230Vision_Clear();
    Serial_SetRxCallback(&huart3, K230Vision_OnRxByte);
}

void K230Vision_Clear(void)
{
    rx_state = 0U;
    rx_sum = 0U;
    k230_last_tick = 0U;
    memset((void *)&k230_target, 0, sizeof(k230_target));
}

void K230Vision_OnRxByte(uint8_t byte)
{
    /* 按字节解析 K230 AA55 二进制协议 */
    switch (rx_state) {
    case 0:
        if (byte == K230VISION_FRAME_HEAD) {
            rx_sum = byte;
            rx_state = 1U;
        }
        break;

    case 1:
        rx_find = byte;
        rx_sum += byte;
        rx_state = 2U;
        break;
    case 2:
        rx_cx_h = byte;
        rx_sum += byte;
        rx_state = 3U;
        break;
    case 3:
        rx_cx_l = byte;
        rx_sum += byte;
        rx_state = 4U;
        break;
    case 4:
        rx_cy_h = byte;
        rx_sum += byte;
        rx_state = 5U;
        break;
    case 5:
        rx_cy_l = byte;
        rx_sum += byte;
        rx_state = 6U;
        break;
    case 6:
        rx_width_h = byte;
        rx_sum += byte;
        rx_state = 7U;
        break;
    case 7:
        rx_width_l = byte;
        rx_sum += byte;
        rx_state = 8U;
        break;
    case 8:
        rx_height_h = byte;
        rx_sum += byte;
        rx_state = 9U;
        break;
    case 9:
        rx_height_l = byte;
        rx_sum += byte;
        rx_state = 10U;
        break;
    case 10:
        rx_distance_h = byte;
        rx_sum += byte;
        rx_state = 11U;
        break;
    case 11:
        rx_distance_l = byte;
        rx_sum += byte;
        rx_state = 12U;
        break;
    case 12:
        rx_checksum = byte;
        rx_state = 13U;
        break;
    case 13:
        if (byte == K230VISION_FRAME_TAIL && rx_checksum == rx_sum) {
            K230Vision_AcceptFrame();
        }
        rx_state = 0U;
        break;

    default:
        rx_state = 0U;
        break;
    }
}

void K230Vision_Update(void)
{
}

uint8_t K230Vision_GetTarget(K230Vision_Target_t *target)
{
    uint8_t found;

    __disable_irq();
    found = k230_target.found;
    if (target != 0) {
        *target = k230_target;
    }
    __enable_irq();

    return found;
}

uint8_t K230Vision_IsOnline(uint32_t timeout_ms)
{
    if (k230_last_tick == 0U) {
        return 0U;
    }

    if (HAL_GetTick() - k230_last_tick > timeout_ms) {
        return 0U;
    }

    return 1U;
}
