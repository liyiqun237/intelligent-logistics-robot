#include "bsp.h"

void bsp_Init(void)
{
    Emm_V5_Init();
    
}

void SendMultiFloat2Vofa(float *values, uint8_t num)
{
    uint8_t buffer[4 * num + 4];

    // 复制所有浮点数据到缓冲区
    for (uint8_t i = 0; i < num; i++)
    {
        memcpy(buffer + 4 * i, &values[i], 4);
    }

    // FireWater协议的帧尾
    buffer[4 * num] = 0x00;
    buffer[4 * num + 1] = 0x00;
    buffer[4 * num + 2] = 0x80;
    buffer[4 * num + 3] = 0x7f;

    // 通过UART3发送
    HAL_UART_Transmit(&huart3, buffer, 4 * num + 4, 0xFFFF);
}