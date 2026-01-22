#include "myoled.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

uint8_t oled_address = 0x78; // OLED I2C地址

void Send_Cmd(uint8_t cmd)
{
    uint8_t sendbuff[2];
    sendbuff[0] = 0x00; // 命令标志
    sendbuff[1] = cmd;
    HAL_I2C_Master_Transmit(&hi2c1, oled_address, sendbuff, 2, HAL_MAX_DELAY);
}

void Send_Data(uint8_t data)
{
    uint8_t sendbuff[2];
    sendbuff[0] = 0x40; // 数据标志
    sendbuff[1] = data;
    HAL_I2C_Master_Transmit(&hi2c1, oled_address, sendbuff, 2, HAL_MAX_DELAY);
}

bool Oled_Init(void)
{
    Send_Cmd(0xAE); // 关闭显示

    Send_Cmd(0xD5); // 设置显示时钟分频比/振荡器频率
    Send_Cmd(0x80);
    HAL_UART_Transmit(&huart1, (uint8_t *)"OLED Init Step 1\r\n", 19, 1000);

    Send_Cmd(0xA8); // 设置多路复用率
    Send_Cmd(0x3F);

    Send_Cmd(0xD3); // 设置显示偏移
    Send_Cmd(0x00);

    Send_Cmd(0x40); // 设置显示开始行

    Send_Cmd(0xA1); // 设置左右方向，0xA1正常 0xA0左右反置
    HAL_UART_Transmit(&huart1, (uint8_t *)"OLED Init Step 2\r\n", 19, 1000);

    Send_Cmd(0xC8); // 设置上下方向，0xC8正常 0xC0上下反置

    Send_Cmd(0xDA); // 设置COM引脚硬件配置
    Send_Cmd(0x12);

    Send_Cmd(0x81); // 设置对比度控制
    Send_Cmd(0xCF);

    Send_Cmd(0xD9); // 设置预充电周期
    Send_Cmd(0xF1);

    Send_Cmd(0xDB); // 设置VCOMH取消选择级别
    Send_Cmd(0x30);
    HAL_UART_Transmit(&huart1, (uint8_t *)"OLED Init Step 3\r\n", 19, 1000);
    Send_Cmd(0xA4); // 设置整个显示打开/关闭

    Send_Cmd(0xA6); // 设置正常/倒转显示

    Send_Cmd(0x8D); // 设置充电泵
    Send_Cmd(0x14);

    Send_Cmd(0xAF); // 开启显示
    HAL_UART_Transmit(&huart1, (uint8_t *)"OLED Init completed\r\n", 22, 1000);
    return true;
}

void Oled_Clear(void)
{
    for (uint8_t page = 0; page < 8; page++)
    {
        Send_Cmd(0xB0 + page); // 设置页地址
        Send_Cmd(0x00);        // 设置列地址低4位
        Send_Cmd(0x10);        // 设置列地址高4位
        for (uint8_t col = 0; col < 128; col++)
        {
            Send_Data(0x00); // 清除数据
        }
    }
}

void Oled_Test(void)
{
    for (uint8_t page = 0; page < 8; page++)
    {
        Send_Cmd(0xB0 + page); // 设置页地址
        Send_Cmd(0x00);        // 设置列地址低4位
        Send_Cmd(0x10);        // 设置列地址高4位
        for (uint8_t col = 0; col < 128; col++)
        {
            Send_Data(0xFF); // 测试数据
        }
    }
}
