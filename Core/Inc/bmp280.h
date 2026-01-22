#ifndef __BMP280_H
#define __BMP280_H

#include "main.h" // 包含HAL库定义
#include <math.h>
#include <stdbool.h>
#include <stdint.h> // 引入标准整数类型

//=============================================================================================================
// BMP280 I2C地址 (8位地址: 7位地址 << 1)
// SD0 接 GND -> 0x76 << 1 = 0xEC
// SD0 接 VDD -> 0x77 << 1 = 0xEE
#define BMP280_ADDR (0xEC)
//=============================================================================================================

#define BMP280_DEFAULT_CHIP_ID (0x58)
#define BMP280_CHIP_ID (0xD0)              /* Chip ID Register */
#define BMP280_RST_REG (0xE0)              /* Softreset Register */
#define BMP280_STAT_REG (0xF3)             /* Status Register */
#define BMP280_CTRL_MEAS_REG (0xF4)        /* Ctrl Measure Register */
#define BMP280_CONFIG_REG (0xF5)           /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG (0xF7)     /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG (0xF8)     /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG (0xF9)    /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG (0xFA)  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG (0xFB)  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG (0xFC) /* Temperature XLSB Reg */

#define BMP280_SLEEP_MODE (0x00)
#define BMP280_FORCED_MODE (0x01)
#define BMP280_NORMAL_MODE (0x03)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH (24)
#define BMP280_DATA_FRAME_SIZE (6)

#define BMP280_OVERSAMP_SKIPPED (0x00)
#define BMP280_OVERSAMP_1X (0x01)
#define BMP280_OVERSAMP_2X (0x02)
#define BMP280_OVERSAMP_4X (0x03)
#define BMP280_OVERSAMP_8X (0x04)
#define BMP280_OVERSAMP_16X (0x05)

/* 函数声明 */

/**
 * @brief  初始化BMP280
 * @param  hi2c: I2C句柄指针 (例如 &hi2c1)
 * @return true: 初始化成功, false: 初始化失败(ID不匹配)
 */
bool BMP280_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  获取传感器数据
 * @param  pressure: 气压 (hPa)
 * @param  temperature: 温度 (DegC)
 * @param  asl: 海拔 (m)
 */
void BMP280_GetData(float *pressure, float *temperature, float *asl);

#endif