#include "bmp280.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "main.h"

/*bmp280 气压和温度过采样 工作模式*/
#define BMP280_PRESSURE_OSR (BMP280_OVERSAMP_8X)
#define BMP280_TEMPERATURE_OSR (BMP280_OVERSAMP_16X)
#define BMP280_MODE (BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_NORMAL_MODE)

typedef struct
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    int32_t t_fine;
} bmp280Calib;

static bmp280Calib bmp280Cal;
static uint8_t bmp280ID = 0;
static bool isInit = false;
static int32_t bmp280RawPressure = 0;
static int32_t bmp280RawTemperature = 0;

// 保存I2C句柄
static I2C_HandleTypeDef *bmp280_i2c_handle;

// 内部函数声明
static void bmp280GetPressure(void);
static void presssureFilter(float *in, float *out);
static float bmp280PressureToAltitude(float *pressure);
static int32_t bmp280CompensateT(int32_t adcT);
static uint32_t bmp280CompensateP(int32_t adcP);

//=============================================================================
// HAL库 I2C 封装函数
//=============================================================================

// 从指定寄存器读取一个字节
static uint8_t iicDevReadByte(uint8_t reg_addr)
{
    uint8_t data = 0;
    // HAL_I2C_Mem_Read(句柄, 设备地址, 寄存器地址, 寄存器地址长度, 数据缓存, 数据长度, 超时时间)
    HAL_I2C_Mem_Read(bmp280_i2c_handle, BMP280_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return data;
}

// 连续读取多个字节
static void iicDevRead(uint8_t reg_addr, uint8_t len, uint8_t *rbuf)
{
    HAL_I2C_Mem_Read(bmp280_i2c_handle, BMP280_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, rbuf, len, 100);
}

// 向指定寄存器写入一个字节
static void iicDevWriteByte(uint8_t reg_addr, uint8_t data)
{
    HAL_I2C_Mem_Write(bmp280_i2c_handle, BMP280_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

//=============================================================================
// BMP280 逻辑函数
//=============================================================================

/**
 * @brief 初始化BMP280
 * @param hi2c I2C句柄指针
 */
bool BMP280_Init(I2C_HandleTypeDef *hi2c)
{
    if (isInit)
        return true;

    bmp280_i2c_handle = hi2c; // 保存句柄供内部使用

    HAL_Delay(20); // 延时等待传感器上电稳定

    bmp280ID = iicDevReadByte(BMP280_CHIP_ID); /* 读取bmp280 ID*/

    if (bmp280ID == BMP280_DEFAULT_CHIP_ID)
    {
        // printf("BMP280 ID IS: 0x%X\n", bmp280ID);
    }
    else
    {
        return false;
    }

    /* 读取校准数据 */
    iicDevRead(BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, 24, (uint8_t *)&bmp280Cal);
    iicDevWriteByte(BMP280_CTRL_MEAS_REG, BMP280_MODE);
    iicDevWriteByte(BMP280_CONFIG_REG, 5 << 2); /*配置IIR滤波*/

    isInit = true;
    return true;
}

static void bmp280GetPressure(void)
{
    uint8_t data[BMP280_DATA_FRAME_SIZE];

    // read data from sensor
    iicDevRead(BMP280_PRESSURE_MSB_REG, BMP280_DATA_FRAME_SIZE, data);

    bmp280RawPressure = (int32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
    bmp280RawTemperature = (int32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
static int32_t bmp280CompensateT(int32_t adcT)
{
    int32_t var1, var2, T;

    var1 = ((((adcT >> 3) - ((int32_t)bmp280Cal.dig_T1 << 1))) * ((int32_t)bmp280Cal.dig_T2)) >> 11;
    var2 = (((((adcT >> 4) - ((int32_t)bmp280Cal.dig_T1)) * ((adcT >> 4) - ((int32_t)bmp280Cal.dig_T1))) >> 12) * ((int32_t)bmp280Cal.dig_T3)) >> 14;
    bmp280Cal.t_fine = var1 + var2;

    T = (bmp280Cal.t_fine * 5 + 128) >> 8;

    return T;
}

static uint32_t bmp280CompensateP(int32_t adcP)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280Cal.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280Cal.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280Cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280Cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280Cal.dig_P3) >> 8) + ((var1 * (int64_t)bmp280Cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280Cal.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - adcP;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280Cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280Cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280Cal.dig_P7) << 4);
    return (uint32_t)p;
}

#define FILTER_NUM 5
#define FILTER_A 0.1f

/*限幅平均滤波法*/
static void presssureFilter(float *in, float *out)
{
    static uint8_t i = 0;
    static float filter_buf[FILTER_NUM] = {0.0};
    double filter_sum = 0.0;
    uint8_t cnt = 0;
    float deta;

    if (filter_buf[i] == 0.0f)
    {
        filter_buf[i] = *in;
        *out = *in;
        if (++i >= FILTER_NUM)
            i = 0;
    }
    else
    {
        if (i)
            deta = *in - filter_buf[i - 1];
        else
            deta = *in - filter_buf[FILTER_NUM - 1];

        if (fabs(deta) < FILTER_A)
        {
            filter_buf[i] = *in;
            if (++i >= FILTER_NUM)
                i = 0;
        }
        for (cnt = 0; cnt < FILTER_NUM; cnt++)
        {
            filter_sum += filter_buf[cnt];
        }
        *out = filter_sum / FILTER_NUM;
    }
}

#define CONST_PF 0.1902630958
#define FIX_TEMP 25

static float bmp280PressureToAltitude(float *pressure)
{
    if (*pressure > 0)
    {
        return ((pow((1015.7f / *pressure), CONST_PF) - 1.0f) * (FIX_TEMP + 273.15f)) / 0.0065f;
    }
    else
    {
        return 0;
    }
}

void BMP280_GetData(float *pressure, float *temperature, float *asl)
{
    static float t;
    static float p;

    bmp280GetPressure();

    t = bmp280CompensateT(bmp280RawTemperature) / 100.0;
    p = bmp280CompensateP(bmp280RawPressure) / 25600.0;

    presssureFilter(&p, pressure);
    *temperature = (float)t;
    *pressure = (float)p;

    *asl = bmp280PressureToAltitude(pressure);
}