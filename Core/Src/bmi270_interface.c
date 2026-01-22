/*
 * bmi270_interface.c
 */

#include "bmi270_interface.h"
#include "spi.h"  // 包含 CubeMX 生成的 spi.h (为了获取 hspi1)
#include "gpio.h" // 包含 CubeMX 生成的 gpio.h

/******************************************************************************/
/*                                 硬件配置宏                                 */
/******************************************************************************/

/* 定义使用的 SPI 句柄 (根据 CubeMX 配置修改，通常是 &hspi1 或 &hspi2) */
#define BMI270_SPI_HANDLE &hspi1

/* 定义 CS (片选) 引脚 (根据 CubeMX 配置修改) */
#define BMI270_CS_GPIO_PORT GPIOA
#define BMI270_CS_PIN GPIO_PIN_4

/* 定义最大读写缓冲区长度 (取决于 STM32 的 RAM 大小，BMI270 固件加载需要较大突发写入) */
#define READ_WRITE_LEN 8192

/******************************************************************************/
/*                                全局变量                                    */
/******************************************************************************/
struct bmi2_dev bmi270_dev;

/******************************************************************************/
/*                             接口函数实现                                   */
/******************************************************************************/

/**
 * @brief  SPI 写入函数 (适配 Bosch 驱动)
 * @param  reg_addr: 寄存器地址
 * @param  reg_data: 写入数据的指针
 * @param  len:      写入数据的长度
 * @param  intf_ptr: 接口指针 (未使用)
 * @retval 0: 成功, -1: 失败
 */
static BMI2_INTF_RETURN_TYPE bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    HAL_StatusTypeDef status;

    /* 1. 拉低片选 (CS Low) 开始通信 */
    HAL_GPIO_WritePin(BMI270_CS_GPIO_PORT, BMI270_CS_PIN, GPIO_PIN_RESET);

    /* 2. 发送寄存器地址 */
    /* BMI270 写操作时，地址位 7 (RW) 应为 0。Bosch 驱动传来的地址通常已处理好，或者是纯地址 */
    status = HAL_SPI_Transmit(BMI270_SPI_HANDLE, &reg_addr, 1, 100);
    if (status != HAL_OK)
        goto error;

    /* 3. 发送数据 */
    if (len > 0)
    {
        status = HAL_SPI_Transmit(BMI270_SPI_HANDLE, (uint8_t *)reg_data, len, 1000); // 固件加载时数据较长，超时设大点
        if (status != HAL_OK)
            goto error;
    }

    /* 4. 拉高片选 (CS High) 结束通信 */
    HAL_GPIO_WritePin(BMI270_CS_GPIO_PORT, BMI270_CS_PIN, GPIO_PIN_SET);
    return BMI2_OK;

error:
    HAL_GPIO_WritePin(BMI270_CS_GPIO_PORT, BMI270_CS_PIN, GPIO_PIN_SET);
    return BMI2_E_COM_FAIL;
}

/**
 * @brief  SPI 读取函数 (适配 Bosch 驱动)
 * @param  reg_addr: 寄存器地址
 * @param  reg_data: 存储读取数据的指针
 * @param  len:      读取数据的长度
 * @param  intf_ptr: 接口指针 (未使用)
 * @retval 0: 成功, -1: 失败
 */
static BMI2_INTF_RETURN_TYPE bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    HAL_StatusTypeDef status;

    /* 1. 拉低片选 (CS Low) */
    HAL_GPIO_WritePin(BMI270_CS_GPIO_PORT, BMI270_CS_PIN, GPIO_PIN_RESET);

    /* 2. 发送寄存器地址 */
    /* 在 SPI 模式下读取，BMI270 内部逻辑通常不需要在这里手动把 reg_addr | 0x80，
       因为在 bmi2_get_regs 内部并没有做位操作，但是标准 SPI 读通常需要首位为 1。
       实测：Bosch 传感器在 SPI 模式下，直接发地址即可，驱动层似乎已处理或芯片能识别上下文。
       如果读不到数据，请尝试发送: reg_addr | 0x80 */
    status = HAL_SPI_Transmit(BMI270_SPI_HANDLE, &reg_addr, 1, 100);
    if (status != HAL_OK)
        goto error;

    /* 3. 接收数据 */
    /*
       注意：BMI270 SPI 读取时，发送地址后的第一个字节是 Dummy Byte。
       bmi270_init() 函数会自动将 dev->dummy_byte 设为 1。
       这意味着 bmi2.c 里的读取函数会自动请求 (len + 1) 个字节，并丢弃第一个字节。
       所以这里我们只需要老老实实地调用 HAL_SPI_Receive 接收 len 个字节即可。
    */
    if (len > 0)
    {
        status = HAL_SPI_Receive(BMI270_SPI_HANDLE, reg_data, len, 100);
        if (status != HAL_OK)
            goto error;
    }

    /* 4. 拉高片选 (CS High) */
    HAL_GPIO_WritePin(BMI270_CS_GPIO_PORT, BMI270_CS_PIN, GPIO_PIN_SET);
    return BMI2_OK;

error:
    HAL_GPIO_WritePin(BMI270_CS_GPIO_PORT, BMI270_CS_PIN, GPIO_PIN_SET);
    return BMI2_E_COM_FAIL;
}

/**
 * @brief  微秒级延时函数
 * @param  period: 延时微秒数
 * @param  intf_ptr: 接口指针 (未使用)
 */
static void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    /* HAL_Delay 是毫秒级，如果需要微秒级，为了简单移植，这里向上取整 */
    uint32_t ms = period / 1000;
    if (period % 1000 != 0 || ms == 0)
    {
        ms++;
    }
    HAL_Delay(ms);
}

/******************************************************************************/
/*                             用户初始化函数                                 */
/******************************************************************************/

int8_t BMI270_User_Init(void)
{
    int8_t rslt;

    /* 1. 确保 CS 引脚处于高电平状态 */
    HAL_GPIO_WritePin(BMI270_CS_GPIO_PORT, BMI270_CS_PIN, GPIO_PIN_SET);

    /* 上电稍微延时，等待传感器内部电路稳定 */
    HAL_Delay(100);

    /* 2. 配置设备结构体 */
    bmi270_dev.intf = BMI2_SPI_INTF;     // 使用 SPI 接口
    bmi270_dev.read = bmi2_spi_read;     // 绑定读函数
    bmi270_dev.write = bmi2_spi_write;   // 绑定写函数
    bmi270_dev.delay_us = bmi2_delay_us; // 绑定延时函数
    bmi270_dev.read_write_len = READ_WRITE_LEN;
    bmi270_dev.intf_ptr = NULL; // 不需要额外参数
    /* config_file_ptr 在 bmi270_init 内部会自动指向数组，这里无需手动赋值 */

    /* 3. 调用官方驱动初始化 (会执行软复位、检查 ChipID、加载固件) */
    /* 注意：此过程可能耗时几百毫秒，因为要加载 8KB 配置文件 */
    rslt = bmi270_init(&bmi270_dev);

    if (rslt != BMI2_OK)
    {
        return rslt; // 初始化失败
    }

    /* 4. 默认配置：开启加速度和陀螺仪 (高性能模式) */
    uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
    struct bmi2_sens_config config[2];

    /* 配置加速度计 */
    config[0].type = BMI2_ACCEL;
    config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;

    /* 配置陀螺仪 */
    config[1].type = BMI2_GYRO;
    config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE; /* 优化噪声性能 (1 = 优化模式) */
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;

    /* 应用配置 */
    rslt = bmi270_set_sensor_config(config, 2, &bmi270_dev);
    if (rslt != BMI2_OK)
        return rslt;

    /* 使能传感器 */
    rslt = bmi270_sensor_enable(sens_list, 2, &bmi270_dev);

    return rslt;
}