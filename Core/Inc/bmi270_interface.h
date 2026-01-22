/*
 * bmi270_interface.h
 * 用于适配 STM32 HAL 库与 Bosch BMI270 官方驱动
 */

#ifndef BMI270_INTERFACE_H_
#define BMI270_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"   // 包含 HAL 库定义
#include "bmi270.h" // 包含 Bosch 官方驱动定义

    /* 对外暴露设备结构体，以便在 main.c 中调用 api 获取数据 */
    extern struct bmi2_dev bmi270_dev;

    /*
     * @brief  初始化 BMI270 传感器
     * @return 0: 成功, 其他值: 失败 (参考 BMI2_E_xxx 错误码)
     */
    int8_t BMI270_User_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* BMI270_INTERFACE_H_ */