#ifndef __MYOLED_H
#define __MYOLED_H

#include "main.h"
#include <stdlib.h>
#include <stdbool.h>

void Send_Cmd(uint8_t cmd);
void Send_Data(uint8_t data);
bool Oled_Init(void);
void Oled_Test(void);

#endif /* __MYOLED_H */