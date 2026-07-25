#ifndef _BSP_H
#define _BSP_H
// #include "board.h"
#include "delay.h"
#include "Emm_V5.h"
#include "fifo.h"



void bsp_Init(void);
void SendMultiFloat2Vofa(float *values, uint8_t num);



#endif