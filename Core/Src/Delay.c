#include "delay.h"
#include "stm32f4xx_hal.h"
//初始化延迟函数
//当使用OS的时候,此函数会初始化OS的时钟节拍
//SYSTICK的时钟需要通过时钟树配置，可选为HCLK系统时钟，也可8分频HCLK_DIV8系统时钟
//SYSCLK:系统时钟频率HCLK
 
 uint32_t multiplier;
						//us延时倍乘
 
void delay_init(void)
{
 multiplier = HAL_RCC_GetHCLKFreq() / 16000000; 
}
 
//延时nus
//nus为要延时的us数.	
//nus:0~190887435(最大值即2^32/fac_us@fac_us=22.5)	 
void delay_us(uint32_t micros)
{		

    /* multiply micro with multipliter */
    micros = multiplier * micros - 10;
    /* 4 cycles for one loop */
    while(micros--);
}
 
//延时nms
//nms:要延时的ms数
void delay_ms(uint16_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) delay_us(1000);
}