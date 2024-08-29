#include "delay.h"
#include "stm32f4xx_hal.h"
//��ʼ���ӳٺ���
//��ʹ��OS��ʱ��,�˺������ʼ��OS��ʱ�ӽ���
//SYSTICK��ʱ����Ҫͨ��ʱ�������ã���ѡΪHCLKϵͳʱ�ӣ�Ҳ��8��ƵHCLK_DIV8ϵͳʱ��
//SYSCLK:ϵͳʱ��Ƶ��HCLK
 
 uint32_t multiplier;
						//us��ʱ����
 
void delay_init(void)
{
 multiplier = HAL_RCC_GetHCLKFreq() / 16000000; 
}
 
//��ʱnus
//nusΪҪ��ʱ��us��.	
//nus:0~190887435(���ֵ��2^32/fac_us@fac_us=22.5)	 
void delay_us(uint32_t micros)
{		

    /* multiply micro with multipliter */
    micros = multiplier * micros - 10;
    /* 4 cycles for one loop */
    while(micros--);
}
 
//��ʱnms
//nms:Ҫ��ʱ��ms��
void delay_ms(uint16_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) delay_us(1000);
}