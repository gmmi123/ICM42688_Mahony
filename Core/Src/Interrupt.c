#include "interrupt.h"

int Tis=0;
float Roll,Pitch,Yaw;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim3.Instance) 
	{	
		if(Tis>20000)
		{
			Tis=0;
		}
		Tis++;
		GetAngle(&Roll,&Pitch,&Yaw);
	}
}

