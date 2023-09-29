#include "bsp_wdg.h"
#include "bsp.h"


// 看门狗初始化, 与分频数为64,重载值为625,溢出时间为1s	
void IWDG_Init(void)
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //使能对寄存器IWDG_PR和IWDG_RLR的写操作
	
	IWDG_SetPrescaler(4);  //设置IWDG预分频值:设置IWDG预分频值为64
	
	IWDG_SetReload(625);  //设置IWDG重装载值
	
	IWDG_ReloadCounter();  //按照IWDG重装载寄存器的值重装载IWDG计数器
	
	IWDG_Enable();  //使能IWDG
}

//喂独立看门狗
void IWDG_Feed(void)
{   
 	IWDG_ReloadCounter();//reload										   
}


