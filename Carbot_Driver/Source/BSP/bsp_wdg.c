#include "bsp_wdg.h"
#include "bsp.h"


// ���Ź���ʼ��, ���Ƶ��Ϊ64,����ֵΪ625,���ʱ��Ϊ1s	
void IWDG_Init(void)
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����
	
	IWDG_SetPrescaler(4);  //����IWDGԤ��Ƶֵ:����IWDGԤ��ƵֵΪ64
	
	IWDG_SetReload(625);  //����IWDG��װ��ֵ
	
	IWDG_ReloadCounter();  //����IWDG��װ�ؼĴ�����ֵ��װ��IWDG������
	
	IWDG_Enable();  //ʹ��IWDG
}

//ι�������Ź�
void IWDG_Feed(void)
{   
 	IWDG_ReloadCounter();//reload										   
}


