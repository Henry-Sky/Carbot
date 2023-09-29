#include "bsp_timer.h"
#include "bsp_pwmServo.h"


/**************************************************************************
�������ܣ�TIM6��ʼ������ʱ10����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM6_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //ʹ�ܶ�ʱ����ʱ��
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;			 // Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_Period = 99;				 //�趨�������Զ���װֵ
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);                //���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;			  //TIM1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);							  //��ʼ��NVIC�Ĵ���

	TIM_Cmd(TIM6, ENABLE);
}


/**************************************************************************
�������ܣ�TIM7��ʼ������ʱ10us��ģ��PWM�����ƶ�������
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM7_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //ʹ�ܶ�ʱ����ʱ��
	TIM_TimeBaseStructure.TIM_Prescaler = 71;			 // Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_Period = 9;				 //�趨�������Զ���װֵ
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);               //���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;			  //TIM�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  //�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);							  //��ʼ��NVIC�Ĵ���

	TIM_Cmd(TIM7, ENABLE);
}


// TIM7�ж�
void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) //���TIM�����жϷ������
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);    //���TIMx�����жϱ�־
		PwmServo_Handle();
	}
}
