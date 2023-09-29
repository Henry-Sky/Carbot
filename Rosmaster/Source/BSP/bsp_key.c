#include "bsp_key.h"
#include "bsp.h"

uint16_t g_key1_long_press = 0;

// �жϰ����Ƿ񱻰��£����·���KEY_PRESS���ɿ�����KEY_RELEASE
static uint8_t Key1_is_Press(void)
{
	if (!GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN))
	{
		return KEY_PRESS; // ������������£��򷵻�KEY_PRESS
	}
	return KEY_RELEASE;   // ����������ɿ�״̬���򷵻�KEY_RELEASE
}


// ����ʽɨ�谴�������·���1���ɿ�����0
uint8_t Key_Scan(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	/*����Ƿ��а������� */
	if (!GPIO_ReadInputDataBit(GPIOx, GPIO_Pin))
	{
		/*�ȴ������ͷ� */
		while (!GPIO_ReadInputDataBit(GPIOx, GPIO_Pin))
			;
		return KEY_PRESS;
	}
	else
		return KEY_RELEASE;
}

// ��ȡ����K1�ĳ���״̬���ۼƴﵽ����ʱ�䷵��1��δ�ﵽ����0.
// timeoutΪ����ʱ�䳤�ȣ���λΪ��
uint8_t Key1_Long_Press(uint16_t timeout)
{
	if (g_key1_long_press > 0)
	{
		if (g_key1_long_press < timeout * 100 + 2)
		{
			g_key1_long_press++;
			if (g_key1_long_press == timeout * 100 + 2)
			{
				DEBUG("key2 long press\n");
				return 1;
			}
			return 0;
		}
	}
	return 0;
}

// ��ȡ����K1��״̬�����·���1���ɿ�����0.
// mode:����ģʽ��0������һֱ����1��1������ֻ����һ��1
uint8_t Key1_State(uint8_t mode)
{
	static uint16_t key1_state = 0;

	if (Key1_is_Press() == KEY_PRESS)
	{
		if (key1_state < (mode + 1) * 2)
		{
			key1_state++;
		}
	}
	else
	{
		key1_state = 0;
		g_key1_long_press = 0;
	}
	if (key1_state == 2)
	{
		g_key1_long_press = 1;
		return KEY_PRESS;
	}
	return KEY_RELEASE;
}

// ��ʼ������
void Key_GPIO_Init(void)
{
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
	/*���������˿ڵ�ʱ��*/
	RCC_APB2PeriphClockCmd(KEY1_GPIO_CLK, ENABLE);
	//ѡ�񰴼�������
	GPIO_InitStructure.GPIO_Pin = KEY1_GPIO_PIN;
	//���ð���������Ϊ��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	//ʹ�ýṹ���ʼ������
	GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);
}


/*********************************************END OF FILE**********************/
