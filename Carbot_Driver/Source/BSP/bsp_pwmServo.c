#include "bsp_pwmServo.h"


uint16_t g_pwm_pulse = 0;

uint8_t g_pwm_angle[MAX_PWM_SERVO] = {0};
uint16_t g_angle_num[MAX_PWM_SERVO] = {0};

// ģ��PWM���ƶ��
void PwmServo_Handle(void)
{
	g_pwm_pulse++;

	#ifdef USE_SERVO_J1
	if (g_pwm_pulse <= g_angle_num[0]) SERVO_1_HIGH();
	else SERVO_1_LOW();
	#endif

	#ifdef USE_SERVO_J2
	if (g_pwm_pulse <= g_angle_num[1]) SERVO_2_HIGH();
	else SERVO_2_LOW();
	#endif

	#ifdef USE_SERVO_J3
	if (g_pwm_pulse <= g_angle_num[2]) SERVO_3_HIGH();
	else SERVO_3_LOW();
	#endif

	#ifdef USE_SERVO_J4
	if (g_pwm_pulse <= g_angle_num[3]) SERVO_4_HIGH();
	else SERVO_4_LOW();
	#endif

	#ifdef USE_SERVO_J5
	if (g_pwm_pulse <= g_angle_num[4]) SERVO_5_HIGH();
	else SERVO_5_LOW();
	#endif

	#ifdef USE_SERVO_J6
	if (g_pwm_pulse <= g_angle_num[5]) SERVO_6_HIGH();
	else SERVO_6_LOW();
	#endif
	if (g_pwm_pulse >= 2000) g_pwm_pulse = 0;
}

// ��ʼ�����
void PwmServo_Init(void)
{
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef USE_SERVO_J1
	/*��������ʱ��*/
	RCC_APB2PeriphClockCmd(Servo_J1_CLK, ENABLE);
	/*ѡ��Ҫ���Ƶ�����*/
	GPIO_InitStructure.GPIO_Pin = Servo_J1_PIN;
	/*��������ģʽΪͨ���������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	/*������������Ϊ50MHz */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	/*���ÿ⺯������ʼ��PORT*/
	GPIO_Init(Servo_J1_PORT, &GPIO_InitStructure);
	GPIO_SetBits(Servo_J1_PORT, Servo_J1_PIN);
#endif

#ifdef USE_SERVO_J2
	/*��������ʱ��*/
	RCC_APB2PeriphClockCmd(Servo_J2_CLK, ENABLE);
	/*ѡ��Ҫ���Ƶ�����*/
	GPIO_InitStructure.GPIO_Pin = Servo_J2_PIN;
	/*��������ģʽΪͨ���������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	/*������������Ϊ50MHz */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	/*���ÿ⺯������ʼ��PORT*/
	GPIO_Init(Servo_J2_PORT, &GPIO_InitStructure);
	GPIO_SetBits(Servo_J2_PORT, Servo_J2_PIN);
#endif

#ifdef USE_SERVO_J3
	/*��������ʱ��*/
	RCC_APB2PeriphClockCmd(Servo_J3_CLK, ENABLE);
	/*ѡ��Ҫ���Ƶ�����*/
	GPIO_InitStructure.GPIO_Pin = Servo_J3_PIN;
	/*��������ģʽΪͨ���������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	/*������������Ϊ50MHz */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	/*���ÿ⺯������ʼ��PORT*/
	GPIO_Init(Servo_J3_PORT, &GPIO_InitStructure);
	GPIO_SetBits(Servo_J3_PORT, Servo_J3_PIN);
#endif

#ifdef USE_SERVO_J4
	/*��������ʱ��*/
	RCC_APB2PeriphClockCmd(Servo_J4_CLK, ENABLE);
	/*ѡ��Ҫ���Ƶ�����*/
	GPIO_InitStructure.GPIO_Pin = Servo_J4_PIN;
	/*��������ģʽΪͨ���������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	/*������������Ϊ50MHz */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	/*���ÿ⺯������ʼ��PORT*/
	GPIO_Init(Servo_J4_PORT, &GPIO_InitStructure);
	GPIO_SetBits(Servo_J4_PORT, Servo_J4_PIN);
#endif

#ifdef USE_SERVO_J5
	/*��������ʱ��*/
	RCC_APB2PeriphClockCmd(Servo_J5_CLK, ENABLE);
	/*ѡ��Ҫ���Ƶ�����*/
	GPIO_InitStructure.GPIO_Pin = Servo_J5_PIN;
	/*��������ģʽΪͨ���������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	/*������������Ϊ50MHz */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	/*���ÿ⺯������ʼ��PORT*/
	GPIO_Init(Servo_J5_PORT, &GPIO_InitStructure);
	GPIO_SetBits(Servo_J5_PORT, Servo_J5_PIN);
#endif

#ifdef USE_SERVO_J6
	/*��������ʱ��*/
	RCC_APB2PeriphClockCmd(Servo_J6_CLK, ENABLE);
	/*ѡ��Ҫ���Ƶ�����*/
	GPIO_InitStructure.GPIO_Pin = Servo_J6_PIN;
	/*��������ģʽΪͨ���������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	/*������������Ϊ50MHz */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	/*���ÿ⺯������ʼ��PORT*/
	GPIO_Init(Servo_J6_PORT, &GPIO_InitStructure);
	GPIO_SetBits(Servo_J6_PORT, Servo_J6_PIN);
#endif

	for (int i = 0; i < MAX_PWM_SERVO; i++)
	{
		g_pwm_angle[i] = 90;
		g_angle_num[i] = (g_pwm_angle[i] * 11 + 500) / 10;
	}
}

uint8_t* PwmServo_Get_Angle(void)
{
	return (uint8_t*)g_pwm_angle;
}


// ����pwm����Ƕȣ�index=0~MAX_PWM_SERVO��angleΪ0-180
void PwmServo_Set_Angle(uint8_t index, uint8_t angle)
{
	if (index >= MAX_PWM_SERVO) return;
	if (angle > 180) return;
	g_pwm_angle[index] = angle;
	g_angle_num[index] = (angle * 11 + 500) / 10;
}

// ����ȫ��pwm����ĽǶ�
void PwmServo_Set_Angle_All(uint8_t angle_s1, uint8_t angle_s2, uint8_t angle_s3, uint8_t angle_s4)
{
	if (angle_s1 <= 180)
	{
		g_pwm_angle[0] = angle_s1;
		g_angle_num[0] = (angle_s1 * 11 + 500) / 10;
	}

	if (angle_s2 <= 180) 
	{
		g_pwm_angle[1] = angle_s2;
		g_angle_num[1] = (angle_s2 * 11 + 500) / 10;
	}

	if (angle_s3 <= 180)
	{
		g_pwm_angle[2] = angle_s3;
		g_angle_num[2] = (angle_s3 * 11 + 500) / 10;
	}

	if (angle_s4 <= 180)
	{
		g_pwm_angle[3] = angle_s4;
		g_angle_num[3] = (angle_s4 * 11 + 500) / 10;
	}
}

