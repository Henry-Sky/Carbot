#include "bsp_io_i2c.h"
#include "bsp.h"


//IO��������
#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=(u32)8<<12;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=(u32)3<<12;}

//IO��������	 
#define IIC_SCL    PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA	 
#define READ_SDA   PBin(11)  //����SDA 

// �ȴ����ŵ�ʱ�䣬�ɸ���оƬʱ���޸ģ�ֻҪ����ͨѶҪ�󼴿ɡ�
#define DELAY_FOR_COUNT      10


static void Delay_For_Pin(vu8 nCount) 
{
    uint8_t i = 0;
    for(; nCount != 0; nCount--)
    {
        for (i = 0; i < DELAY_FOR_COUNT; i++); 
    }
}


/**
 * @Brief: ��ʼ��I2C��Ӧ�Ľӿ�����
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_Init(void)
{
    RCC->APB2ENR |= 1 << 3;   //��ʹ������IO PORTBʱ��
    GPIOB->CRH &= 0XFFFF00FF; //PB 10/11 �������
    GPIOB->CRH |= 0X00003300;
}

/**
 * @Brief: ����IIC��ʼ�ź�
 * @Note:
 * @Parm: void
 * @Retval: void
 */
int IIC_Start(void)
{
    SDA_OUT(); //sda�����
    IIC_SDA = 1;
    if (!READ_SDA)
        return 0;
    IIC_SCL = 1;
    Delay_For_Pin(1);
    IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
    if (READ_SDA)
        return 0;
    Delay_For_Pin(2);
    IIC_SCL = 0; //ǯסI2C���ߣ�׼�����ͻ��������
    return 1;
}

/**
 * @Brief: ����IICֹͣ�ź�
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_Stop(void)
{
    SDA_OUT(); //sda�����
    IIC_SCL = 0;
    IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
    Delay_For_Pin(2);
    IIC_SCL = 1;
    Delay_For_Pin(1);
    IIC_SDA = 1; //����I2C���߽����ź�
    Delay_For_Pin(2);
}

/**
 * @Brief: �ȴ�Ӧ���źŵ���
 * @Note:
 * @Parm:
 * @Retval: 1:����Ӧ��ɹ� | 0:����Ӧ��ʧ��
 */
int IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    SDA_IN(); //SDA����Ϊ����
    IIC_SDA = 1;
    Delay_For_Pin(1);
    IIC_SCL = 1;
    Delay_For_Pin(1);
    while (READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 50)
        {
            IIC_Stop();
            return 0;
        }
        Delay_For_Pin(1);
    }
    IIC_SCL = 0; //ʱ�����0
    return 1;
}

/**
 * @Brief: ����ACKӦ��
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_Ack(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 0;
    Delay_For_Pin(1);
    IIC_SCL = 1;
    Delay_For_Pin(1);
    IIC_SCL = 0;
}

/**
 * @Brief: ����NACKӦ��
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_NAck(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 1;
    Delay_For_Pin(1);
    IIC_SCL = 1;
    Delay_For_Pin(1);
    IIC_SCL = 0;
}

/**
 * @Brief: IIC����һ���ֽ�
 * @Note:
 * @Parm: void
 * @Retval: void
 */
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    IIC_SCL = 0; //����ʱ�ӿ�ʼ���ݴ���
    for (t = 0; t < 8; t++)
    {
        IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        Delay_For_Pin(1);
        IIC_SCL = 1;
        Delay_For_Pin(1);
        IIC_SCL = 0;
        Delay_For_Pin(1);
    }
}

uint8_t g_addr[128] = {0};


void i2c_scanf_addr(void)
{
    uint8_t i2c_count = 0;
    for (int i = 1; i < 128; i++)
    {
        if (!IIC_Start())
        {
            printf("External IIC Start Error:%d\n", i);
            return;
        }
        IIC_Send_Byte(i << 1);
        if (!IIC_Wait_Ack())
        {
            if (i == 127)
            {
                printf("External IIC Scanf End, Count=%d\n", i2c_count);
            }
            continue;
        }
        IIC_Stop();
        i2c_count++;
        printf("External IIC Found address:0x%2x\n", i);
        g_addr[i] = i;
    }
}


/**
 * @Brief: I2Cд���ݺ���
 * @Note:
 * @Parm: addr:I2C��ַ | reg:�Ĵ��� | len:���ݳ��� | data:����ָ��
 * @Retval: 0:ֹͣ | 1:����д
 */
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    int i;
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    for (i = 0; i < len; i++)
    {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack())
        {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}

/**
 * @Brief: I2C������
 * @Note:
 * @Parm: ����ͬд����
 * @Retval:
 */
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((addr << 1) + 1);
    IIC_Wait_Ack();
    while (len)
    {
        if (len == 1)
            *buf = IIC_Read_Byte(0);
        else
            *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}

/**
 * @Brief: ��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
 * @Note:
 * @Parm:
 * @Retval:
 */
uint8_t IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN(); //SDA����Ϊ����
    for (i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        Delay_For_Pin(2);
        IIC_SCL = 1;
        receive <<= 1;
        if (READ_SDA)
            receive++;
        Delay_For_Pin(2);
    }
    if (ack)
        IIC_Ack(); //����ACK
    else
        IIC_NAck(); //����nACK
    return receive;
}

/**
 * @Brief: ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
 * @Note: 
 * @Parm: I2C_Addr  Ŀ���豸��ַ | addr     �Ĵ�����ַ
 * @Retval: 
 */
//�ú���������,���������!!!!
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr)
{
    unsigned char res = 0;

    IIC_Start();
    IIC_Send_Byte(I2C_Addr); //����д����
    res++;
    IIC_Wait_Ack();
    IIC_Send_Byte(addr);
    res++; //���͵�ַ
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(I2C_Addr + 1);
    res++; //�������ģʽ
    IIC_Wait_Ack();
    res = IIC_Read_Byte(0);
    IIC_Stop(); //����һ��ֹͣ����

    return res;
}

/**
 * @Brief: ��ȡָ���豸 ָ���Ĵ����� length��ֵ
 * @Note: 
 * @Parm: dev  Ŀ���豸��ַ | reg   �Ĵ�����ַ | length Ҫ�����ֽ��� | *data  ���������ݽ�Ҫ��ŵ�ָ��
 * @Retval: ���������ֽ�����
 */
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    uint8_t count = 0;

    IIC_Start();
    IIC_Send_Byte(dev); //����д����
    IIC_Wait_Ack();
    IIC_Send_Byte(reg); //���͵�ַ
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(dev + 1); //�������ģʽ
    IIC_Wait_Ack();

    for (count = 0; count < length; count++)
    {

        if (count != length - 1)
            data[count] = IIC_Read_Byte(1); //��ACK�Ķ�����
        else
            data[count] = IIC_Read_Byte(0); //���һ���ֽ�NACK
    }
    IIC_Stop(); //����һ��ֹͣ����
    return count;
}

/**
 * @Brief: ������ֽ�д��ָ���豸 ָ���Ĵ���
 * @Note: 
 * @Parm: dev  Ŀ���豸��ַ | reg   �Ĵ�����ַ | length Ҫд���ֽ��� | *data  ��Ҫд�����ݵ��׵�ַ
 * @Retval: �����Ƿ�ɹ�
 */
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{

    uint8_t count = 0;
    IIC_Start();
    IIC_Send_Byte(dev); //����д����
    IIC_Wait_Ack();
    IIC_Send_Byte(reg); //���͵�ַ
    IIC_Wait_Ack();
    for (count = 0; count < length; count++)
    {
        IIC_Send_Byte(data[count]);
        IIC_Wait_Ack();
    }
    IIC_Stop(); //����һ��ֹͣ����

    return 1; //status == 0;
}

/**
 * @Brief: ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
 * @Note: 
 * @Parm: dev  Ŀ���豸��ַ | reg    �Ĵ�����ַ | *data  ���������ݽ�Ҫ��ŵĵ�ַ
 * @Retval: 1
 */
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
{
    *data = I2C_ReadOneByte(dev, reg);
    return 1;
}

/**
 * @Brief: д��ָ���豸 ָ���Ĵ���һ���ֽ�
 * @Note: 
 * @Parm: dev  Ŀ���豸��ַ | reg    �Ĵ�����ַ | data  ��Ҫд����ֽ�
 * @Retval: 1
 */
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}

/**
 * @Brief: �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
 * @Note: 
 * @Parm: dev  Ŀ���豸��ַ | reg    �Ĵ�����ַ | bitStart  Ŀ���ֽڵ���ʼλ | length   λ���� | data    ��Ÿı�Ŀ���ֽ�λ��ֵ
 * @Retval: 1:�ɹ� | 0:ʧ��
 */
uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{

    uint8_t b;
    if (IICreadByte(dev, reg, &b) != 0)
    {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    }
    else
    {
        return 0;
    }
}

/**
 * @Brief: �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
 * @Note: 
 * @Parm: dev  Ŀ���豸��ַ | reg    �Ĵ�����ַ | bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ | data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
 * @Retval: 1:�ɹ� | 0:ʧ��
 */
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}
