#include "include_05.h"

AD2SXB AD2SXBPare = AD2SXB_DEFAULTS;
FLASHDATA flash_data = {0,0,0,0};
Uint16 value[4] = { 0, 0, 0, 0 };


void Motor_State_Read(void)
{
    AD2SXBPare.init_Angle = flash_data.flash_init_Angle; //��ȡ�����ʼ״̬
    AD2SXBPare.AD2S_angle = (Uint16) (SPIRead_data[SPI_ROTOR_POSITION]); // ��ȡλ���ź�

    if (SPI_ControlPara.Control_Mode == Control_Mode_ZeroAngle) //��λ��ȷ��ģʽ���Ƕȸ�ֵΪ��
    {
        AD2SXBPare.PM_angle = 0;
    }
    else
    {
        AD2SXBPare.PM_angle = AD2SXBPare.AD2S_angle - AD2SXBPare.init_Angle;
    }
    if (AD2SXBPare.PM_angle > 4095)
        AD2SXBPare.PM_angle -= 4096;
    else if (AD2SXBPare.PM_angle < 0)
        AD2SXBPare.PM_angle += 4096;

    AD2SXBPare.Move_State = (Uint16) (SPIRead_data[SPI_VEER]); //��ȡ���״̬

    if (AD2SXBPare.Move_State == 1)                          //���ݵ��״̬��ȷ������ٶȵ�����ת
    {
        AD2SXBPare.AD2S_Spd = (Uint16) (SPIRead_data[SPI_ROTATE_SPEED]);
    }
    else if (AD2SXBPare.Move_State == 2)
    {
        AD2SXBPare.AD2S_Spd = -((Uint16) (SPIRead_data[SPI_ROTATE_SPEED]));
    }

    AD2SXBPare.AD2S_SpdRPM = (int16) (AD2SXBPare.AD2S_Spd);
    AD2SXBPare.AD2S_SpdRPMH = AD2SXBPare.AD2S_SpdRPMH * 0.218
            + AD2SXBPare.AD2S_SpdRPM * 0.782;  // ת��һ�ڵ�ͨ�˲�
    AD2SXBPare.Abs_SpdRPM = Abs(AD2SXBPare.AD2S_SpdRPMH);
}

void Motor_Parameter_Read(void)                  //��ȡ�����ʼ״̬
{
    Uint16 i = 0;
    Uint16 *flashAddress = (unsigned int*) Save_Flash_Address;
    for (i = 0; i < 4; i++)        //��FLASH�ж�ȡֵ
    {
        value[i] = *flashAddress;
        flashAddress += 1;
    }
    flash_data.flash_flag = value[0];
    flash_data.flash_init_Angle = value[1];   //��ʼ�Ƕ�Ϊ��һλ����
}
//===========================================================================
// No more.
//===========================================================================
