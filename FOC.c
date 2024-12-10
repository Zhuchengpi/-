#include "include_04.h"
#include "include_05.h"

Control_FB Control_FB_Para = Control_FB_DEFAULTS;
GXieLv SpeedRpm_GXieLv=GXieLv_DEFAULTS;
GXieLv Trq_0D1NM_GXieLv=GXieLv_DEFAULTS;
GXieLv VF_Freq_GXieLv=GXieLv_DEFAULTS;
GXieLv VF_Vq_GXieLv=GXieLv_DEFAULTS;
GXieLv V_d_GXieLv=GXieLv_DEFAULTS;
GXieLv V_q_GXieLv=GXieLv_DEFAULTS;
GXieLv I_d_GXieLv=GXieLv_DEFAULTS;
GXieLv I_q_GXieLv=GXieLv_DEFAULTS;
CLARKE CLARKE_PCurr=CLARKE_DEFAULTS;
PARK PARK_PCurr=PARK_DEFAULTS;
Ang_SinCos Park_SinCos=Ang_SinCos_DEFAULTS;
Ang_SinCos IPark_SinCos=Ang_SinCos_DEFAULTS;
IPARK IPARK_PVdq=IPARK_DEFAULTS;
SVPWM SVPWM_dq=SVPWM_DEFAULTS;

Uint16 Spd_Count=0,
FW_Count = 0;    // �ٶȻ�ִ�����ڼ���
_iq10 FW_Curr_d = 0, V_d = 0, V_q = 0, VF_Angle = 0;    // ��ѹVs����PI�����d�����ŵ���
Uint16 VF_AngleJZ = 0, Park_Angle = 0;

void UVW_Axis_dq(void)   // ���������dq������仯
{
    CLARKE_PCurr.Us = (_IQ10(Volt_CurrPara1.PhaseU_Curr)); // �����Ķ�������������CLARKE�仯
    CLARKE_PCurr.Vs = (_IQ10(Volt_CurrPara1.PhaseV_Curr));
    CLARKE_Cale((p_CLARKE) &CLARKE_PCurr); //Clarke�任���� �ȷ�ֵ�任

    Park_Angle = AD2SXBPare.PM_angle;
    PARK_PCurr.Angle = Park_Angle;

    Park_SinCos.table_Angle = PARK_PCurr.Angle;  // ��ȡ������ż�λ��
    SinCos_Table((p_Ang_SinCos) &Park_SinCos);   // ����λ�ö�ȡ������ֵ
    PARK_PCurr.Sine = Park_SinCos.table_Sin;
    PARK_PCurr.Cosine = Park_SinCos.table_Cos;  //��ز����������PARK�仯�õ�dq�����
    PARK_PCurr.Alpha = CLARKE_PCurr.Alpha;
    PARK_PCurr.Beta = CLARKE_PCurr.Beta;
    PARK_Cale((p_PARK) &PARK_PCurr);
}

void Speed_FOC(void) // �ٶȿ���ģʽ
{
    SpeedRpm_GXieLv.XieLv_Grad = Speed_Grad_RPM;  // �ٶ�������ݶȼ���
    SpeedRpm_GXieLv.Grad_Timer = Speed_Grad_Timer;  // 2ms�ٶȻ�����
    SpeedRpm_GXieLv.Timer_Count++;
    if (SpeedRpm_GXieLv.Timer_Count > SpeedRpm_GXieLv.Grad_Timer)
    {
        SpeedRpm_GXieLv.Timer_Count = 0;
        Grad_XieLv((p_GXieLv) &SpeedRpm_GXieLv);
        pi_spd.Ref = SpeedRpm_GXieLv.XieLv_Y;   // �ٶȻ�����ֵ,��_IQ10��ʽ�趨�ٶ����IQ10
        pi_spd.Fbk = _IQ10(AD2SXBPare.AD2S_SpdRPMH);
        PI_Controller((p_PI_Control) &pi_spd);  // �ٶȻ�PI����
        pi_spd.OutF = _IQ10mpy(
                pi_spd.OutF, GM_Low_Lass_A)+_IQ10mpy(pi_spd.Out,GM_Low_Lass_B); //��·�˲������
    }
    Trq_0D1NM_GXieLv.XieLv_X = pi_spd.OutF;
}

void FW_PI(void)    //  ���Ժ���   // pi_FW
{
    FW_Count++;
    if (FW_Count > FW_Grad_Timer)
    {
        FW_Count = 0;
        pi_FW.Ref = _IQ10(Vdc_s);
        pi_FW.Fbk =
        _IQ10(IQSqrt(
                        (_IQ10toF(V_q)) * (_IQ10toF(V_q)) * 100
                        + (_IQ10toF(V_d)) * (_IQ10toF(V_d)) * 100)) * 0.1; //����VS
        PI_Controller((p_PI_Control) &pi_FW);
        pi_FW.OutF = _IQ10mpy(pi_FW.OutF,
                GM_Low_Lass_A) + _IQ10mpy(pi_FW.Out, GM_Low_Lass_B);
        FW_Curr_d = pi_FW.OutF;
    }
}

void VF_Control(void)    //VF����ģʽ
{
    float32 PWM_Step_Angle = 0;
    VF_Freq_GXieLv.XieLv_Grad = VF_F_Grad_0D1HZ;  //�����ٶȵ��ݶ�ֵ
    VF_Freq_GXieLv.Grad_Timer = VF_F_Grad_Timer;  //�����ٶȵ��ݶ�ʱ��
    VF_Vq_GXieLv.XieLv_Grad = VF_q_Grad_0D1V;  //�����ٶȵ��ݶ�ֵ
    VF_Vq_GXieLv.Grad_Timer = VF_q_Grad_Timer;  //�����ٶȵ��ݶ�ʱ��
    VF_Freq_GXieLv.Timer_Count++;
    if (VF_Freq_GXieLv.Timer_Count > VF_Freq_GXieLv.Grad_Timer)
    {
        VF_Freq_GXieLv.Timer_Count = 0;
        Grad_XieLv((p_GXieLv) &VF_Freq_GXieLv);
        Grad_XieLv((p_GXieLv) &VF_Vq_GXieLv);
    }
    float32 VF_Freq_EX = Limit_Sat(VF_Freq_GXieLv.XieLv_Y, VF_F_Max, VF_F_Min);
    PWM_Step_Angle = (float32) (4096 / ((float32) (PWM_FREQ / VF_Freq_EX)));
    VF_Angle += PWM_Step_Angle;
    if (VF_Angle >= 4096)
        VF_Angle -= 4096;
    else if (VF_Angle < 0)
        VF_Angle += 4096;
    VF_AngleJZ = (((Uint16) VF_Angle) & AD2SXB_12Bit);

    V_d = 0;
    V_q = VF_Vq_GXieLv.XieLv_Y;
}


void Trq_FOC(void)    //Ť�ؿ���ģʽ  // pi_FW
{
    Uint16 MinSpeed_Section;
    Uint16 Speed_Index = 0;
    Uint16 Abs_FOC_Trq = 0;
    Uint16 FOC_Trq_Index = 0;
    Trq_0D1NM_GXieLv.XieLv_Grad = Trq_Grad_0D1NM;
    Trq_0D1NM_GXieLv.Grad_Timer = Trq_Grad_Timer;  //�����ݶ�ʱ��
    Trq_0D1NM_GXieLv.Timer_Count++;
    if (Trq_0D1NM_GXieLv.Timer_Count > Trq_0D1NM_GXieLv.Grad_Timer)
    {
        float32 Id_Table_CMD = 0, Iq_Table_CMD = 0;

        Trq_0D1NM_GXieLv.Timer_Count = 0;
        Grad_XieLv((p_GXieLv) &Trq_0D1NM_GXieLv);  //Trq_0D1NM_GXieLv.XieLv_X
        MinSpeed_Section = (Uint16) (AD2SXBPare.Abs_SpdRPM / MinSpeed_Grad);
        Speed_Index = (Uint16) (MinSpeed_Section / MinSpeed_Grad);
        Uint16 Speed_Index_Offset = (Uint16) (MinSpeed_Section % MinSpeed_Grad);

        Abs_FOC_Trq = (Uint16) (Abs(_IQ10toF(Trq_0D1NM_GXieLv.XieLv_Y)));
        FOC_Trq_Index = Abs_FOC_Trq;

        if (Speed_Index > Motor_MAX_Speed_Count - 2)
            Speed_Index = Motor_MAX_Speed_Count - 2;
        if (FOC_Trq_Index > Motor_MAX_Trq_Count - 2)
            FOC_Trq_Index = Motor_MAX_Trq_Count - 2;

        //Id���
        float32 Id_Value = (float32) (Id_Map[Speed_Index][FOC_Trq_Index]);
        float32 Next_Id_Value =
                (float32) (Id_Map[Speed_Index + 1][FOC_Trq_Index]);
        Id_Table_CMD = (float32) (Id_Value
                + (float32) ((Next_Id_Value - Id_Value) * Speed_Index_Offset
                        / MinSpeed_Grad));

        float32 Iq_Value = (float32) (Iq_Map[Speed_Index][FOC_Trq_Index]);
        float32 Next_Iq_Value =
                (float32) (Iq_Map[Speed_Index + 1][FOC_Trq_Index]);
        Iq_Table_CMD = (float32) (Iq_Value
                + (float32) ((Next_Iq_Value - Iq_Value) * Speed_Index_Offset
                        / MinSpeed_Grad));

        I_d_GXieLv.XieLv_X = _IQ10(Id_Table_CMD);

        if (Trq_0D1NM_GXieLv.XieLv_Y >= 0)
        {
            I_q_GXieLv.XieLv_X = _IQ10(Iq_Table_CMD);
        }
        else
        {
            I_q_GXieLv.XieLv_X = -_IQ10(Iq_Table_CMD);
        }

    }
}

/*
void Trq_FOC(void)    // Ť�ؿ���ģʽ  // pi_FW
{
    static const Uint16 MAX_SPEED_INDEX = Motor_MAX_Speed_Count - 2;
    static const Uint16 MAX_TRQ_INDEX = Motor_MAX_Trq_Count - 2;

    Uint16 minSpeedSection;
    Uint16 speedIndex = 0;
    Uint16 absFOCTrq = 0;
    Uint16 focTrqIndex = 0;
    Uint16 speedIndexOffset = 0;
    float32 idTableCMD = 0.0f, iqTableCMD = 0.0f;

    Trq_0D1NM_GXieLv.XieLv_Grad = Trq_Grad_0D1NM;
    Trq_0D1NM_GXieLv.Grad_Timer = Trq_Grad_Timer;  // �����ݶ�ʱ��
    Trq_0D1NM_GXieLv.Timer_Count++;

    if (Trq_0D1NM_GXieLv.Timer_Count > Trq_0D1NM_GXieLv.Grad_Timer)
    {
        Trq_0D1NM_GXieLv.Timer_Count = 0;
        Grad_XieLv((p_GXieLv)&Trq_0D1NM_GXieLv);  // Trq_0D1NM_GXieLv.XieLv_X

        minSpeedSection = (Uint16)(AD2SXBPare.Abs_SpdRPM / MinSpeed_Grad);
        speedIndex = minSpeedSection / MinSpeed_Grad;
        speedIndexOffset = minSpeedSection % MinSpeed_Grad;

        absFOCTrq = (Uint16)Abs(_IQ10toF(Trq_0D1NM_GXieLv.XieLv_Y));
        focTrqIndex = absFOCTrq;

        speedIndex = (speedIndex > MAX_SPEED_INDEX) ? MAX_SPEED_INDEX : speedIndex;
        focTrqIndex = (focTrqIndex > MAX_TRQ_INDEX) ? MAX_TRQ_INDEX : focTrqIndex;

        // Id���
        const float32 *idMapRow = Id_Map[speedIndex];
        const float32 *nextIdMapRow = Id_Map[speedIndex + 1];
        float32 idValue = idMapRow[focTrqIndex];
        float32 nextIdValue = nextIdMapRow[focTrqIndex];
        idTableCMD = idValue + (nextIdValue - idValue) * speedIndexOffset / MinSpeed_Grad;

        const float32 *iqMapRow = Iq_Map[speedIndex];
        const float32 *nextIqMapRow = Iq_Map[speedIndex + 1];
        float32 iqValue = iqMapRow[focTrqIndex];
        float32 nextIqValue = nextIqMapRow[focTrqIndex];
        iqTableCMD = iqValue + (nextIqValue - iqValue) * speedIndexOffset / MinSpeed_Grad;

        I_d_GXieLv.XieLv_X = _IQ10(idTableCMD);

        if (Trq_0D1NM_GXieLv.XieLv_Y >= 0)
        {
            I_q_GXieLv.XieLv_X = _IQ10(iqTableCMD);
        }
        else
        {
            I_q_GXieLv.XieLv_X = -_IQ10(iqTableCMD);
        }
    }
}
*/

void Vdq_FOC(void)    //��ѹ����ģʽ
{
    float32 Us_Limit = 0;
    V_d_GXieLv.XieLv_Grad = Vdq_Grad_0D1V;  //�����ٶȵ��ݶ�ֵ
    V_d_GXieLv.Grad_Timer = Vdq_Grad_Timer;  //�����ٶȵ��ݶ�ʱ��
    V_q_GXieLv.XieLv_Grad = Vdq_Grad_0D1V;  //�����ٶȵ��ݶ�ֵ
    V_q_GXieLv.Grad_Timer = Vdq_Grad_Timer;  //�����ٶȵ��ݶ�ʱ��
    V_d_GXieLv.Timer_Count++;
    if (V_d_GXieLv.Timer_Count > V_d_GXieLv.Grad_Timer)
    {
        V_d_GXieLv.Timer_Count = 0;
        Grad_XieLv((p_GXieLv) &V_d_GXieLv);
        Grad_XieLv((p_GXieLv) &V_q_GXieLv);
    }

    V_d = V_d_GXieLv.XieLv_Y;
    Us_Limit = (IQSqrt(17694.703499 - V_d * V_d * 100)) * 0.1;
    V_q = V_q_GXieLv.XieLv_Y;
    V_q = Min(Us_Limit, V_q);
}

void Idq_FOC(void)    //��������ģʽ
{
    _iq10 Us_Limit;

    pi_id.Ref = I_d_GXieLv.XieLv_Y;   //Trq_FOC�õ�  IQ10��ʽ
    pi_id.Fbk = PARK_PCurr.Ds;        //PARK_PCurr�õ�
    PI_Controller((p_PI_Control) &pi_id);
    pi_id.OutF = _IQ10mpy(pi_id.OutF,
            GM_Low_Lass_A) + _IQ10mpy(pi_id.Out, GM_Low_Lass_B);
    V_d = pi_id.OutF;

    /*Us_Limit = (IQSqrt((17694.703499) - (_IQ10toF(V_d) * _IQ10toF(V_d) * 70)))
            * 0.1;
            * */
    Us_Limit = _IQ10mpy(_IQ10sqrt((18119376) - _IQ10mpy(_IQ10mpy(V_d,V_d),0X11800)),0X66);
    pi_iq.Umax = Us_Limit;
    pi_iq.Umin = -Us_Limit;

    pi_iq.Ref = I_q_GXieLv.XieLv_Y;   //Trq_FOC�õ� ����IQ10
    pi_iq.Fbk = PARK_PCurr.Qs;        //PARK_PCurr�õ�
    PI_Controller((p_PI_Control) &pi_iq);
    pi_iq.OutF = _IQ10mpy(pi_iq.OutF,
            GM_Low_Lass_A) + _IQ10mpy(pi_iq.Out, GM_Low_Lass_B);
    V_q = pi_iq.OutF;
}

void FOC_Svpwm_dq(void)                               // dq���ѹ���룬��Park�任�����SVPWM
{
    IPARK_PVdq.Ds = V_d;     //Idq_FOC�õ� IQ10��ʽ
    IPARK_PVdq.Qs = V_q;     //Idq_FOC�õ� IQ10��ʽ
    switch (SPI_ControlPara.Control_Mode)
    {
    case Control_Mode_VF:              // VF����ģʽ��ʱ�򣬵���Ƕ��ԼӼ��Ƕ�λ��
        IPARK_PVdq.Angle = VF_AngleJZ;
        break;
    default:
        IPARK_PVdq.Angle = AD2SXBPare.PM_angle;            // ��VFʱ��λ�ô�������Ϊ��Ƕ�
        break;
    }
    IPark_SinCos.table_Angle = IPARK_PVdq.Angle;           //�����������IQ10��ʽ
    SinCos_Table((p_Ang_SinCos) &IPark_SinCos);           // ��PARK�仯�������Ҳ��
    IPARK_PVdq.Sine = IPark_SinCos.table_Sin;
    IPARK_PVdq.Cosine = IPark_SinCos.table_Cos;
    IPARK_Cale((p_IPARK) &IPARK_PVdq);                    // ��Park�任
    SVPWM_dq.Ualpha = IPARK_PVdq.Alpha;
    SVPWM_dq.Ubeta = IPARK_PVdq.Beta;
    SVPWM_Cale((p_SVPWM) &SVPWM_dq);               // ��Alpha��Beta��ѹ�������SVPWM��ռ�ձ�

}

void Start_Motor(void)
{
    GpioDataRegs.GPASET.bit.GPIO12 = 0; //�͵�ƽ��ʹ��SN74LVC8T245PWR�������PWM�������


}
void Stop_Motor(void)   // ͣ������  ��ʼ�������Ʋ������
{
// ����ָ������ٶȻ�����PID���ƹ��̱���
    GpioDataRegs.GPASET.bit.GPIO12 = 1; //�ߵ�ƽ���ж�SN74LVC8T245PWR���������PWM�������
    Control_FB_Para.Motor_State = 0;
    SpeedRpm_GXieLv.XieLv_X = 0;
    SpeedRpm_GXieLv.XieLv_Y = 0;

    Trq_0D1NM_GXieLv.XieLv_X = 0;
    Trq_0D1NM_GXieLv.XieLv_Y = 0;

    VF_Freq_GXieLv.XieLv_X = 0;
    VF_Vq_GXieLv.XieLv_X = 0;
    VF_Freq_GXieLv.XieLv_Y = 0;
    VF_Vq_GXieLv.XieLv_Y = 0;

    V_d_GXieLv.XieLv_X = 0;
    V_q_GXieLv.XieLv_X = 0;
    I_d_GXieLv.XieLv_X = 0;
    I_q_GXieLv.XieLv_X = 0;
    V_d_GXieLv.XieLv_Y = 0;
    V_q_GXieLv.XieLv_Y = 0;
    I_d_GXieLv.XieLv_Y = 0;
    I_q_GXieLv.XieLv_Y = 0;

    pi_spd.Ref = 0;
    pi_spd.Fbk = 0;
    pi_spd.v1 = 0;
    pi_spd.Out = 0;
    pi_spd.OutF = 0;
    pi_spd.i1 = 0;
    pi_spd.ui = 0;

    pi_id.Ref = 0;
    pi_id.Fbk = 0;
    pi_id.v1 = 0;
    pi_id.Out = 0;
    pi_id.OutF = 0;
    pi_id.i1 = 0;
    pi_id.ui = 0;

    pi_iq.Ref = 0;
    pi_iq.Fbk = 0;
    pi_iq.v1 = 0;
    pi_iq.Out = 0;
    pi_iq.OutF = 0;
    pi_iq.i1 = 0;
    pi_iq.ui = 0;

    pi_FW.Ref = 0;
    pi_FW.Fbk = 0;
    pi_FW.v1 = 0;
    pi_FW.Out = 0;
    pi_FW.OutF = 0;
    pi_FW.i1 = 0;
    pi_FW.ui = 0;
    V_d = 0;
    V_q = 0;

}

interrupt void MainISR(void)                //0.2ms����һ��
{
    ADC_Sample1();                          //ADC��ѹ����
    ADC_Sample_dea();                       //��ѹ����
    Motor_State_Read();                     //���״̬��ȡ
//IQ10
    UVW_Axis_dq();                          //���������dq��ı仯
//FW_PI();                                  //���ŵ��Ժ���
    FOC_Control_Select();                   //ģʽѡ��
    FOC_Svpwm_dq();                         //��PARK�仯
    Svpwm_Outpwm();                         //PWM���

//Open_loop();                              //����ȷ�������ʼ״̬
    EPwm1Regs.ETCLR.bit.INT = 1;            // ʹ���ж�
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; //���ж�����

}

void Open_loop(void)
{
    V_d = 0.1;
    V_q = 1.5;
    FOC_Svpwm_dq();
    Svpwm_Outpwm();                            //PWM���
}

void FOC_Control_TestPara(void)
{
// �������
    Control_FB_Para.SpeedRPM = AD2SXBPare.AD2S_SpdRPMH;
    Control_FB_Para.VF_Freq = VF_Freq_GXieLv.XieLv_Y;
    Control_FB_Para.Trq = Trq_0D1NM_GXieLv.XieLv_Y;
//Control_FB_Para.V_s= (IQSqrt(V_q*V_q*100+V_d*V_d*100))*0.1;
    Control_FB_Para.V_d = V_d;
    Control_FB_Para.V_q = V_q;
    Control_FB_Para.I_d_fb = PARK_PCurr.Ds;
    Control_FB_Para.I_q_fb = PARK_PCurr.Qs;
    Control_FB_Para.I_d = I_d_GXieLv.XieLv_Y;
    Control_FB_Para.I_q = I_q_GXieLv.XieLv_Y;
    Control_FB_Para.PM_Angle = IPARK_PVdq.Angle;
    Control_FB_Para.Motor_State = 0;
}

