/*
 * File: RLS.c
 *
 */

#include "RLS.h"
#include <math.h>
#include <string.h>
#include "rtwtypes.h"

/*�����źź�״̬  */
DW rtDW;

/* �ⲿ���� */
ExtU rtU;

/* �ⲿ��� */
ExtY rtY;

/* ʵʱģ�� */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/*
��ʼ���ͼ����м������

�������ȼ��������źŵĲ�֣��洢�� rtDW.U_k_D �� rtDW.U_k_Q �����С�
���� gama_t���������ڸ���Э�����������ӡ�
����Э�������

ͨ���� P_k_1 ���� gama_t��������һ����Э��������Է�ӳ�µĹ��ơ�
����������ƣ�

ͨ����Э��������������ź���ˣ������������ֵ�����洢�� rtDW.U_k_D �С�
��� U_k_D[0] ��ֵ���������0�������״̬���� X_hat_k_1��
�������ֵ��

�����յĲ�������ֵ���ݵ�����˿� rtY��
*/
/* ģ�ͽ�Ծ����*/
void RLS_step(void)
{
  float tmp;
  float tmp_0;
  float tmp_1;
  float tmp_2;
  float tmp_3;

  /********* ******************************************
  �ݹ���С���ˣ�RLS�� �����źŴ����Э����������
  *****************************************************/
  rtDW.U_k_D[0] = rtU.RLS_In_Id;                            // ��id������U_k_D[0]��
  rtDW.U_k_D[1] = (rtU.RLS_In_Id - rtDW.id_k_1) * 10000.0F; // ��id�ı仯�ʳ���10000����ǿ�仯��Ӱ��
  rtDW.U_k_D[2] = (real32_T)-rtU.RLS_In_We * rtU.RLS_In_Iq; // ���㸺�Ľ��ٶ� (RLS_In_We) ��q����� (RLS_In_Iq) �ĳ˻������洢�� U_k_D[2] ��
  rtDW.U_k_D[3] = 0.0F;                                     // �� U_k_D[3] ��ʼ��Ϊ0
  rtDW.U_k_Q[0] = rtU.RLS_In_Iq;                            // ����ǰ��q������洢�� U_k_Q[0] ��
  rtDW.U_k_Q[1] = (real32_T)rtU.RLS_In_We * rtU.RLS_In_Id;  // ������ٶ���d������ĳ˻����洢�� U_k_Q[1] �У����ڶ�̬ϵͳ�Ĺ��ơ�
  rtDW.U_k_Q[2] = (rtU.RLS_In_Iq - rtDW.iq_k_1) * 10000.0F; // ����q������ı仯�ʣ�������֮ǰ��d������Ĵ����洢�� U_k_Q[2] ��
  rtDW.U_k_Q[3] = (real32_T)rtU.RLS_In_We;                  // �����ٶȴ洢�� U_k_Q[3] ��
  // ����ǰһ��ʱ�̵ĵ���ֵ���Ա�����һ�ε����н��в�ּ���
  rtDW.iq_k_1 = rtU.RLS_In_Iq;
  rtDW.id_k_1 = rtU.RLS_In_Id;
  // ���� gama_t�����ڸ���Э������󣬻��ڵ�ǰ���Ƶ������ʵ�����֮�������ֵԽ�ӽ�1����ʾ�Թ�ȥ���Ƶ�Ȩ��Խ��
  rtDW.gama_t = 0.999F - (real32_T)exp((real32_T)fabs(rtU.RLS_In_Uq -
                                                      (((rtU.RLS_In_Iq * rtDW.X_hat_k_1[0] + rtDW.U_k_Q[1] * rtDW.X_hat_k_1[1]) +
                                                        rtDW.U_k_Q[2] * rtDW.X_hat_k_1[2]) +
                                                       (real32_T)rtU.RLS_In_We *
                                                           rtDW.X_hat_k_1[3])) *
                                       -0.999F) *
                             0.001F;

  // ����Э������� P_k_1 �ĸ��£���ÿ��Ԫ�س��� gama_t����ӳ�µĹ��Ƶ����Ŷȡ�
  for (rtDW.i = 0; rtDW.i < 16; rtDW.i++)
  {
    rtDW.P_k_1[rtDW.i] = rtDW.P_k_1[rtDW.i] / rtDW.gama_t;
  }

  /***********************************************************************
   * �ݹ���С���ˣ�RLS�����ƹ��̣�����״̬������Э�������
   *
   *************************************************************************/

  // ��ʼ�� U_k_D_m Ϊ0�������� fv1 ���飬���а�������Э������� P_k_1 �������źŵļ�Ȩ�͡�
  rtDW.U_k_D_m = 0.0F;
  for (rtDW.i = 0; rtDW.i < 4; rtDW.i++)
  {
    rtDW.fv1[rtDW.i] = ((rtDW.P_k_1[rtDW.i + 4] * rtDW.U_k_D[1] +
                         rtDW.P_k_1[rtDW.i] * rtU.RLS_In_Id) +
                        rtDW.P_k_1[rtDW.i + 8] * rtDW.U_k_D[2]) +
                       rtDW.P_k_1[rtDW.i + 12] * 0.0F;
    rtDW.U_k_D_tmp = rtDW.i << 2;
    rtDW.U_k_D_m += (((rtDW.P_k_1[rtDW.U_k_D_tmp + 1] * rtDW.U_k_D[1] +
                       rtDW.P_k_1[rtDW.U_k_D_tmp] * rtU.RLS_In_Id) +
                      rtDW.P_k_1[rtDW.U_k_D_tmp + 2] * rtDW.U_k_D[2]) +
                     rtDW.P_k_1[rtDW.U_k_D_tmp + 3] * 0.0F) *
                    rtDW.U_k_D[rtDW.i];
  }

  // �� fv1 �е�ֵ����� y, f, f1, �� f2��Ȼ��������Щֵ�� U_k_D �е�ÿ��Ԫ�ؼ��㲢���� fv ���顣
  rtDW.y = rtDW.fv1[0];
  rtDW.f = rtDW.fv1[1];
  rtDW.f1 = rtDW.fv1[2];
  rtDW.f2 = rtDW.fv1[3];
  for (rtDW.i = 0; rtDW.i < 4; rtDW.i++)
  {
    rtDW.U_k_D_c = rtDW.U_k_D[rtDW.i];
    rtDW.U_k_D_tmp = rtDW.i << 2;
    rtDW.fv[rtDW.U_k_D_tmp] = rtDW.y * rtDW.U_k_D_c;
    rtDW.fv[rtDW.U_k_D_tmp + 1] = rtDW.f * rtDW.U_k_D_c;
    rtDW.fv[rtDW.U_k_D_tmp + 2] = rtDW.f1 * rtDW.U_k_D_c;
    rtDW.fv[rtDW.U_k_D_tmp + 3] = rtDW.f2 * rtDW.U_k_D_c;
  }

  // ʹ�� fv �е�ֵ������Э������� P_k��ͨ����ǰ����ֵ�͹�ȥ��Э������е���
  for (rtDW.i = 0; rtDW.i < 4; rtDW.i++)
  {
    rtDW.y = rtDW.fv[rtDW.i + 4];
    rtDW.f = rtDW.fv[rtDW.i];
    rtDW.f1 = rtDW.fv[rtDW.i + 8];
    rtDW.f2 = rtDW.fv[rtDW.i + 12];
    for (rtDW.U_k_D_tmp = 0; rtDW.U_k_D_tmp < 4; rtDW.U_k_D_tmp++)
    {
      rtDW.P_k_tmp = rtDW.U_k_D_tmp << 2;
      rtDW.P_k_tmp_k = rtDW.P_k_tmp + rtDW.i;
      rtDW.P_k[rtDW.P_k_tmp_k] = rtDW.P_k_1[rtDW.P_k_tmp_k] -
                                 (((rtDW.P_k_1[rtDW.P_k_tmp + 1] * rtDW.y + rtDW.P_k_1[rtDW.P_k_tmp] *
                                                                                rtDW.f) +
                                   rtDW.P_k_1[rtDW.P_k_tmp + 2] * rtDW.f1) +
                                  rtDW.P_k_1[rtDW.P_k_tmp + 3] * rtDW.f2) /
                                     (rtDW.U_k_D_m + 1.0F);
    }
  }

  // ���¼����Э������� P_k ���Ƶ� P_k_1�����ż��㵱ǰ������ y����ʹ��SIMDָ��и��� U_k_D������Э�����״̬���ơ�
  memcpy(&rtDW.P_k_1[0], &rtDW.P_k[0], sizeof(real32_T) << 4U);
  rtDW.y = rtU.RLS_In_Ud - (((rtU.RLS_In_Id * rtDW.X_hat_k_1[0] + rtDW.U_k_D[1] *
                                                                      rtDW.X_hat_k_1[1]) +
                             rtDW.U_k_D[2] * rtDW.X_hat_k_1[2]) +
                            0.0F *
                                rtDW.X_hat_k_1[3]);
  rtDW.U_k_D_m = rtDW.U_k_D[1];
  rtDW.U_k_D_c = rtDW.U_k_D[2];

  for (rtDW.i = 0; rtDW.i < 4; rtDW.i++)
  {
    tmp_3 = rtDW.P_k[rtDW.i + 4];
    tmp = rtDW.P_k[rtDW.i];
    tmp_0 = rtDW.P_k[rtDW.i + 8];
    tmp_1 = rtDW.P_k[rtDW.i + 12];
    tmp_2 = rtDW.X_hat_k_1[rtDW.i];
    rtDW.U_k_D[rtDW.i] = (((((tmp_3 * rtDW.U_k_D_m) + (tmp * rtU.RLS_In_Id)) + (tmp_0 * rtDW.U_k_D_c) + (tmp_1 * 0.0F)) * rtDW.y) + tmp_2);
  }

  /* ' */
  if (rtDW.U_k_D[0] > 0.0F)
  {
    rtDW.X_hat_k_1[0] = rtDW.U_k_D[0];
    rtDW.X_hat_k_1[1] = rtDW.U_k_D[1];
    rtDW.X_hat_k_1[2] = rtDW.U_k_D[2];
    rtDW.X_hat_k_1[3] = rtDW.U_k_D[3];
  }

  // Э����������
  for (rtDW.i = 0; rtDW.i < 16; rtDW.i++)
  {
    rtDW.P_k_1[rtDW.i] = rtDW.P_k_1[rtDW.i] / rtDW.gama_t;
  }

  // ��ʼ�� U_k_D_m Ϊ0�������� fv1 ���飬�漰��Э������� P_k_1 ����ȡ���ݣ���Ȩ�������ź���˲���͡����� U_k_D_m ���ں�����Э���������¡�
  rtDW.U_k_D_m = 0.0F;
  for (rtDW.i = 0; rtDW.i < 4; rtDW.i++)
  {
    rtDW.fv1[rtDW.i] = ((rtDW.P_k_1[rtDW.i + 4] * rtDW.U_k_Q[1] +
                         rtDW.P_k_1[rtDW.i] * rtU.RLS_In_Iq) +
                        rtDW.P_k_1[rtDW.i + 8] * rtDW.U_k_Q[2]) +
                       rtDW.P_k_1[rtDW.i + 12] *
                           (real32_T)rtU.RLS_In_We;
    rtDW.U_k_D_tmp = rtDW.i << 2;
    rtDW.U_k_D_m += (((rtDW.P_k_1[rtDW.U_k_D_tmp + 1] * rtDW.U_k_Q[1] +
                       rtDW.P_k_1[rtDW.U_k_D_tmp] * rtU.RLS_In_Iq) +
                      rtDW.P_k_1[rtDW.U_k_D_tmp + 2] * rtDW.U_k_Q[2]) +
                     rtDW.P_k_1[rtDW.U_k_D_tmp + 3] * (real32_T)rtU.RLS_In_We) *
                    rtDW.U_k_Q[rtDW.i];
  }

  // �� fv1 ��ֵ���� y, f, f1, �� f2�����ţ�ʹ����Щֵ�� U_k_Q �е�Ԫ�ظ��� fv ���飬�������µ��м�����

  rtDW.y = rtDW.fv1[0];
  rtDW.f = rtDW.fv1[1];
  rtDW.f1 = rtDW.fv1[2];
  rtDW.f2 = rtDW.fv1[3];
  for (rtDW.i = 0; rtDW.i < 4; rtDW.i++)
  {
    rtDW.U_k_D_c = rtDW.U_k_Q[rtDW.i];
    rtDW.U_k_D_tmp = rtDW.i << 2;
    rtDW.fv[rtDW.U_k_D_tmp] = rtDW.y * rtDW.U_k_D_c;
    rtDW.fv[rtDW.U_k_D_tmp + 1] = rtDW.f * rtDW.U_k_D_c;
    rtDW.fv[rtDW.U_k_D_tmp + 2] = rtDW.f1 * rtDW.U_k_D_c;
    rtDW.fv[rtDW.U_k_D_tmp + 3] = rtDW.f2 * rtDW.U_k_D_c;
  }

  // ʹ�� fv �����е�ֵ����Э������� P_k��ͨ���������źŵļ�Ȩ�ʹ� P_k_1 ����ȡ�������� U_k_D_m ����Э���ȷ���㷨���ȶ��ԡ�
  for (rtDW.i = 0; rtDW.i < 4; rtDW.i++)
  {
    rtDW.y = rtDW.fv[rtDW.i + 4];
    rtDW.f = rtDW.fv[rtDW.i];
    rtDW.f1 = rtDW.fv[rtDW.i + 8];
    rtDW.f2 = rtDW.fv[rtDW.i + 12];
    for (rtDW.U_k_D_tmp = 0; rtDW.U_k_D_tmp < 4; rtDW.U_k_D_tmp++)
    {
      rtDW.P_k_tmp = rtDW.U_k_D_tmp << 2;
      rtDW.P_k_tmp_k = rtDW.P_k_tmp + rtDW.i;
      rtDW.P_k[rtDW.P_k_tmp_k] = rtDW.P_k_1[rtDW.P_k_tmp_k] -
                                 (((rtDW.P_k_1[rtDW.P_k_tmp + 1] * rtDW.y + rtDW.P_k_1[rtDW.P_k_tmp] *
                                                                                rtDW.f) +
                                   rtDW.P_k_1[rtDW.P_k_tmp + 2] * rtDW.f1) +
                                  rtDW.P_k_1[rtDW.P_k_tmp + 3] * rtDW.f2) /
                                     (rtDW.U_k_D_m + 1.0F);
    }
  }

  // ��Э������� P_k �����ݸ��Ƶ� P_k_1��Ϊ��һ�ε�����׼����sizeof(real32_T) << 4U ��ʾ���Ƶ��ֽ�����ͨ����һ��4x4����Ĵ�С��ÿ�� real32_T Ϊ4�ֽڣ��ܹ�16�ֽڣ���
  memcpy(&rtDW.P_k_1[0], &rtDW.P_k[0], sizeof(real32_T) << 4U);

  // ����в� gama_t����������� RLS_In_Uq ��ģ��Ԥ�����֮��Ĳ��졣ͨ���������ź� RLS_In_Iq �� RLS_In_We ��״̬���� X_hat_k_1 �Ķ�Ӧֵ��˲���ͣ������ģ�͵������
  rtDW.gama_t = rtU.RLS_In_Uq - (((rtU.RLS_In_Iq * rtDW.X_hat_k_1[0] +
                                   rtDW.U_k_Q[1] * rtDW.X_hat_k_1[1]) +
                                  rtDW.U_k_Q[2] * rtDW.X_hat_k_1[2]) +
                                 (real32_T)rtU.RLS_In_We * rtDW.X_hat_k_1[3]);

  // �� U_k_Q �е�ֵ���� U_k_D_m �� U_k_D_c�����ں����ļ��㡣
  rtDW.U_k_D_m = rtDW.U_k_Q[1];
  rtDW.U_k_D_c = rtDW.U_k_Q[2];

  // ִ�м�Ȩ���
  for (rtDW.i = 0; rtDW.i < 4; rtDW.i++)
  {
    tmp_3 = rtDW.P_k[rtDW.i + 4];
    tmp = rtDW.P_k[rtDW.i];
    tmp_0 = rtDW.P_k[rtDW.i + 8];
    tmp_1 = rtDW.P_k[rtDW.i + 12];
    tmp_2 = rtDW.X_hat_k_1[rtDW.i];
    rtDW.U_k_D[rtDW.i] = ((((((tmp_3 * rtDW.U_k_D_m) + (tmp * rtU.RLS_In_Iq)) + (tmp_0 * rtDW.U_k_D_c)) + (tmp_1 * rtU.RLS_In_We)) * (rtDW.gama_t)) + (tmp_2));
  }

  /*  */
  if (rtDW.U_k_D[0] > 0.0F)
  {
    rtDW.X_hat_k_1[0] = rtDW.U_k_D[0];
    rtDW.X_hat_k_1[1] = rtDW.U_k_D[1];
    rtDW.X_hat_k_1[2] = rtDW.U_k_D[2];
    rtDW.X_hat_k_1[3] = rtDW.U_k_D[3];
  }

  /* Outport: '<Root>/x1' incorporates:
   *  Rs
   */
  rtY.x1 = rtDW.U_k_D[0];

  /* Outport: '<Root>/x2' incorporates:
   *  Ld
   */
  rtY.x2 = rtDW.U_k_D[1];

  /* Outport: '<Root>/x3' incorporates:
   *  Lq
   */
  rtY.x3 = rtDW.U_k_D[2];

  /* Outport: '<Root>/x4' incorporates:
   *  Flux
   */
  rtY.x4 = rtDW.U_k_D[3];
}

/* Model initialize function */
void RLS_initialize(void)
{

  int32_T i;
  static const int16_T tmp[16] = {10000, 0, 0, 0, 0, 10000, 0, 0, 0, 0, 10000,
                                  0, 0, 0, 0, 10000};

       /* ��ʶ������ʼ����ʼ  */
  for (i = 0; i < 16; i++)
  {
    rtDW.P_k_1[i] = tmp[i];
  }

  rtDW.X_hat_k_1[0] = 0.1F;
  rtDW.X_hat_k_1[1] = 0.1F;
  rtDW.X_hat_k_1[2] = 0.1F;
  rtDW.X_hat_k_1[3] = 0.1F;

  /* ��ʶ������ʼ������ */
}
