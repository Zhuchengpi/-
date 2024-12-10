/*
 * File: RLS.c
 *
 */

#include "RLS.h"
#include <math.h>
#include <string.h>
#include "rtwtypes.h"

/*区块信号和状态  */
DW rtDW;

/* 外部输入 */
ExtU rtU;

/* 外部输出 */
ExtY rtY;

/* 实时模型 */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/*
初始化和计算中间变量：

函数首先计算输入信号的差分，存储到 rtDW.U_k_D 和 rtDW.U_k_Q 数组中。
计算 gama_t，这是用于更新协方差矩阵的因子。
更新协方差矩阵：

通过将 P_k_1 除以 gama_t，更新上一步的协方差矩阵，以反映新的估计。
计算参数估计：

通过将协方差矩阵与输入信号相乘，计算参数估计值，并存储在 rtDW.U_k_D 中。
检查 U_k_D[0] 的值，如果大于0，则更新状态变量 X_hat_k_1。
输出估计值：

将最终的参数估计值传递到输出端口 rtY。
*/
/* 模型阶跃函数*/
void RLS_step(void)
{
  float tmp;
  float tmp_0;
  float tmp_1;
  float tmp_2;
  float tmp_3;

  /********* ******************************************
  递归最小二乘（RLS） 输入信号处理和协方差矩阵更新
  *****************************************************/
  rtDW.U_k_D[0] = rtU.RLS_In_Id;                            // 将id储存在U_k_D[0]中
  rtDW.U_k_D[1] = (rtU.RLS_In_Id - rtDW.id_k_1) * 10000.0F; // 将id的变化率乘以10000以增强变化的影响
  rtDW.U_k_D[2] = (real32_T)-rtU.RLS_In_We * rtU.RLS_In_Iq; // 计算负的角速度 (RLS_In_We) 与q轴电流 (RLS_In_Iq) 的乘积，并存储在 U_k_D[2] 中
  rtDW.U_k_D[3] = 0.0F;                                     // 将 U_k_D[3] 初始化为0
  rtDW.U_k_Q[0] = rtU.RLS_In_Iq;                            // 将当前的q轴电流存储在 U_k_Q[0] 中
  rtDW.U_k_Q[1] = (real32_T)rtU.RLS_In_We * rtU.RLS_In_Id;  // 计算角速度与d轴电流的乘积，存储在 U_k_Q[1] 中，用于动态系统的估计。
  rtDW.U_k_Q[2] = (rtU.RLS_In_Iq - rtDW.iq_k_1) * 10000.0F; // 计算q轴电流的变化率，类似于之前对d轴电流的处理，存储在 U_k_Q[2] 中
  rtDW.U_k_Q[3] = (real32_T)rtU.RLS_In_We;                  // 将角速度存储在 U_k_Q[3] 中
  // 更新前一个时刻的电流值，以便在下一次调用中进行差分计算
  rtDW.iq_k_1 = rtU.RLS_In_Iq;
  rtDW.id_k_1 = rtU.RLS_In_Id;
  // 计算 gama_t，用于更新协方差矩阵，基于当前估计的输出与实际输出之间的误差。该值越接近1，表示对过去估计的权重越大。
  rtDW.gama_t = 0.999F - (real32_T)exp((real32_T)fabs(rtU.RLS_In_Uq -
                                                      (((rtU.RLS_In_Iq * rtDW.X_hat_k_1[0] + rtDW.U_k_Q[1] * rtDW.X_hat_k_1[1]) +
                                                        rtDW.U_k_Q[2] * rtDW.X_hat_k_1[2]) +
                                                       (real32_T)rtU.RLS_In_We *
                                                           rtDW.X_hat_k_1[3])) *
                                       -0.999F) *
                             0.001F;

  // 处理协方差矩阵 P_k_1 的更新，将每个元素除以 gama_t，反映新的估计的置信度。
  for (rtDW.i = 0; rtDW.i < 16; rtDW.i++)
  {
    rtDW.P_k_1[rtDW.i] = rtDW.P_k_1[rtDW.i] / rtDW.gama_t;
  }

  /***********************************************************************
   * 递归最小二乘（RLS）估计过程，更新状态变量和协方差矩阵
   *
   *************************************************************************/

  // 初始化 U_k_D_m 为0，并计算 fv1 数组，其中包含基于协方差矩阵 P_k_1 和输入信号的加权和。
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

  // 将 fv1 中的值分配给 y, f, f1, 和 f2。然后利用这些值与 U_k_D 中的每个元素计算并更新 fv 数组。
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

  // 使用 fv 中的值来更新协方差矩阵 P_k，通过当前估计值和过去的协方差进行调整
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

  // 将新计算的协方差矩阵 P_k 复制到 P_k_1。接着计算当前输出误差 y，并使用SIMD指令并行更新 U_k_D，整合协方差和状态估计。
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

  // 协方差矩阵更新
  for (rtDW.i = 0; rtDW.i < 16; rtDW.i++)
  {
    rtDW.P_k_1[rtDW.i] = rtDW.P_k_1[rtDW.i] / rtDW.gama_t;
  }

  // 初始化 U_k_D_m 为0，并计算 fv1 数组，涉及从协方差矩阵 P_k_1 中提取数据，加权与输入信号相乘并求和。更新 U_k_D_m 用于后续的协方差矩阵更新。
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

  // 将 fv1 的值赋给 y, f, f1, 和 f2。接着，使用这些值和 U_k_Q 中的元素更新 fv 数组，以生成新的中间结果。

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

  // 使用 fv 数组中的值更新协方差矩阵 P_k。通过将输入信号的加权和从 P_k_1 中提取，并依据 U_k_D_m 更新协方差，确保算法的稳定性。
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

  // 将协方差矩阵 P_k 的内容复制到 P_k_1，为下一次迭代做准备。sizeof(real32_T) << 4U 表示复制的字节数，通常是一个4x4矩阵的大小（每个 real32_T 为4字节，总共16字节）。
  memcpy(&rtDW.P_k_1[0], &rtDW.P_k[0], sizeof(real32_T) << 4U);

  // 计算残差 gama_t，即期望输出 RLS_In_Uq 与模型预测输出之间的差异。通过将输入信号 RLS_In_Iq 和 RLS_In_We 与状态估计 X_hat_k_1 的对应值相乘并求和，计算出模型的输出。
  rtDW.gama_t = rtU.RLS_In_Uq - (((rtU.RLS_In_Iq * rtDW.X_hat_k_1[0] +
                                   rtDW.U_k_Q[1] * rtDW.X_hat_k_1[1]) +
                                  rtDW.U_k_Q[2] * rtDW.X_hat_k_1[2]) +
                                 (real32_T)rtU.RLS_In_We * rtDW.X_hat_k_1[3]);

  // 将 U_k_Q 中的值赋给 U_k_D_m 和 U_k_D_c，用于后续的计算。
  rtDW.U_k_D_m = rtDW.U_k_Q[1];
  rtDW.U_k_D_c = rtDW.U_k_Q[2];

  // 执行加权求和
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

       /* 辨识参数初始化开始  */
  for (i = 0; i < 16; i++)
  {
    rtDW.P_k_1[i] = tmp[i];
  }

  rtDW.X_hat_k_1[0] = 0.1F;
  rtDW.X_hat_k_1[1] = 0.1F;
  rtDW.X_hat_k_1[2] = 0.1F;
  rtDW.X_hat_k_1[3] = 0.1F;

  /* 辨识参数初始化结束 */
}
