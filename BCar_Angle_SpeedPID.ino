/*----------小车器件清单---------------------------------
  控制器：Arduino NANO *1
  马达：JGA25-370B编码器直流电机（12V） *2  （建议大家选6V的电机）
  马达驱动模块：DRV8833 *1
  通信模块：HC-05蓝牙模块*1
  陀螺仪模块：MPU6050 *1
  LM2596S稳压模块 *1
  -----------陀螺仪测试参数------------------------------
  小车静态机械平衡时的角度为1.5度。
  -----------编码器电机测试参数---------------------------
  左马达A， PWM 32起转  右马达B，PWM 50起转
  -----------PID调试结果--------------------------------
  角度PID  调试结果  P:18   I:0.8  D:0.8
  速度PID  调试结果  P:20     I:0.1
  -----------------------------------------------------
*/

/*-------加载各种库文件-------*/
#include <Wire.h>                                         //I2C通信库，单片机与陀螺仪通信
#include <SoftwareSerial.h>                               //软串口通信库，单片机与蓝牙通信
#include <MPU6050_tockn.h>                                //陀螺仪库，获取陀螺仪相关参数
#include <MsTimer2.h>                                     //定时中断库，使用MsTimer2库文件不能使用引脚PIN3、PIN11的PWM输出

/*-------设置Arduino引脚号-------*/
#define Left_PinA1         10                             //定义左轮驱动DRV8833引脚A1
#define Left_PinA2         9                              //定义左轮驱动DRV8833引脚A2
#define Left_SpeedA        7                              //定义左轮编码器测速引脚
#define Right_PinB1        5                              //定义右轮方向DRV8833引脚B1
#define Right_PinB2        6                              //定义右轮步进DRV8833引脚B2
#define Right_SpeedB       8                              //定义右轮编码器测速引脚
#define RX_Pin             2                              //定义软串口RX针脚
#define TX_Pin             3                              //定义软串口TX针脚
#define SDA_Pin            A4                             //定义陀螺仪SDA针脚库默认
#define SCL_Pin            A5                             //定义陀螺仪SCL针脚库默认
#define LStar_PWM          40                             //测试出左电机起转PWM补偿值42
#define RStar_PWM          54                             //测试出右电机起转PWM补偿值55 
#define Drive_ANG          1.4                            //定义小车前进后退角度数值，数值越大冲的越快 
#define TRUN               160                            //定义小车转动数值 
#define TIMEX              10                             //定义时间间隔

/*-------实例化蓝牙串口和陀螺仪模块-------*/
SoftwareSerial Bluetooth (RX_Pin, TX_Pin);                 //初始化软通信串口RX, TX,对应蓝牙串口对象
MPU6050 Mpu6050(Wire);                                     //初始化Mpu6050对象

/*-------定义调试和预设变量-------*/
float Balance_ANG = 1.25;                                  //测试出的静态机械平衡角度。
float ANG_Kp = 18, ANG_Ki = 0.9, ANG_Kd = 0.8;           //反复调试得到角度环的Kp Ki Kd的值
float SPD_Kp = 20 , SPD_Ki = 0.1;                          //反复调试得到速度环的Kp Ki Kd的值  通常SPD_Ki = SPD_Kp / 200
float Turn_Kp = 0.6, Turn_SPD;                             //转向环Kp值通常为0.6

/*-------定义平衡车控制变量-------*/
float Keep_Angle = 1.25, ANG_DIF_Val, ANG_INTG_Val;          //平衡车需要保持的角度，存在的角度偏差，偏差积分变量
float AngleX, AngleY, GyroX, GyroZ, AccZ;                   //Mpu6050输出的角度值为浮点数，两位有效小数
float Spd_Last;                                             //上一次的速度数值
int   ANG_PWM, SPD_PWM, Turn_PWM, TOT_PWM;                  //定义角度PWM、速度PWM、转向PWM和合计PWM
long  SPD_INTG_ValA, SPD_INTG_ValB;                         //定义速度积分变量
int   SPD_A = 0, SPD_B = 0, Mark_A = 0, Mark_B = 0;         //定义速度脉冲数和

/*-------定义角度环PID调试程序输出电机电压PWM数值-------*/
/* 用陀螺仪返回的数据计算直立PID的PWM
   前倾陀螺仪X轴为正，后仰陀螺仪X轴为负
   车子前后移动保持平衡状态
*/
void AnglePID_PWMCount() {                                          //计算电机转动需要的PWM数值
  Mpu6050.update();                                                 //刷新陀螺仪数据
  AngleX = Mpu6050.getAngleY();                                     //获取陀螺仪X方向角度
  AngleY = Mpu6050.getAngleX();                                     //获取陀螺仪Y方向角度
  GyroX = Mpu6050.getGyroX();                                       //获取陀螺仪X方向角加速度
  if ((abs(AngleX) <= 45) && (abs(AngleY) < 5)) {                   //如果小车前后倾斜角度小于45°,并且左右倾角小于5°则运行
    ANG_DIF_Val = AngleX - Keep_Angle;                              //计算小车偏转角度与静态平衡角度的差值。
    ANG_INTG_Val += ANG_DIF_Val;                                    //计算角度偏差的积分，INTG_Val为全局变量，一直积累
    ANG_INTG_Val = constrain(ANG_INTG_Val, -1000, 1000);            //限定误差积分的最大和最小值
    ANG_PWM = ANG_Kp * ANG_DIF_Val + ANG_Ki * ANG_INTG_Val + ANG_Kd * GyroX; //通过调节PID计算角度环PWM数值
  }
  else ANG_PWM = 0;
}

/*-------定义速度环PID调试程序输出电机电压PWM数值-------*/
/* 通过电机转动算出速度环PID的PWM
   前进左轮A速度环为正，右纶B速度环为负
   用于辅助小车尽快平衡
*/
void SpeedPID_PWMCount(int Spd_LA, int Spd_RB) {             //车轮位移PID计算PWM，通过电机编码器返回数据计算
  float Spd_Val = (Spd_LA + Spd_RB) / 2;                     //消除两轮不一致的误差
  Spd_Val = 0.3 * Spd_Val + 0.7 * Spd_Last ;                 //★增加速度环一阶滤波器，让数值平缓过渡
  Spd_Last = Spd_Val;
  SPD_INTG_ValA += Spd_LA;
  SPD_INTG_ValB += Spd_RB;
  SPD_INTG_ValA = constrain(SPD_INTG_ValA, -1000, 1000);       //限定误差积分的最大和最小值
  SPD_INTG_ValB = constrain(SPD_INTG_ValB, -1000, 1000);       //限定误差积分的最大和最小值
  long SPD_INTG_Val = (SPD_INTG_ValA + SPD_INTG_ValB) / 2;
  SPD_INTG_Val = constrain(SPD_INTG_Val, -2000, 2000);
  if ((abs(AngleX) <= 45) && (abs(AngleY) < 5)) {
    SPD_PWM = SPD_Kp * Spd_Val + SPD_Ki * SPD_INTG_Val;        //通过调节PI计算速度环PWM数值
  }
  else SPD_PWM = 0;
  if (ANG_PWM < 0) SPD_PWM = -SPD_PWM;                         //★解决单A相测速方向问题
}

/*-------定义转向环PID调试程序输出电机电压PWM数值-------*/
/* 通过陀螺仪Z轴加速度计算小车转向PWM
*/
void TurnPID_PWMCount()                                    //转向PMW计算
{
  GyroZ = Mpu6050.getGyroZ();                              //获取陀螺仪Z轴角速度
  AccZ = Mpu6050.getAccZ();                                //获取陀螺仪Z轴角加速度
  Turn_PWM = Turn_Kp * (Turn_SPD - GyroZ) - Turn_Kp * AccZ ;
}



/*-------★可视化PWM曲线★-------*/
/* 通过IDE串口绘图器，可显示PWM数据波形
   当串口波特率为9600时，数据只能显示
   2条超过2条后小车大幅度晃动，可能是
   通信速率低影响PWM数值计算，提高波
   特率，可以显示多条信息。
*/

void View_PWM() {
  //  Serial.print("SPD_A:"); Serial.print(SPD_A);  Serial.print(',');        //打印出左侧A电机速度值脉冲值
  //  Serial.print("SPD_B:"); Serial.print(SPD_B);  Serial.print(',');        //打印出右侧B电机速度值脉冲值
  Serial.print("直立环:"); Serial.print(ANG_PWM); Serial.print(',');
  Serial.print("速度环:"); Serial.print(-SPD_PWM); Serial.print(',');
  Serial.print("综合值:"); Serial.print(TOT_PWM); Serial.print(',');
  Serial.print("理想值:"); Serial.println(0);
}
void setup() {
  Serial.begin(38400);                                     //打开串行通信,波特率大小影响print数量
  Bluetooth.begin(9600);                                   //蓝牙串口初始化
  Motor_begin() ;                                          //电机马达初始化
  MPU6050_begin();                                         //陀螺仪初始化
  MsTimer2::set(TIMEX, INT_TIMER);                         //10ms触发一次中断，调用程序TIMER()
  MsTimer2::start();                                       //开启定时中断
  while (!Serial) {
    ;
  }
  //  Serial.println("I'm Ready！");
}

void loop() {
  Bluetooth_DEBUG();                                       //手机蓝牙调试控制
  EnCode_Count();                                          //通过编码器计算速度
}
/*-------定义中断程序-------*/
void INT_TIMER() {
  sei();                                                   //允许全局中断
  AnglePID_PWMCount();                                     //角度环PWM计算
  //SpeedPID_PWMCount(SPD_A, SPD_B);                         //速度环PWM计算
  //View_PWM();
  SPD_A = SPD_B = 0;                                       //计数器清零重新计数
  TurnPID_PWMCount();                                      //转向环PWM计算
  TOT_PWM = ANG_PWM - SPD_PWM;                             //串联角度环和速度环
  Car_DRV(TOT_PWM);                                        //小车运动执行
}

/*-------定义编码器计数程序-------*/
void EnCode_Count() {
  if (digitalRead(Left_SpeedA) == 1 && Mark_A == 0) {      //计算霍尔编码器电机速度脉冲数值
    SPD_A++; Mark_A = 1;
  }
  if (digitalRead(Left_SpeedA) == 0 && Mark_A == 1) {
    SPD_A++; Mark_A = 0;
  }
  if (digitalRead(Right_SpeedB) == 1 && Mark_B == 0) {
    SPD_B++; Mark_B = 1;
  }
  if (digitalRead(Right_SpeedB) == 0 && Mark_B == 1) {
    SPD_B++; Mark_B = 0;
  }
}

/*-------定义陀螺仪初始化程序-------*/
void MPU6050_begin() {
  Wire.begin();
  Mpu6050.begin();
  //  Mpu6050.calcGyroOffsets(true);                        //测试陀螺仪补偿值，耗时3秒
  Mpu6050.setGyroOffsets(-0.94, -4.2, -0.08);               //后期可直接设置不用每次都测试
}

/*-------定义马达初始化程序-------*/
void Motor_begin() {
  pinMode(Left_PinA1, OUTPUT);                           //设置与DRV8833连接的引脚都设置为输出
  pinMode(Left_PinA2, OUTPUT);
  pinMode(Right_PinB1, OUTPUT);
  pinMode(Right_PinB2, OUTPUT);
  pinMode(Left_SpeedA, INPUT);                           //设置检测左侧A电机脉冲的引脚为输入
  pinMode(Right_SpeedB, INPUT);                          //设置检测右侧B电机脉冲的引脚为输入
}

/*-------定义小车控制程序-------*/
void Car_DRV(int Pwm) {                                  //两个电机转动方向镜像对称，小车向一个方向行驶
  int Left_PWM, Right_PWM;
  /*-------补偿两轮启动PWM-------*/
  if (Pwm > 0) {
    Left_PWM = Pwm + LStar_PWM;
    Right_PWM = Pwm + RStar_PWM;
  }
  if (Pwm < 0) {
    Left_PWM = Pwm - LStar_PWM;
    Right_PWM = Pwm - RStar_PWM;
  }
  if (Pwm == 0) {
    Left_PWM = 0;
    Right_PWM = 0;
  }

  /*-------控制转向-------*/
  if (Turn_PWM > 0) Left_PWM += Turn_PWM;                 //左转左轮多运动
  if (Turn_PWM < 0) Right_PWM -= Turn_PWM;                //右转右轮多运动

  /*-------电机输出-------*/
  Motor_DRV(Left_PinA1, Left_PinA2, Left_PWM);            //设置左侧电机两个针脚和PWM数值
  Motor_DRV(Right_PinB1, Right_PinB2, -Right_PWM);        //设置右侧电机两个针脚和PWM数值
}

/*-------定义电机转动程序-------*/
void Motor_DRV(int OutPin1, int OutPin2, int Pwm) {        //马达输出程序
  Pwm = constrain(Pwm, -255, 255);                         //限定Pwm区间在-255~255
  if (Pwm >= 0)                                            //如果Pwm大于等于0则电机顺时针转动
  { analogWrite(OutPin1, 0);
    analogWrite(OutPin2, Pwm);
  }
  if (Pwm < 0)                                             //如果Pwm小于0则电机逆时针转动
  { analogWrite(OutPin1, -Pwm);
    analogWrite(OutPin2, 0);
  }
}

/*-------定义蓝牙串口调试和控制程序-------*/
void Bluetooth_DEBUG() {                         //根据手机端发送来的串口数据控制数值增减
  while (Bluetooth.available())                  //当有数据时开始执行
  {
    char Receive_Char = Bluetooth.read();        //读取蓝牙端发送的数据
    switch (Receive_Char)
    {
      /*-------机械平衡点调整-------*/
      case '1': Keep_Angle += 0.01; break;       //调节物理平衡点,w前倾，s后仰
      case '2': Keep_Angle -= 0.01; break;
      /*-------角度环调试-------*/
      case '3': ANG_Kp += 0.5; break;            //调节直立环 比例Kp项-
      case '4': ANG_Kp -= 0.5; break;            //调节直立环 比例Kp项+
      case '5': ANG_Ki += 0.05; break;            //调节直立环 积分项Ki-
      case '6': ANG_Ki -= 0.05; break;            //调节直立环 积分项Ki+
      case '7': ANG_Kd += 0.05; break;            //调节直立环 微分项Kd-
      case '8': ANG_Kd -= 0.05; break;            //调节直立环 微分项Kd+
      /*-------转向环调试-------*/
      case '9': Turn_Kp += 0.1; break;           //调节转向环 比例项+
      case '0': Turn_Kp -= 0.1; break;           //调节转向环 比例项-

      /*-------速度环调试-------*/
      case 'z': SPD_Kp += 2; SPD_Ki = SPD_Kp / 200; break;           //调节速度环比例项-+
      case 'x': SPD_Kp -= 2; SPD_Ki = SPD_Kp / 200; break;
      case 'c': SPD_Ki += 0.1; break;                                //调节速度环积分项-+ ki取值范围0.01-0.25 25级
      case 'v': SPD_Ki -= 0.1; break;

      /*------控制小车转向------------------------*/
      case 'l': Turn_SPD = TRUN;  break;
      case 'r': Turn_SPD = -TRUN; break;
      case 'p': Turn_SPD = 0;     break;

      /*------控制小车前后运行------------------------*/
      case 'w': Keep_Angle = Balance_ANG + Drive_ANG;  break;
      case 'b': Keep_Angle = Balance_ANG - Drive_ANG;  break;
      case 's': Keep_Angle = Balance_ANG; break;
    }
    /*-------调试时PID极性限制-------*/
    if (ANG_Kp < 0)ANG_Kp = 0;
    if (ANG_Ki < 0)ANG_Ki = 0;
    if (ANG_Kd < 0)ANG_Kd = 0;
    if (SPD_Kp < 0)ANG_Kp = 0;
    if (SPD_Ki < 0)ANG_Ki = 0;
    if (Turn_Kp < 0)Turn_Kp = 0;
    /*-------串口打印输出显示-------*/
    Bluetooth.print("Keep_Angle: "); Bluetooth.println(Keep_Angle);
    Bluetooth.print("ANG_Kp:"); Bluetooth.print(ANG_Kp);
    Bluetooth.print("  ANG_Ki:"); Bluetooth.print(ANG_Ki);
    Bluetooth.println("  ANG_Kd:"); Bluetooth.println(ANG_Kd);
    //    Bluetooth.println("--------------");
    //    Bluetooth.print("SPD_Kp:"); Bluetooth.print(SPD_Kp);
    //    Bluetooth.println("  SPD_Ki:"); Bluetooth.println(SPD_Ki);
    //    Bluetooth.println("--------------");
    //    Bluetooth.print("Turn_Kp:"); Bluetooth.println(Turn_Kp);
    //    Bluetooth.println("--------------");
  }
}
