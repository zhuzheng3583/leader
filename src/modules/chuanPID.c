#include "CONTROL.h"
#include "IMU1.h"       
#include "moto.h"
#include "RFdate.h"
#include <math.h>
extern T_RC_Data                         Rc_D;                //遥控通道数据;

extern u8 txbuf[4];         //发送缓冲
extern u8 rxbuf[4];         //接收缓冲
extern u16 test1[3]; //接收到NRf24L01数据
extern S_INT16_XYZ ACC_F,GYRO_F;

PID PID_ROL,PID_PIT,PID_YAW;

extern S_INT16_XYZ        MPU6050_ACC_LAST,MPU6050_GYRO_LAST;       


int Motor_Ele=0;                                           //俯仰期望
int Motor_Ail=0;                                           //横滚期望

//u8 ARMED = 0;

//float rol_i=0,pit_i=0,yaw_p=0;
float thr=0;

S_FLOAT_XYZ EXP_ANGLE ,DIF_ANGLE;
PID1 PID_Motor;
/*********************************/
float Pitch_i,Roll_i,Yaw_i;                         //积分项
float Pitch_old,Roll_old,Yaw_old;                   //角度保存
float Pitch_d,Roll_d,Yaw_d;                         //微分项
float RC_Pitch,RC_Roll,RC_Yaw;                      //姿态角

//外环PID参数
float Pitch_shell_kp=280;//30 140
float Pitch_shell_kd=0;//
float Pitch_shell_ki=0;//
float Roll_shell_kp=250;//30
float Roll_shell_kd=0;//10                 
float Roll_shell_ki=0;//0.08
float Yaw_shell_kp=1.5;//10;//30
float Yaw_shell_kd=0;//10                 
float Yaw_shell_ki=0;//0.08;//0.08
float Pitch_shell_out,Roll_shell_out,Yaw_shell_out; //外环总输出
       
//内环PID参数
float Pitch_core_kp=0.040;
float Pitch_core_kd=0.002;////0.007;//0.07;0.008;
float Roll_core_kp=0.040;//;
float Roll_core_kd=0.002;////0.007;//06;//0.07;
float Yaw_core_kp=0.046;//;
float Yaw_core_kd=0.012;////0.007;//06;//0.07;
float Gyro_radian_old_x,Gyro_radian_old_y,Gyro_radian_old_z;//陀螺仪保存
float pitch_core_kp_out,pitch_core_kd_out,Roll_core_kp_out,Roll_core_kd_out,Yaw_core_kp_out,Yaw_core_kd_out;//内环单项输出
float Pitch_core_out,Roll_core_out,Yaw_core_out;//内环总输出     

int16_t moto1=0,moto2=0,moto3=0,moto4=0;

float tempjd=0;
void CONTROL(float rol, float pit, float yaw)
{
    ////////////////////////外环角度环(PID)///////////////////////////////
    RC_Pitch = (Rc_D.PITCH - 1500) / 20;
    Pitch_i += (Q_ANGLE.Pitch - RC_Pitch);
    //-------------Pitch积分限幅----------------//
    if(Pitch_i > 300) Pitch_i = 300;
    else if(Pitch_i < -300) Pitch_i = -300;
    //-------------Pitch微分--------------------//
    Pitch_d = Q_ANGLE.Pitch - Pitch_old;
    //-------------Pitch  PID-------------------//
    Pitch_shell_out = Pitch_shell_kp*(Q_ANGLE.Pitch - RC_Pitch) + Pitch_shell_ki*Pitch_i + Pitch_shell_kd*Pitch_d;
    //角度保存
    Pitch_old = Q_ANGLE.Pitch;      
       
    RC_Roll = (Rc_D.ROLL - 1500) / 20;
    Roll_i += (Q_ANGLE.Rool - RC_Roll); 
    //-------------Roll积分限幅----------------//
    if(Roll_i > 300) Roll_i = 300;
    else if(Roll_i < -300) Roll_i = -300;
    //-------------Roll微分--------------------//
    Roll_d = Q_ANGLE.Rool - Roll_old;
    //-------------Roll  PID-------------------//
    Roll_shell_out = Roll_shell_kp*(Q_ANGLE.Rool - RC_Roll) + Roll_shell_ki*Roll_i + Roll_shell_kd*Roll_d;
    //------------Roll角度保存------------------//
    Roll_old = Q_ANGLE.Rool;
       
       
    RC_Yaw=(Rc_D.YAW-1500)*10;
    //-------------Yaw微分--------------------//
    Yaw_d=MPU6050_GYRO_LAST.Z-Yaw_old;
    //-------------Roll  PID-------------------//
    Yaw_shell_out  = Yaw_shell_kp*(MPU6050_GYRO_LAST.Z-RC_Yaw) + Yaw_shell_ki*Yaw_i + Yaw_shell_kd*Yaw_d;
    //------------Roll角度保存------------------//
    Yaw_old=MPU6050_GYRO_LAST.Z;
       
       
    ////////////////////////内环角速度环(PD)///////////////////////////////       
    pitch_core_kp_out = Pitch_core_kp * (Pitch_shell_out + MPU6050_GYRO_LAST.Y * 3.5);
    pitch_core_kd_out = Pitch_core_kd * (MPU6050_GYRO_LAST.Y   - Gyro_radian_old_y);
    Roll_core_kp_out  = Roll_core_kp  * (Roll_shell_out  + MPU6050_GYRO_LAST.X *3.5);
    Roll_core_kd_out  = Roll_core_kd  * (MPU6050_GYRO_LAST.X   - Gyro_radian_old_x);
    Yaw_core_kp_out  = Yaw_core_kp  * (Yaw_shell_out  + MPU6050_GYRO_LAST.Z * 1);
    Yaw_core_kd_out  = Yaw_core_kd  * (MPU6050_GYRO_LAST.Z   - Gyro_radian_old_z);
       
    Pitch_core_out = pitch_core_kp_out + pitch_core_kd_out;
    Roll_core_out  = Roll_core_kp_out  + Roll_core_kd_out;
    Yaw_core_out   = Yaw_core_kp_out   + Yaw_core_kd_out;

    Gyro_radian_old_y = MPU6050_GYRO_LAST.X;
    Gyro_radian_old_x = MPU6050_GYRO_LAST.Y;
    Gyro_radian_old_z = MPU6050_GYRO_LAST.Z;   //储存历史值
       
//--------------------将输出值融合到四个电机--------------------------------//

       
        if(Rc_D.THROTTLE>1020)
        {
  thr=Rc_D.THROTTLE- 1000;

//                if(Rc_D.THROTTLE<=2000)
//                {
//  moto1=(int16_t)(thr  - Pitch_core_out);//- yaw);
//        moto2=(int16_t)(thr  - Pitch_core_out);//+ yaw);       
//        moto3=(int16_t)(thr  + Pitch_core_out);// - yaw);
//        moto4=(int16_t)(thr  + Pitch_core_out);//+ yaw);       
   
//  moto1=(int16_t)(thr  - Roll_core_out);//- yaw);
//        moto2=(int16_t)(thr  + Roll_core_out);//+ yaw);       
//        moto3=(int16_t)(thr  + Roll_core_out);// - yaw);
//        moto4=(int16_t)(thr  - Roll_core_out);//+ yaw);

//  moto1=(int16_t)(thr  - Yaw_core_out);//- yaw);
//        moto2=(int16_t)(thr  + Yaw_core_out);//+ yaw);       
//        moto3=(int16_t)(thr  - Yaw_core_out);// - yaw);
//        moto4=(int16_t)(thr  + Yaw_core_out);//+ yaw);                       
                       
//moto1=(int16_t)(thr - Roll_core_out - Pitch_core_out);
//moto2=(int16_t)(thr + Roll_core_out - Pitch_core_out);       
//moto3=(int16_t)(thr + Roll_core_out + Pitch_core_out);
//moto4=(int16_t)(thr - Roll_core_out + Pitch_core_out);       
//                       
  moto1=(int16_t)(thr - Roll_core_out - Pitch_core_out- Yaw_core_out);
        moto2=(int16_t)(thr + Roll_core_out - Pitch_core_out+ Yaw_core_out);       
        moto3=(int16_t)(thr + Roll_core_out + Pitch_core_out- Yaw_core_out);
        moto4=(int16_t)(thr - Roll_core_out + Pitch_core_out+ Yaw_core_out);                       
                       
//                }
  }
        else
        {
                moto1 = 0;
                moto2 = 0;
                moto3 = 0;
                moto4 = 0;
        }
        MOTO_PWMRFLASH(moto1,moto2,moto3,moto4);//        Moto_PwmRflash(moto1,moto2,moto3,moto4);
}











/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
* 文件名 ：ANO_FlyControl.cpp
* 描述 ：飞行控制
**********************************************************************************/
include "ANO_FlyControl.h"ANO_FlyControl fc;

/*
先整定内环，后整定外环。
参数整定找最佳，从小到大顺序查
先是比例后积分，最后再把微分加
曲线振荡很频繁，比例度盘要放大
曲线漂浮绕大湾，比例度盘往小扳
曲线偏离回复慢，积分时间往下降
曲线波动周期长，积分时间再加长
曲线振荡频率快，先把微分降下来
动差大来波动慢。微分时间应加长
理想曲线两个波，前高后低4比1
*/

/*
ROLL和PIT轴向按照以上公式计算PID输出，但YAW轴比较特殊，因为偏航角法线方向刚好和地球重力平行，
这个方向的角度无法由加速度计直接测得，需要增加一个电子罗盘来替代加速度计。如果不使用罗盘的话，
我们可以单纯的通过角速度积分来测得偏航角，缺点是由于积分环节中存在积分漂移，偏航角随着时间的推移
会偏差越来越大。我们不使用罗盘就没有比例项，只仅使用微分环节来控制。
*/

ANO_FlyControl::ANO_FlyControl()
{
    yawRate = 120;
    //重置PID参数
    PID_Reset();
}

//重置PID参数
void ANO_FlyControl:ID_Reset(void)
{
    //因为YAW角度会漂移，所以参数和ROLL、PITCH不一样
    pid[PIDROLL].set_pid(70, 15, 120, 2000000); //ROLL角度的内环控制系数,20000:积分上限 
    pid[PIDPITCH].set_pid(70, 30, 120, 2000000);//PITCH角度的内环控制系数
    pid[PIDYAW].set_pid(100, 50, 0, 2000000); //YAW角度的内环控制系数

    pid[PIDLEVEL].set_pid(280, 0, 0, 0); //外环控制系数
    pid[PIDMAG].set_pid(15, 0, 0, 0); //电子罗盘控制系数
}

/* 
【扫盲知识】 
串级PID：采用的角度P和角速度PID的双闭环PID算法------>角度的误差被作为期望输入到角速度控制器中 （角度的微分就是角速度） 
对于本系统则采用了将角度控制与角速度控制级联的方式组成整个串级 PID 控制器。

串级 PID 算法中，角速度内环占着极为重要的地位。在对四旋翼飞行的物理模型进
行分析后，可以知道造成系统不稳定的物理表现之一就是不稳定的角速度。
因此，若能够直接对系统的角速度进行较好的闭环控制，必然会改善系统的动态特性
及其稳定性，通常也把角速度内环称为增稳环节。而角度外环的作用则体现在对四旋翼飞
行器的姿态角的精确控制。 
外环：输入为角度,输出为角速度
内环：输入为角速度，输出为PWM增量
使用串级pid，分为：角度环控制pid环，和角速度控制环稳定环。主调为角度环（外环），副调为角速度环（内环）。
参数整定原则为先内后外，故在整定内环时将外环的PID均设为0
所谓外环就是只是一个P在起作用，也就是比例在起作用；P也就是修正力度，越大越容易使飞机震荡。 
震荡的特点是：频率小、幅度大
*/

/*
【横滚（Roll）和俯仰（Pitch）的控制算法】 
横滚（Roll）和俯仰（Pitch）的控制算法是一样的，控制参数也比较接近。

首先得到轴姿态的角度差（angle error），将这个值乘以角度系数p
后限幅（限幅必须有，否则剧烈打舵时容易引发震荡）作为角速度控制器期望值（target_rate）。target_rate
与陀螺仪得到的当前角速度作差，得到角速度误差（rate_error）乘以kp得到P。在I值小于限幅值（这个值大概在5%油门）或者
rate_error与i值异号时将rate_error累加到I中。前后两次rate_error的差作为D项，值得注意的是加需要入20hz
（也可以采用其它合适频率）滤波，以避免震荡。将P,I,D三者相加并限幅（50%油门）得到最终PID输出。
*/

//串环PID调节详情参见：http://blog.csdn.net/super_mic ... 36723 

//飞行器姿态外环控制
void ANO_FlyControl::Attitude_Outter_Loop(void)
{
    int32_t errorAngle[2];
    Vector3f Gyro_ADC;

    //计算角度误差值, 角度误差值=期望值-此刻姿态值 
    //constrain_int32作用：32位整型数限幅，使其控制输入的最大飞行倾角不大于25度（如果控制量比25度大，飞机早就坠毁了）
    //rc.Command[ROLL]：遥控数据 imu.angle.x ：此刻姿态(角度)
    //1.得到轴姿态的角度差（errorAngle）
    //2.这个角度差值进行限幅(constrain_int32)（正负FLYANGLE_MAX）
    //（限幅必须有，否则剧烈打舵时容易引发震荡）作为角速度控制器期望值（target_rate） 
    errorAngle[ROLL] = constrain_int32((rc.Command[ROLL] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.x * 10; 
    errorAngle[PITCH] = constrain_int32((rc.Command[PITCH] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.y * 10; 

    //获取此时陀螺仪上的角速度，取角速度的四次平均值 
    Gyro_ADC = mpu6050.Get_Gyro() / 4;
    /* 
    得到外环PID输出（角速度的差值）(实质是相当于内环的P比例项)--------> 
    3.target_rate与陀螺仪得到的当前角速度作差，得到角速度误差（RateError）乘以kp（外环控制系数 pid[PIDLEVEL]--->(280, 0, 0
    , 0)）得到给内环的P。
    */ 

    //横滚roll：外环控制。输入为角度,输出为角速度。RateError[ROLL] 作为内环的输入。
    RateError[ROLL] = pid[PIDLEVEL].get_p(errorAngle[ROLL]) - Gyro_ADC.x; //Gyro_ADC.x:陀螺仪X轴的值 
    //俯仰pitch：外环控制。输入为角度,输出为角速度。RateError[PITCH] 作为内环的输入。
    RateError[PITCH] = pid[PIDLEVEL].get_p(errorAngle[PITCH]) - Gyro_ADC.y;//Gyro_ADC.y:陀螺仪Y轴的值

    /*
    偏航（Yaw）的控制算法和前两者略有不同，是将打舵量（遥控数据量rc.Command[YAW]）和角度误差的和作为角速度内环的期望值，
    这样可以获得更好的动态响应。角速度内环和横滚与俯仰的控制方法一致，参数（积分限幅值会很小，默认只有万分之8）上有不同。*/

    //航向yaw：外环控制。输入为角度,输出为角速度。 RateError[YAW] 作为内环的输入。
    RateError[YAW] = ((int32_t)(yawRate) * rc.Command[YAW]) / 32 - Gyro_ADC.z; //Gyro_ADC.z:陀螺仪Z轴的值

}

//飞行器姿态内环控制: 输入为角速度，输出为PWM增量
//内环的效果就是：减小 P比例控制带来的震荡
void ANO_FlyControl::Attitude_Inner_Loop(void)
{
    int32_t PIDTerm[3];

    //注意这里是i的值是0到2
    //PIDROLL、PIDPITCH、PIDYAW是枚举类型，也就是0、1、2，也就是下面的pid 、PIDTerm就是3组PID
    for(u8 i=0; i<3;i++)
    {
        //现象：当油门低于检查值时积分清零，重新积分

        //猜测：这里应该是担心飞机没飞起来时就开始有积分，会导致起飞时不稳定
        if ((rc.rawData[THROTTLE]) < RC_MINCHECK)
            pid.reset_I();

        //get_pid函数：return get_p(error) + get_i(error, dt) + get_d(error, dt);-------->这里实际就是一个完整的PID
        //PID_INNER_LOOP_TIME：2000us--->0.2ms 积分微分时间，每隔0.2ms操作积分和微分,RateError是外环计算的结果（从外环算出） 
        //得到内环PID输出，直接输出转为电机控制量 
        PIDTerm = pid.get_pid(RateError, PID_INNER_LOOP_TIME);
    }

    //对YAW角继续处理，加入遥控控制 
    //在I值小于限幅值（这个值大概在5%油门）或者rate_error与i值异号时将rate_error累加到I中。
    PIDTerm[YAW] = -constrain_int32(PIDTerm[YAW], -300 - abs(rc.Command[YAW]), +300 + abs(rc.Command[YAW])); 

    //PID输出转为电机控制量
    motor.writeMotor(rc.Command[THROTTLE], PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
}

/*
【调节串环PID大概过程（注意修正反向）】 

1、估计大概的起飞油门。
2、调整角速度内环参数。
3、将角度外环加上，调整外环参数。
4、横滚俯仰参数一般可取一致，将飞机解绑，抓在手中测试两个轴混合控制的效果（注意安全），有问题回到“烤四轴”继续调整，直�
练苫谑种胁换岢榇ぁ�
5、大概设置偏航参数（不追求动态响应，起飞后头不偏即可），起飞后再观察横滚和俯仰轴向打舵的反应，如有问题回到“烤四轴”。
6、横滚和俯仰ok以后，再调整偏航轴参数以达到好的动态效果。
*/ 

/*
【过程详解】

1、要在飞机的起飞油门基础上进行PID参数的调整，否则“烤四轴”的时候调试稳定了，飞起来很可能又会晃荡。
2、内环的参数最为关键！理想的内环参数能够很好地跟随打舵（角速度控制模式下的打舵）控制量。
在平衡位置附近（正负30度左右），舵量突加，飞机快速响应；舵量回中，飞机立刻停止运动（几乎没有回弹和震荡）。
2.1首先改变程序，将角度外环去掉，将打舵量作为内环的期望（角速度模式，在APM中叫ACRO模式，在大疆中叫手动模式）。
2.2加上P，P太小，不能修正角速度误差表现为很“软”倾斜后难以修正，打舵响应也差。P太大，在平衡位置容易震荡，
打舵回中或给干扰（用手突加干扰）时会震荡。合适的P
能较好的对打舵进行响应，又不太会震荡，但是舵量回中后会回弹好几下才能停止（没有D）。
2.3加上D，D的效果十分明显，加快打舵响应，最大的作用是能很好地抑制舵量回中后的震荡，可谓立竿见影。
太大的D会在横滚俯仰混控时表现出来（尽管在“烤四轴”时的表现可能很好），具体表现是四轴抓在手里推油门会抽搐。
如果这样，只能回到“烤四轴”降低D，同时P也只能跟着降低。D调整完后可以再次加大P值，以能够跟随打舵为判断标准。
2.4加上I，会发现手感变得柔和了些。由于笔者“烤四轴”的装置中四轴的重心高于旋转轴，这决定了在四轴偏离水平位置后
会有重力分量使得四轴会继续偏离平衡位置。I的作用就可以使得在一定角度范围内（30度左右）可以修正重力带来的影响。
表现打舵使得飞机偏离平衡位置，舵量回中后飞机立刻停止转动，若没有I或太小，飞机会由于重力继续转动。

3、角度外环只有一个参数P。将外环加上（在APM中叫Stabilize模式，在大疆中叫姿态模式）。打舵会对应到期望的角度。
P的参数比较简单。太小，打舵不灵敏，太大，打舵回中易震荡。以合适的打舵反应速度为准。

4、至此，烤四轴”效果应该会很好了，但是两个轴混控的效果如何还不一定，有可能会抽（两个轴的控制量叠加起来，
特别是较大的D，会引起抽搐）。如果抽了，降低PD的值，I基本不用变。

5、加上偏航的修正参数后（直接给双环参数，角度外环P和横滚差不多，内环P比横滚大些，I和横滚差不多，D可以先不加），
拿在手上试过修正和打舵方向正确后可以试飞了（试飞很危险！！！！选择在宽敞、无风的室内，1
米的高度（高度太低会有地面效应干扰，
太高不容易看清姿态且容易摔坏），避开人群的地方比较适合，如有意外情况，立刻关闭油门！！！
5.1试飞时主要观察这么几个方面的情况，一般经过调整的参数在平衡位置不会大幅度震荡，需要观察：
5.1.1在平衡位置有没有小幅度震荡（可能是由于机架震动太大导致姿态解算错误造成。也可能是角速度内环D的波动过大，
前者可以加强减震措施，传感器下贴上3M胶，必要时在两层3M泡沫胶中夹上“减震板”，注意：铁磁性的减震板会干扰磁力计读数；
后者可以尝试降低D项滤波的截止频率）。
5.1.2观察打舵响应的速度和舵量回中后飞机的回复速度。
5.1.3各个方向（记得测试右前，左后等方向）大舵量突加输入并回中时是否会引起震荡。
如有，尝试减小内环PD也可能是由于“右前”等混控方向上的舵量太大造成。

6、横滚和俯仰调好后就可以调整偏航的参数了。合适参数的判断标准和之前一样，打舵快速响应，舵量回中飞机立刻停止转动（参数D
的作用）。

至此，双环PID参数调节完毕！祝爽飞！
*/