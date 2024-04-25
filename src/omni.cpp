#include "omni.h"

Timer main_timer;

int route_number=0;
double target_x=0,target_y=0,target_yaw=0;
double global_x_caution,global_y_caution;
int omni_xcomp=0,omni_ycomp=0;
double target_direction,target_distance,motor_speed;
int omni_turn_speed=0;

const int Addr_g=0xd2;//0x69
double yaw=0;
double gz=0;
double gz_ave;

const int motor1_address=0b0100;
const int motor2_address=0b0011;
const int motor3_address=0b0010;
const int motor4_address=0b0001;
const int motor_speed_manual=10;

const int encode1_palse=200*4;//PPR*4　エンコーダ1の分解能
const int encode2_palse=200*4;//PPR*4　エンコーダ1の分解能
const double encode1_omni=50.8;//omniの直系
const double encode2_omni=50.8;//omniの直系

const double KP = 3;
const double KI = 0.0;
const double KD = 0.4;
const int pid_limit = 100; //PID制御の最大値　※要調整　モタドラの関係で(0~199)

const double oKP = 0.08;
const double oKI = 0.0002;
const double oKD = 0.005;
const int opid_limit = 10; //PID制御の最大値　※要調整　モタドラの関係で(0~199)

double e_pre = 0.0; //前回の角度の差
double ie = 0.0;    //積分の値保存
double de = 0.0;    //微分の値保存
unsigned long long int t_pre = 0.0; //時間の一時保存

double oe_pre = 0.0; //前回の角度の差
double oie = 0.0;    //積分の値保存
double ode = 0.0;    //微分の値保存
unsigned long long int ot_pre = 0.0; //時間の一時保存
int  omni_speed= 0;   //PID制御出力

const int motor_y_speed=40;
const int motor_x_speed=80;
const int turn_speed=30;

const int encode_table[]={0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
double encode1_count=0,encode2_count=0;
double encode1_angle,encode2_angle;
double local_x,local_y;
double local_last_x,local_last_y;
double global_x,global_y;
double global_mo_x,global_mo_y;

double radian(double degree){

    return degree*M_PI/180.0;

}

void omni_PID(){
    unsigned long long int now = main_timer.read_us();
    double cycle = now - ot_pre;
    ot_pre = now;
    double e  = target_distance;     // 角度の差を計算
    ode = (e - oe_pre)/(cycle/1000000);        // 誤差の微分を近似計算
    oie = oie + (e + oe_pre)*(cycle/1000000)/2; // 誤差の積分を近似計算
    oe_pre = e;                      //前回の差を保存
    omni_speed  = oKP*e + oKI*oie + oKD*ode; // PID制御の式にそれぞれを代入
    if(omni_speed > opid_limit){
        omni_speed = opid_limit;
    }
    if(omni_speed < opid_limit * -1){
        omni_speed = opid_limit * -1;
    }
}

void omni_turn_PID(){
    unsigned long long int now = main_timer.read_us();
    double cycle = now - t_pre;
    t_pre = now;
    double e  = yaw - target_yaw;     // 角度の差を計算
    de = (e - e_pre)/(cycle/1000000);        // 誤差の微分を近似計算
    ie = ie + (e + e_pre)*(cycle/1000000)/2; // 誤差の積分を近似計算
    e_pre = e;                      //前回の差を保存
    omni_turn_speed  = KP*e + KI*ie + KD*de; // PID制御の式にそれぞれを代入
    if(omni_turn_speed > pid_limit){
        omni_turn_speed = pid_limit;
    }
    if(omni_turn_speed < pid_limit * -1){
        omni_turn_speed = pid_limit * -1;
    }
}

void omni_calculation(){
    int motor1_speed=sin(radian(target_direction-30))*motor_speed-omni_turn_speed;
    int motor2_speed=sin(radian(target_direction-150))*motor_speed-omni_turn_speed;
    int motor3_speed=sin(radian(target_direction+150))*motor_speed-omni_turn_speed;
    int motor4_speed=sin(radian(target_direction+30))*motor_speed-omni_turn_speed;

    // motor(motor1_address,motor1_speed);須山先輩のやーつ
    // motor(motor2_address,motor2_speed);
    // motor(motor3_address,motor3_speed);
    // wait_us(70);                         待たないと動かないらしい
    // motor(motor4_address,motor4_speed);
}


void position_cal(){//エンコーダ系の計算　位置や速度など
  encode1_angle=float(encode1_count)*360.0/encode1_palse;
  encode2_angle=float(encode2_count)*360.0/encode2_palse;

  local_x=encode1_omni*M_PI*encode1_angle/360.0;
  local_y=encode2_omni*M_PI*encode2_angle/360.0;

  double posture_z_rad=yaw/(180/M_PI);

  global_mo_x=(local_x-local_last_x)*cos(posture_z_rad)-(local_y-local_last_y)*sin(posture_z_rad);
  global_mo_y=(local_y-local_last_y)*cos(posture_z_rad)+(local_x-local_last_x)*sin(posture_z_rad);

  global_x+=global_mo_x;
  global_y+=global_mo_y;

  local_last_x=local_x;
  local_last_y=local_y;
}