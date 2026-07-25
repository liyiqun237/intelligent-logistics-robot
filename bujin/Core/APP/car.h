#ifndef CAR_H
#define CAR_H
#include "bsp.h"
//**********************************结构体声明*******************************************//
//车辆模式
enum Car_Mode
{
    CAR_ACC = 0,//加速过程
    CAR_AVE =1 ,//匀速过程
    CAR_DEC = 2,//减速过程
    CAR_RUNNING = 3,//运行中
    CAR_STOP = 4,//停止
    CAR_ERROR = 5//错误

};

enum Car_Face
{
  Forward = 0 ,
  Left = 1 ,
  Back = 2 ,
  Right = 3
};

enum Height
{
  Hight_Plate = 0 ,
  Hight_Rough = 1 ,
  Hight_Accurate = 2 ,
  Hight_Running_Plate = 3 ,

  Hight_Plate_Final = 4 ,
  Hight_Rough_Final = 5 ,
  Hight_Accurate_Final = 6 ,
  Hight_Running_Plate_Final = 7
  
};

typedef enum {
    STATE_NORMAL,    // 正常接收数据状态
    STATE_TIMEOUT    // 超时状态
} CommsState;
////在movexyz函数中，用于检测信号超时的函数
// PID控制器
typedef struct {
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    float dt;               // 时间间隔
    float integral;         // 积分项
    float prev_error;       // 上一次的误差
    float max_output;       // 最大输出限制


    uint32_t last_call_time  ;
    float deriv_buf[4] ;
    float last_output ;
    float prev_filtered ;
    float prev_position ;
    bool first_call ;

  } PIDController;
  
  // 车辆状态
typedef struct {
  
    float current_car_Vx ;  // x方向速度(相对于车身自身坐标系)
    float current_car_Vy ;  // 
    float target_car_Vx ;
    float target_car_Vy ;
    
    float delta_car_position_x  ; //车每次相对自身x方向移动量
    float delta_car_position_y  ; 
  
    float target_car_position_x  ;  
    float target_car_position_y  ; 
  
    int16_t target_Vel1 ;
    int16_t target_Vel2 ;
    int16_t target_Vel3 ;
    int16_t target_Vel4 ;
  
    int16_t current_Vel1 ;
    int16_t current_Vel2 ;
    int16_t current_Vel3 ;
    int16_t current_Vel4 ;
  
    float current_map_Vx ;  //绝对速度，指令刚发出时候为绝对坐标系，也就是不同于车身
    float current_map_Vy ;  // 
    float target_map_Vx ;
    float target_map_Vy ;
  
    float  current_map_position_x ; //绝对位置，指令发出时刻为绝对坐标系的位置
    float  current_map_position_y ; 
    float  target_map_position_x ;
    float  target_map_position_y ;
  
    float target_angle_speed ;//目标角速度，逆时针为正，单位°/s
    enum Car_Mode car_mode ;//车辆模式
    enum Car_Face car_face ;//车辆朝向
  
  } Car_Status;



//**********************************全局变量声明*******************************************//

extern uint8_t txcmd_2[16];
extern uint8_t txcmd5[33];
extern uint8_t rxcmd5_dma[33];
extern uint8_t rxcmd5_app[33];
extern float omega_1 ;
extern float omega ;
extern float current_angle_speed ;
extern PIDController* outer_PID_f;
extern PIDController* outer_PID_Y_f;
extern PIDController* outer_PID_Z_f;
extern PIDController* outer_PID_omega_f;
extern PIDController *inner_PID_X ;
extern PIDController *inner_PID_Y ;
extern Car_Status * car ;
extern uint32_t last_reveive_motor_time;
extern uint32_t last_receive_hwt_time;
extern volatile uint32_t last_receive_raspi_time;

//**********************************函数声明*******************************************//
void omega_zero();
float PID_calc(float error_position , PIDController * PID);
float PID_calc_Y(float error_position , PIDController * PID);
float PID_calc_z(float target_position, float init_omega, PIDController *PID);
void PIDController_reset(PIDController* self);
//位置模式
void Move_TransfromX(float X);
void Move_TransfromY(float Y);
void Move_TransfromZ(float Z);
void Move_TransfromXY(float XY);//刚开始的向左前，不过改版之后需要向左后

//速度-位置闭环
uint8_t Move_To_Position_Z(float target_omega , uint32_t TIMEOUT);
uint8_t Move_To_Position_Y(float target_position_y, uint32_t TIMEOUT);
uint8_t Move_To_Position_X(float target_position_x, uint32_t TIMEOUT);

void Move_To_Position_XYZ(float target_position_x, float target_position_y, float target_omega, uint32_t TIMEOUT);

void Move_To_Position_XYZ_Color(float target_position_x, float target_position_y, float target_omega, uint32_t TIMEOUT);

void Turn_To(float target_omega , float angle_error_limit , uint32_t TIMEOUT);
///这个height取物块和码垛时候的参数，分别设置为0和1
void Car_Blog(uint32_t TIMEOUT , int8_t height);

void Car_Ring( uint32_t TIMEOUT );

void Car_Ring_Timeout( uint32_t TIMEOUT );

void Car_Blog_Test(uint32_t TIMEOUT, enum Height height);

void Car_Ring_Timeout_Test( uint32_t TIMEOUT , enum Height height );

void Car_Ring_Test(uint32_t TIMEOUT);

void Car_Ring_Fast(uint32_t TIMEOUT);

void Car_Ring_Timeout_Fast( uint32_t TIMEOUT , enum Height height );




//获取电机状态
void From_Motor_to_Car_Status(void);
void Update_Car_Status(uint32_t delay_ms) ;
void Reset_Car_Status(void);
void Publish_motor_speed(void);

/**
 * @brief 小车位姿闭环控制函数
 * @param x 目标世界坐标x，单位mm
 * @param y 目标世界坐标y，单位mm
 * @param angle 目标方向角，单位deg
 */
void CarMoveTo(float x, float y, float angle , uint32_t TIMEOUT ) ;


/**
 * @brief 实现小车在指定方向上的平移运动
 * @param dirc：平移方向，机器人坐标系，0=X方向，1=Y方向
 * @param v：平移速度，单位mm/s
 * @param distance：平移距离，单位mm
 * @param eA：角度控制精度，单位度
 * @param pA：角度控制比例系数
 */
void CarTrans(int dirc, float v, float distance, float eA, float pA);


/**
 * @brief 实现小车的侧向漂移运动
 * @param r：漂移半径，单位mm
 * @param angle：漂移角度，单位度，正值为朝外漂移
 * @param v：漂移速度，左为正，单位mm/s
 */
void CarDrift(float r, float angle, float v);
// 辅助函数：符号函数
float sign(float val);

#endif