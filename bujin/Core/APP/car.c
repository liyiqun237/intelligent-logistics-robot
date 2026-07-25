#include "car.h"
#include <stdatomic.h>
#include <math.h>
////////car_move_tp的宏定义
// 定义指数非线性控制所需常量
#define PI 3.14159265358979323846f
#define POS_PRECISION_X 9.0f // 位置精度阈值X，单位mm
#define POS_PRECISION_Y 9.0f // 位置精度阈值Y，单位mm
/// 角度的误差进行了修改
/// 角度的误差阈值需要进行修改
#define ANG_PRECISION 0.8f // 角度精度阈值，单位deg

//
//
//                     B      C
//
// 源代码里的轮子相对位置   A      D
//
// 我们的里面就是     1         2
//
//                     3         4
//
// 方向      正前方X
//           左Y
//           逆时针Z
//
//

// 角度校准为0
void omega_zero()
{
    uint8_t txcmd_5[16] = {0};
    txcmd_5[0] = 0xFF;
    txcmd_5[1] = 0xAA;
    txcmd_5[2] = 0x00;
    txcmd_5[3] = 0xFF;
    txcmd_5[4] = 0x00;
    HAL_UART_Transmit(&huart5, txcmd_5, 5, 100);
}

//************************************************************************************************************ */
// 以下是pid控制器的实现部分
////////////////////
///////////////


PIDController *outer_PID_f = NULL;
PIDController *outer_PID_Y_f = NULL;
PIDController *outer_PID_Z_f = NULL;
PIDController *outer_PID_omega_f = NULL;
PIDController *inner_PID_X = NULL;
PIDController *inner_PID_Y = NULL;
PIDController *inner_PID_X_Col = NULL;
PIDController *inner_PID_Y_Col = NULL;
PIDController *outer_PID_f_Fast = NULL;
PIDController *outer_PID_Y_f_Fast = NULL;
PIDController *outer_PID_Z_f_Fast = NULL;  


void initializePIDController(void)
{

    // 为outer_PID_f分配内存并初始化
    outer_PID_f = (PIDController *)malloc(sizeof(PIDController));
    if (outer_PID_f != NULL)
    {
        outer_PID_f->kp = 12.0f / 10.345f;         // 原值: 0.4f  50
        outer_PID_f->ki = 1.2f * 5.0f / 10.345f;          // 原值: 0.0f
        outer_PID_f->kd = 1.2f * 4.0f / 10.345f;   // 原值: 0.2f
        outer_PID_f->dt = 0.05f;                   // dt 不需要调整
        outer_PID_f->integral = 0.0f;              // 积分项初始值
        outer_PID_f->prev_error = 0.0f;            // 上一次误差初始值
        outer_PID_f->max_output = 10.0f ; // 原值: 80.0f

        outer_PID_f->last_call_time = HAL_GetTick();
        memset(outer_PID_f->deriv_buf, 0, sizeof(outer_PID_f->deriv_buf));
        outer_PID_f->last_output = 0;
        outer_PID_f->prev_filtered = 0;
        outer_PID_f->integral = 0;
        outer_PID_f->prev_position = 0;
        outer_PID_f->first_call = true;
    }

    // 为outer_PID_Y_f分配内存并初始化
    outer_PID_Y_f = (PIDController *)malloc(sizeof(PIDController));
    if (outer_PID_Y_f != NULL)
    {
        outer_PID_Y_f->kp = 18.0f / 10.345f;         // 原值: 0.4f    75
        outer_PID_Y_f->ki = 1.8f * 5.0f / 10.345f;          // 原值: 0.0f
        outer_PID_Y_f->kd = 1.8f * 4.0f / 10.345f;   // 原值: 0.2f
        outer_PID_Y_f->dt = 0.05f;                   // dt 不需要调整
        outer_PID_Y_f->integral = 0.0f;              // 积分项初始值
        outer_PID_Y_f->prev_error = 0.0f;            // 上一次误差初始值
        outer_PID_Y_f->max_output = 10.0f ; // 原值: 80.0f

        outer_PID_Y_f->last_call_time = HAL_GetTick();
        memset(outer_PID_Y_f->deriv_buf, 0, sizeof(outer_PID_Y_f->deriv_buf));
        outer_PID_Y_f->last_output = 0;
        outer_PID_Y_f->prev_filtered = 0;
        outer_PID_Y_f->integral = 0;
        outer_PID_Y_f->prev_position = 0;
        outer_PID_Y_f->first_call = true;
    }

    // 为outer_PID_Z_f分配内存并初始化
    outer_PID_Z_f = (PIDController *)malloc(sizeof(PIDController));
    if (outer_PID_Z_f != NULL)
    {
        outer_PID_Z_f->kp = 40.0f / 10.345f;         // 原值: 0.0038f
        outer_PID_Z_f->ki = 4.0f * 5.0f / 10.345f;          // 原值: 0.0f
        outer_PID_Z_f->kd = 12.0f / 10.345f;         // 原值: 0.02f
        outer_PID_Z_f->dt = 0.05f;                   // dt 不需要调整
        outer_PID_Z_f->integral = 0.0f;              // 积分项初始值
        outer_PID_Z_f->prev_error = 0.0f;            // 上一次误差初始值
        outer_PID_Z_f->max_output = 10.0f ; // 原值: 50.0f

        outer_PID_Z_f->last_call_time = HAL_GetTick();
        memset(outer_PID_Z_f->deriv_buf, 0, sizeof(outer_PID_Z_f->deriv_buf));
        outer_PID_Z_f->last_output = 0;
        outer_PID_Z_f->prev_filtered = 0;
        outer_PID_Z_f->integral = 0;
        outer_PID_Z_f->prev_position = 0;
        outer_PID_Z_f->first_call = true;
    }

    // 为outer_PID_omega_f分配内存并初始化
    outer_PID_omega_f = (PIDController *)malloc(sizeof(PIDController));
    if (outer_PID_omega_f != NULL)
    {
        outer_PID_omega_f->kp = 14.4f / 10.345f;  // 原值: 3.8f//改成目前这样，还算行吧，只能说
        outer_PID_omega_f->ki = 0.033f / 10.345f; // 原值: 0.0f
        outer_PID_omega_f->kd = 24.0f / 10.345f;  // 原值: 0.2f
        outer_PID_omega_f->dt = 0.062f;           // dt 不需要调整
        outer_PID_omega_f->integral = 0.0f;       // 积分项初始值
        outer_PID_omega_f->prev_error = 0.0f;     // 上一次误差初始值
        outer_PID_omega_f->max_output = 1100.0f;  // 原值: 300.0f    再次的原来数值，90，听说这个数据就是耦合的很好的了

        outer_PID_omega_f->last_call_time = HAL_GetTick();
        memset(outer_PID_omega_f->deriv_buf, 0, sizeof(outer_PID_omega_f->deriv_buf));
        outer_PID_omega_f->last_output = 0;
        outer_PID_omega_f->prev_filtered = 0;
        outer_PID_omega_f->integral = 0;
        outer_PID_omega_f->prev_position = 0;
        outer_PID_omega_f->first_call = true;
    }

    // 为inner_PID_X分配内存并初始化
    inner_PID_X = (PIDController *)malloc(sizeof(PIDController));
    if (inner_PID_X != NULL)
    {
        // 初始化 PID 参数
        inner_PID_X->kp = 36.0f / 10.345f; // 比例增益1.5
        inner_PID_X->ki = 0.18f / 10.345f; // 积分增益
        inner_PID_X->kd = 18.0f / 10.345f; // 微分增益1.5
        inner_PID_X->dt = 0.05f;           // 采样时间
        inner_PID_X->integral = 0.0f;      // 积分项初始值
        inner_PID_X->prev_error = 0.0f;    // 上一次误差初始值
        inner_PID_X->max_output = 800.0f;  // (现在通过转化变成了mm/s)
        // 这里最后还是需要转换成

        inner_PID_X->last_call_time = HAL_GetTick();
        memset(inner_PID_X->deriv_buf, 0, sizeof(inner_PID_X->deriv_buf));
        inner_PID_X->last_output = 0;
        inner_PID_X->prev_filtered = 0;
        inner_PID_X->integral = 0;
        inner_PID_X->prev_position = 0;
        inner_PID_X->first_call = true;
    }

    inner_PID_Y = (PIDController *)malloc(sizeof(PIDController));
    if (inner_PID_Y != NULL)
    {
        // 初始化 PID 参数
        inner_PID_Y->kp = 38.0f / 10.345f; // 比例增益
        inner_PID_Y->ki = 0.2f / 10.345f;  // 积分增益
        inner_PID_Y->kd = 12.0f / 10.345f; // 微分增益
        inner_PID_Y->dt = 0.05f;           // 采样时间
        inner_PID_Y->integral = 0.0f;      // 积分项初始值
        inner_PID_Y->prev_error = 0.0f;    // 上一次误差初始值
        inner_PID_Y->max_output = 800.0f;  // 输出限幅

        inner_PID_Y->last_call_time = HAL_GetTick();
        memset(inner_PID_Y->deriv_buf, 0, sizeof(inner_PID_Y->deriv_buf));
        inner_PID_Y->last_output = 0;
        inner_PID_Y->prev_filtered = 0;
        inner_PID_Y->integral = 0;
        inner_PID_Y->prev_position = 0;
        inner_PID_Y->first_call = true;
    }

    // 为inner_PID_X分配内存并初始化
    inner_PID_X_Col = (PIDController *)malloc(sizeof(PIDController));
    if (inner_PID_X_Col != NULL)
    {
        // 初始化 PID 参数
        inner_PID_X_Col->kp = 36.0f * 0.84 / 10.345f; // 比例增益1.5
        inner_PID_X_Col->ki = 0.18f / 10.345f;        // 积分增益
        inner_PID_X_Col->kd = 18.0f * 1.83 / 10.345f; // 微分增益1.5
        inner_PID_X_Col->dt = 0.05f;                  // 采样时间
        inner_PID_X_Col->integral = 0.0f;             // 积分项初始值
        inner_PID_X_Col->prev_error = 0.0f;           // 上一次误差初始值
        inner_PID_X_Col->max_output = 800.0f;         // (现在通过转化变成了mm/s)
        // 这里最后还是需要转换成

        inner_PID_X_Col->last_call_time = HAL_GetTick();
        memset(inner_PID_X_Col->deriv_buf, 0, sizeof(inner_PID_X_Col->deriv_buf));
        inner_PID_X_Col->last_output = 0;
        inner_PID_X_Col->prev_filtered = 0;
        inner_PID_X_Col->integral = 0;
        inner_PID_X_Col->prev_position = 0;
        inner_PID_X_Col->first_call = true;
    }

    inner_PID_Y_Col = (PIDController *)malloc(sizeof(PIDController));
    if (inner_PID_Y_Col != NULL)
    {
        // 初始化 PID 参数
        inner_PID_Y_Col->kp = 38.0f * 0.84 / 10.345f; // 比例增益
        inner_PID_Y_Col->ki = 0.2f / 10.345f;         // 积分增益
        inner_PID_Y_Col->kd = 12.0f * 1.83 / 10.345f; // 微分增益
        inner_PID_Y_Col->dt = 0.05f;                  // 采样时间
        inner_PID_Y_Col->integral = 0.0f;             // 积分项初始值
        inner_PID_Y_Col->prev_error = 0.0f;           // 上一次误差初始值
        inner_PID_Y_Col->max_output = 800.0f;         // 输出限幅

        inner_PID_Y_Col->last_call_time = HAL_GetTick();
        memset(inner_PID_Y_Col->deriv_buf, 0, sizeof(inner_PID_Y_Col->deriv_buf));
        inner_PID_Y_Col->last_output = 0;
        inner_PID_Y_Col->prev_filtered = 0;
        inner_PID_Y_Col->prev_position = 0;
        inner_PID_Y_Col->first_call = true;
    }

    outer_PID_f_Fast = (PIDController *)malloc(sizeof(PIDController));
    if (outer_PID_f_Fast != NULL)
    {
        outer_PID_f_Fast->kp = 12.0f / 10.345f;         // 原值: 8 
        outer_PID_f_Fast->ki = 1.2f * 5.0f / 10.345f;          // 原值: 0.0f
        outer_PID_f_Fast->kd = 1.2f * 4.0f / 10.345f;   // 原值: 0.2f
        outer_PID_f_Fast->dt = 0.05f;                   // dt 不需要调整
        outer_PID_f_Fast->integral = 0.0f;              // 积分项初始值
        outer_PID_f_Fast->prev_error = 0.0f;            // 上一次误差初始值
        outer_PID_f_Fast->max_output = 25.0f ; // 原值: 80.0f

        outer_PID_f_Fast->last_call_time = HAL_GetTick();
        memset(outer_PID_f_Fast->deriv_buf, 0, sizeof(outer_PID_f_Fast->deriv_buf));
        outer_PID_f_Fast->last_output = 0;
        outer_PID_f_Fast->prev_filtered = 0;
        outer_PID_f_Fast->integral = 0;
        outer_PID_f_Fast->prev_position = 0;
        outer_PID_f_Fast->first_call = true;
    }

    // 为outer_PID_Y_f分配内存并初始化
    outer_PID_Y_f_Fast = (PIDController *)malloc(sizeof(PIDController));
    if (outer_PID_Y_f_Fast != NULL)
    {
        outer_PID_Y_f_Fast->kp = 18.0f / 10.345f;         // 原值: 0.4f    12
        outer_PID_Y_f_Fast->ki = 1.8f * 5.0f / 10.345f;          // 原值: 0.0f
        outer_PID_Y_f_Fast->kd = 1.8f * 4.0f / 10.345f;   // 原值: 0.2f
        outer_PID_Y_f_Fast->dt = 0.05f;                   // dt 不需要调整
        outer_PID_Y_f_Fast->integral = 0.0f;              // 积分项初始值
        outer_PID_Y_f_Fast->prev_error = 0.0f;            // 上一次误差初始值
        outer_PID_Y_f_Fast->max_output = 25.0f ; // 原值: 80.0f

        outer_PID_Y_f_Fast->last_call_time = HAL_GetTick();
        memset(outer_PID_Y_f_Fast->deriv_buf, 0, sizeof(outer_PID_Y_f_Fast->deriv_buf));
        outer_PID_Y_f_Fast->last_output = 0;
        outer_PID_Y_f_Fast->prev_filtered = 0;
        outer_PID_Y_f_Fast->integral = 0;
        outer_PID_Y_f_Fast->prev_position = 0;
        outer_PID_Y_f_Fast->first_call = true;
    }


    outer_PID_Z_f_Fast = (PIDController *)malloc(sizeof(PIDController));
    if (outer_PID_Z_f_Fast != NULL)
    {
        outer_PID_Z_f_Fast->kp = 40.0f / 10.345f;         // 原值: 40 
        outer_PID_Z_f_Fast->ki = 4.0f / 10.345f;          // 原值: 0.0f
        outer_PID_Z_f_Fast->kd = 12.0f / 10.345f;         // 原值: 0.02f
        outer_PID_Z_f_Fast->dt = 0.05f;                   // dt 不需要调整
        outer_PID_Z_f_Fast->integral = 0.0f;              // 积分项初始值
        outer_PID_Z_f_Fast->prev_error = 0.0f;            // 上一次误差初始值
        outer_PID_Z_f_Fast->max_output = 20.0f; // 原值: 50.0f

        outer_PID_Z_f_Fast->last_call_time = HAL_GetTick();
        memset(outer_PID_Z_f_Fast->deriv_buf, 0, sizeof(outer_PID_Z_f_Fast->deriv_buf));
        outer_PID_Z_f_Fast->last_output = 0;
        outer_PID_Z_f_Fast->prev_filtered = 0;
        outer_PID_Z_f_Fast->integral = 0;
        outer_PID_Z_f_Fast->prev_position = 0;
        outer_PID_Z_f_Fast->first_call = true;
    }

    


}
void PIDController_reset(PIDController *self)
{
    if (self != NULL)
    {
        self->integral = 0.0f;
        self->prev_error = 0.0f;

        self->last_call_time = HAL_GetTick(); // 这个其实没有啥问题，每次reset后，都会重置时间，和函数内部的逻辑构成双重保险
        memset(self->deriv_buf, 0, sizeof(self->deriv_buf));
        self->last_output = 0;
        self->prev_filtered = 0;
        self->prev_position = 0;
        self->first_call = true; // 这个first_call是用于判断是否是第一次调用，假设是第一次的话，就在pic_calc_z中把prev_position变成Omega
    }
}

//**************************************************************************************************** */

Car_Status *car = NULL;
// 初始化小车状态
void initializeCar(void)
{
    car = (Car_Status *)malloc(sizeof(Car_Status));
    if (car != NULL)
    {
        // 反馈加上计算得出的车身实时速度，最好是需要与target保持良好的跟随性
        car->current_car_Vx = 0.0f; // 车身的x方向速度，通过反馈的电机速度计算得来
        car->current_car_Vy = 0.0f; //
        car->target_car_Vx = 0.0f;
        car->target_car_Vy = 0.0f;
        // 这个数据是电机返还的，是准确的
        car->delta_car_position_x = 0.0f;  // 上一次微分项（每一次传回来数据，运行的delta数值）
        car->delta_car_position_y = 0.0f;  // 上一次微分项
        car->target_car_position_x = 0.0f; // 上一次微分项
        car->target_car_position_y = 0.0f; // 上一次微分项

        car->target_Vel1 = 0; // 通过pid解算出目标速度之后，分解到各个轮子的目标速度
        car->target_Vel2 = 0;
        car->target_Vel3 = 0;
        car->target_Vel4 = 0;

        car->current_Vel1 = 0; // 这个是电机速度的记录函数，
        car->current_Vel2 = 0;
        car->current_Vel3 = 0;
        car->current_Vel4 = 0;

        car->current_map_Vx = 0.0f; // 这个是根据current_car_Vx计算出来的，是实际执行的X方向速度
        car->current_map_Vy = 0.0f; //
        car->target_map_Vx = 0.0f;
        car->target_map_Vy = 0.0f;

        // 这个变量的偏差是受到即时性影响最大的，因为这个map_position是通过delta积分计算出来的
        // 最后可能需要加上一个（补偿），也就是补偿时间间隔之间的“角度变换”误差？
        // 今天调试，要测试单独走的current_map_position_x和合起来走的current_map_position_x是否存在稳定的偏差
        // 测试结果：（应该会有一个比例系数）
        car->current_map_position_x = 0.0f; // 这个就是通过积分计算出来的了，
        car->current_map_position_y = 0.0f; //
        car->target_map_position_x = 0.0f;
        car->target_map_position_y = 0.0f;
        car->target_angle_speed = 0.0f; // 角速度

        car->car_mode = CAR_STOP;
        car->car_face = Forward;
    }
}
void Reset_Car_Status(void)
{
    car->current_car_Vx = 0.0f; // x方向速度
    car->current_car_Vy = 0.0f; //
    car->target_car_Vx = 0.0f;
    car->target_car_Vy = 0.0f;

    car->delta_car_position_x = 0.0f;  // 上一次微分项
    car->delta_car_position_y = 0.0f;  // 上一次微分项
    car->target_car_position_x = 0.0f; // 上一次微分项
    car->target_car_position_y = 0.0f; // 上一次微分项

    car->target_Vel1 = 0;
    car->target_Vel2 = 0;
    car->target_Vel3 = 0;
    car->target_Vel4 = 0;

    car->current_Vel1 = 0;
    car->current_Vel2 = 0;
    car->current_Vel3 = 0;
    car->current_Vel4 = 0;

    car->current_map_Vx = 0.0f; //
    car->current_map_Vy = 0.0f; //
    car->target_map_Vx = 0.0f;
    car->target_map_Vy = 0.0f;

    car->current_map_position_x = 0.0f; // x方向速度
    car->current_map_position_y = 0.0f; //
    car->target_map_position_x = 0.0f;
    car->target_map_position_y = 0.0f;

    car->target_angle_speed = 0.0f; // 重置目标角速度
    car->car_mode = CAR_STOP;

    motor1_status->first_run = true;
    motor2_status->first_run = true;
    motor3_status->first_run = true;
    motor4_status->first_run = true;
}
void From_Motor_to_Car_Status(void)
{
    float angle = omega * pi / 180; // 角度转弧度
    if (fmod(omega, 360.0f) > 315.0f || fmod(omega, 360.0f) < 45.0f)
    {
        car->car_face = Forward;
    }
    else if (fmod(omega, 360.0f) > 45.0f && fmod(omega, 360.0f) < 135.0f)
    {
        car->car_face = Left;
    }
    else if (fmod(omega, 360.0f) > 135.0f && fmod(omega, 360.0f) < 225.0f)
    {
        car->car_face = Back;
    }
    else if (fmod(omega, 360.0f) > 225.0f && fmod(omega, 360.0f) < 315.0f)
    {
        car->car_face = Right;
    }

    car->current_Vel1 = motor1_status->current_speed;
    car->current_Vel2 = motor2_status->current_speed;
    car->current_Vel3 = motor3_status->current_speed;
    car->current_Vel4 = motor4_status->current_speed;

    car->current_car_Vx = (motor1_status->current_speed + motor2_status->current_speed + motor3_status->current_speed + motor4_status->current_speed) / 4;
    car->current_car_Vy = (-motor1_status->current_speed + motor2_status->current_speed + motor3_status->current_speed - motor4_status->current_speed) / 4;
    // 这里的map那就应该是全局坐标的方向
    car->current_map_Vx = car->current_car_Vx * cosf(angle) - car->current_car_Vy * sinf(angle);
    car->current_map_Vy = car->current_car_Vx * sinf(angle) + car->current_car_Vy * cosf(angle);
    //
    car->delta_car_position_x = (+motor1_status->delta_distance_traveled + motor2_status->delta_distance_traveled + motor3_status->delta_distance_traveled + motor4_status->delta_distance_traveled) / 4;
    car->delta_car_position_y = (-motor1_status->delta_distance_traveled + motor2_status->delta_distance_traveled + motor3_status->delta_distance_traveled - motor4_status->delta_distance_traveled) / 4;

    uint32_t current_time = HAL_GetTick();
    // 如果两次接收的时间超过2s，那么就认为这个时候是没有接收到数据的
    if (current_time - last_reveive_motor_time > 1000)
    {
        car->delta_car_position_x = 0;
        car->delta_car_position_y = 0;
    }

    car->current_map_position_x += car->delta_car_position_x * cosf(angle) - car->delta_car_position_y * sinf(angle);
    car->current_map_position_y += car->delta_car_position_x * sinf(angle) + car->delta_car_position_y * cosf(angle);
}

// 发送信息给电机之后，获取到了每个电机的状态，并且计算出车的状态

void Update_Car_Status(uint32_t delay_ms)
{

    Emm_V5_GetMotor_Status(1);
    HAL_Delay(delay_ms);
    Emm_V5_GetMotor_Status(2);
    HAL_Delay(delay_ms);
    Emm_V5_GetMotor_Status(3);
    HAL_Delay(delay_ms);
    Emm_V5_GetMotor_Status(4);
    HAL_Delay(delay_ms);
    From_Motor_to_Car_Status();
}

void Publish_motor_speed(void)
{
    float angle = omega * pi / 180.0f; // 角度转弧度
    // 根据map_speed计算car_speed
    car->target_car_Vx = car->target_map_Vx * cosf(angle) + car->target_map_Vy * sinf(angle);
    car->target_car_Vy = -car->target_map_Vx * sinf(angle) + car->target_map_Vy * cosf(angle);
    // 根据car_speed计算motor_speed
    float Car_H = 1.670f; // 前后轮心距
    float Car_W = 2.380f; // 左右轮心距
    float Kx = 1;
    float Ky = 1;
    float Kz = 1.74 * 1; // 1.74将target和current单位统一

    car->target_Vel1 = Kx * car->target_car_Vx - Ky * car->target_car_Vy + Kz * (-car->target_angle_speed * (Car_H / 2 + Car_W / 2));
    car->target_Vel2 = Kx * car->target_car_Vx + Ky * car->target_car_Vy + Kz * (+car->target_angle_speed * (Car_H / 2 + Car_W / 2));
    car->target_Vel3 = Kx * car->target_car_Vx + Ky * car->target_car_Vy + Kz * (-car->target_angle_speed * (Car_H / 2 + Car_W / 2));
    car->target_Vel4 = Kx * car->target_car_Vx - Ky * car->target_car_Vy + Kz * (+car->target_angle_speed * (Car_H / 2 + Car_W / 2));

    // 线性平滑
    car->target_Vel1 = car->target_Vel1 * 0.85 + car->current_Vel1 * 0.15;
    car->target_Vel2 = car->target_Vel2 * 0.85 + car->current_Vel2 * 0.15;
    car->target_Vel3 = car->target_Vel3 * 0.85 + car->current_Vel3 * 0.15;
    car->target_Vel4 = car->target_Vel4 * 0.85 + car->current_Vel4 * 0.15;

    uint8_t acc = 150;
    bool snF = true;
    uint32_t delay_ms = 5;
    Emm_V5_Send_Four_Vel_Control(car->target_Vel1, car->target_Vel2, car->target_Vel3, car->target_Vel4, acc, snF, delay_ms);
}

// 停车
void Car_Stop(bool snF) // 如果给零，就急停，给一速度为零
{
    car->car_mode = CAR_STOP;
    ////这里进行了修改，把急停的延时加在了这个里面
    HAL_Delay(100);

    if (snF == false)
    {
        Emm_V5_Stop_Now(0, false);
    }
    else if (snF == true)
    {
        Emm_V5_Vel_Control(0, 0, 0, 150, 0);
    }
    // HAL_Delay(100);
}

//************************************************************************************************************ */
// 位置模式
void Move_TransfromX(float X)
{

    float Target1, Target2, Target3, Target4 = 0.0f;
    //     uint8_t dir2,dir4=0;//zhong wen bu neng yong le,CW zheng xiang.
    //     uint8_t dir1,dir3=1;//ccw,zheng xiang
    Target3 = X;
    Target1 = X;
    Target2 = X;
    Target4 = X;
    /////这里原来的速度是1000，这里修改了20
    Emm_V5_Pos_Control_Self(1, 1000, 20, (int32_t)Target1, 0, 1); // 多级同步标志位设置为1，
    HAL_Delay(10);
    Emm_V5_Pos_Control_Self(2, 1000, 20, (int32_t)Target2, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control_Self(3, 1000, 20, (int32_t)Target3, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control_Self(4, 1000, 20, (int32_t)Target4, 0, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0); // tongbu
    HAL_Delay(10);
}

void Move_TransfromY(float Y)
{
    float Target1, Target2, Target3, Target4 = 0.0f;

    Target3 = +Y; // 实验结果，不知道为啥与理论不一样，现在是规定的向左为正
    Target1 = -Y;
    Target2 = +Y;
    Target4 = -Y;

    Emm_V5_Pos_Control_Self(1, 1000, 20, (int32_t)Target1, 0, 1); // 多级同步标志位设置为1，
    HAL_Delay(10);
    Emm_V5_Pos_Control_Self(2, 1000, 20, (int32_t)Target2, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control_Self(3, 1000, 20, (int32_t)Target3, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control_Self(4, 1000, 20, (int32_t)Target4, 0, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0); // tongbu
    HAL_Delay(10);
}

void Move_TransfromXY(float XY) // 这个是向左前方向运动
{
    float Target1, Target2, Target3, Target4 = 0.0f;
    //     uint8_t dir2,dir4=0;//zhong wen bu neng yong le,CW zheng xiang.
    //     uint8_t dir1,dir3=1;//ccw,zheng xiang
    Target3 = -2 * XY;
    Target1 = 0;
    Target2 = -2 * XY;
    Target4 = 0;

    Emm_V5_Pos_Control_Self(1, 1000, 10, (int32_t)Target1, 0, 1); // 多级同步标志位设置为1，
    HAL_Delay(10);
    Emm_V5_Pos_Control_Self(2, 1000, 10, (int32_t)Target2, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control_Self(3, 1000, 10, (int32_t)Target3, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control_Self(4, 1000, 10, (int32_t)Target4, 0, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0); // tongbu
    HAL_Delay(10);
}

void Move_TransfromZ(float Z) /// 感觉写完这些也就不需要别的了，就每一个步骤向前或者特定方向
{
    //      uint8_t dir2,dir4=0;//zhong wen bu neng yong le,CW zheng xiang.
    //     uint8_t dir1,dir3=1;//ccw,zheng xiang
    float Target1, Target2, Target3, Target4 = 0.0f;
    int32_t Car_H = 167, Car_W = 238; // 前后距离/(width)实际上这个应该是一个常数，但是为了方便，还是先写成变量的形式（问机械）
    // 关于这个是角度还是弧度，还是要看具体参数，，所以只需要最后加上一个系数即可，不要纠结
    Target3 = (float)(-Z * (Car_H / 2 + Car_W / 2));
    Target1 = (float)(-Z * (Car_H / 2 + Car_W / 2));
    Target2 = (float)(+Z * (Car_H / 2 + Car_W / 2));
    Target4 = (float)(+Z * (Car_H / 2 + Car_W / 2));
    float ki = 0.017255 * 180.0f / 179.0f * 180.0f / 180.2f; //(一个实际测出来的数字)(17/18.5/180)(3.37 * (17 / (18.5 * 180)))
    // 在contrl_self里面，要把单位调整成mm
    Target3 = Target3 * ki;
    Target1 = Target1 * ki;
    Target2 = Target2 * ki;
    Target4 = Target4 * ki;

    Emm_V5_Pos_Control_Self(1, 800, 150, (int32_t)Target1, 0, 1); // 多级同步标志位设置为1，
    HAL_Delay(10);
    Emm_V5_Pos_Control_Self(2, 800, 150, (int32_t)Target2, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control_Self(3, 800, 150, (int32_t)Target3, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control_Self(4, 800, 150, (int32_t)Target4, 0, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0); // tongbu
    HAL_Delay(10);
}

// 这里设置一个Omega是因为要根据全局变量这样一个omega去进行校准，

//************************************************************************************************************ */
// PID控制器

float PID_calc(float error_position, PIDController *PID)
{

    if (PID->first_call == true)
    {
        PID->first_call = false;

        PID->last_call_time = HAL_GetTick();
        return 0;
    }

    uint32_t current_time = HAL_GetTick();
    float actual_dt = (current_time - PID->last_call_time) / 1000.0f;
    actual_dt = fmaxf(fminf(actual_dt, 0.1f), 0.005f);
    // 这个是用于限制dt的，因为dt不能太大，也不能太小，所以需要限制
    // 这个是哪里来的莫名其妙的代码，我们的时间很长，不要被莫名其妙的限制幅度了

    PID->last_call_time = current_time;
    PID->dt = actual_dt;

    float error = error_position;
    float abs_error = fabsf(error);

    //===== 四阶巴特沃斯滤波（调整为8Hz截止频率） =====//

    float raw_deriv = (error - PID->prev_error) / actual_dt;
    float filtered_deriv = 0.032f * raw_deriv + 0.198f * PID->deriv_buf[0] + 0.468f * PID->deriv_buf[1] + 0.198f * PID->deriv_buf[2] + 0.032f * PID->deriv_buf[3];
    memmove(&PID->deriv_buf[1], PID->deriv_buf, 3 * sizeof(float));
    PID->deriv_buf[0] = filtered_deriv;

    //===== 增强型动态刚柔切换 =====//
    float kp_ratio, kd_ratio;
    if (abs_error > 100.0f)
    { // 刚性模式
        kp_ratio = 0.7f;
        kd_ratio = 1.5f;
    }
    else
    { // 柔性模式
        // 修改增益曲线斜率（提升末端驱动力）
        kp_ratio = 0.5f + 0.5f * (abs_error / 100.0f); // 原0.4→0.5
        kd_ratio = 1.0f - 0.5f * (abs_error / 100.0f); // 原0.8→1.0
    }

//===== 新增末端动力补偿 =====//
#define DEADZONE_COMP 3.0f
#define COMP_START 30.0f
    float comp_term = 0.0f; // 在此处声明并初始化

    if (abs_error < COMP_START)
    {
        // 动态增益重组
        kp_ratio *= 1.0f + (COMP_START - abs_error) / COMP_START;
        kd_ratio *= 1.0f - (COMP_START - abs_error) / COMP_START;

        // 计算补偿量（需在此处赋值）
        comp_term = (error > 0) ? DEADZONE_COMP : -DEADZONE_COMP;
        comp_term *= (COMP_START - abs_error) / COMP_START;
    }

    /// 现在进行函数的修改，对于这个kp和kd还有ki用二次的那啥去那啥，然后末端阻力可以缩短的很小，（二次p）

    //===== 输出合成（使用已声明的comp_term） =====//
    float output = (PID->kp * kp_ratio) * error + (PID->kd * kd_ratio) * filtered_deriv + comp_term; // 此处可正常访问

    //===== 条件积分模块 =====//
    if (fabsf(error) < 7.5f)                     // 直接按照积分模块去那啥咯
    {                                            // 仅在误差<目标值的5%时激活
        PID->integral += error * PID->dt * 0.3f; // Ki=0.3

        // 积分限幅（限制在最大输出的10%）
        float max_integral = PID->max_output * 0.1f / 0.3f; // 反算限幅
        PID->integral = fmaxf(fminf(PID->integral, max_integral), -max_integral);
    }
    else
    {
        PID->integral *= 0.5f; // 快速释放历史积分
    }

    // 合成输出（新增积分项）
    output += PID->ki * PID->integral;

    //===== 增强型安全限幅 =====//

    output = fmaxf(fminf(output, PID->max_output), -PID->max_output);
    output = 0.6f * output + 0.4f * PID->last_output; // 调整惯性系数
    PID->last_output = output;

    //===== 末端微调防抖 =====//
    if (abs_error < 5.0f)
    {
        output *= 0.3f; // 极近端输出衰减
    }

    PID->prev_error = error;
    return output;
}

// float PID_calc(float error_position, PIDController *PID)
// {
//     static uint32_t last_call_time = 0;
//     uint32_t current_time = HAL_GetTick();
//     float actual_dt = (current_time - last_call_time) / 1000.0f;
//     actual_dt = fmaxf(fminf(actual_dt, 0.1f), 0.005f);
//     last_call_time = current_time;
//     PID->dt = actual_dt;

//     float error = error_position;
//     float abs_error = fabsf(error);

//     //===== 四阶巴特沃斯滤波（调整为8Hz截止频率） =====//
//     static float deriv_buf[4] = {0};
//     float raw_deriv = (error - PID->prev_error) / actual_dt;
//     float filtered_deriv = 0.032f * raw_deriv + 0.198f * deriv_buf[0] + 0.468f * deriv_buf[1] + 0.198f * deriv_buf[2] + 0.032f * deriv_buf[3];
//     memmove(&deriv_buf[1], deriv_buf, 3 * sizeof(float));
//     deriv_buf[0] = filtered_deriv;

//     float p, a, b;
//     a = (float)9/100000;
//     b = 0.5;
//     p = a * error * error + b;
//     PID->kp = p;
//     //===== 输出合成（使用已声明的comp_term） =====//
//     float output = PID->kp * error;

//     //===== 条件积分模块 =====//
//     if (fabsf(error) < 7.5f)                     // 直接按照积分模块去那啥咯
//     {                                            // 仅在误差<目标值的5%时激活
//         PID->integral += error * PID->dt * 0.3f; // Ki=0.3

//         // 积分限幅（限制在最大输出的10%）
//         float max_integral = PID->max_output * 0.1f / 0.3f; // 反算限幅
//         PID->integral = fmaxf(fminf(PID->integral, max_integral), -max_integral);
//     }
//     else
//     {
//         PID->integral *= 0.5f; // 快速释放历史积分
//     }

//     // 合成输出（新增积分项）
//     output += PID->ki * PID->integral;

//     //===== 增强型安全限幅 =====//
//     static float last_output = 0.0f;
//     output = fmaxf(fminf(output, PID->max_output), -PID->max_output);
//     output = 0.6f * output + 0.4f * last_output; // 调整惯性系数
//     last_output = output;

//     //===== 末端微调防抖 =====//
//     if (abs_error < 5.0f)
//     {
//         output *= 0.3f; // 极近端输出衰减
//     }

//     PID->prev_error = error;
//     return output;
// }

float PID_calc_Y(float error_position, PIDController *PID)
{

    /////////////
    if (PID->first_call == true)
    {
        PID->first_call = false;

        PID->last_call_time = HAL_GetTick();
        return 0;
    }

    uint32_t current_time = HAL_GetTick();
    float actual_dt = (current_time - PID->last_call_time) / 1000.0f;
    actual_dt = fmaxf(fminf(actual_dt, 0.1f), 0.005f);
    PID->last_call_time = current_time;
    PID->dt = actual_dt;

    float error = error_position;
    float abs_error = fabsf(error);

    //===== 四阶巴特沃斯滤波（调整为8Hz截止频率） =====//

    float raw_deriv = (error - PID->prev_error) / actual_dt;
    float filtered_deriv = 0.032f * raw_deriv + 0.198f * PID->deriv_buf[0] + 0.468f * PID->deriv_buf[1] + 0.198f * PID->deriv_buf[2] + 0.032f * PID->deriv_buf[3];
    memmove(&PID->deriv_buf[1], PID->deriv_buf, 3 * sizeof(float));
    PID->deriv_buf[0] = filtered_deriv;

    //===== 增强型动态刚柔切换 =====//
    float kp_ratio, kd_ratio;
    if (abs_error > 100.0f)
    { // 刚性模式
        kp_ratio = 0.7f;
        kd_ratio = 1.5f;
    }
    else
    { // 柔性模式
        // 修改增益曲线斜率（提升末端驱动力）
        kp_ratio = 0.5f + 0.5f * (abs_error / 100.0f); // 原0.4→0.5
        kd_ratio = 1.0f - 0.5f * (abs_error / 100.0f); // 原0.8→1.0
    }

//===== 新增末端动力补偿 =====//
#define DEADZONE_COMP 3.0f
#define COMP_START 30.0f
    float comp_term = 0.0f; // 在此处声明并初始化

    if (abs_error < COMP_START)
    {
        // 动态增益重组
        kp_ratio *= 1.0f + (COMP_START - abs_error) / COMP_START;
        kd_ratio *= 1.0f - (COMP_START - abs_error) / COMP_START;

        // 计算补偿量（需在此处赋值）
        comp_term = (error > 0) ? DEADZONE_COMP : -DEADZONE_COMP;
        comp_term *= (COMP_START - abs_error) / COMP_START;
    }

    //===== 输出合成（使用已声明的comp_term） =====//
    float output = (PID->kp * kp_ratio) * error + (PID->kd * kd_ratio) * filtered_deriv + comp_term; // 此处可正常访问

    //===== 条件积分模块 =====//
    if (fabsf(error) < 7.5f)                     // 直接按照积分模块去那啥咯
    {                                            // 仅在误差<目标值的5%时激活
        PID->integral += error * PID->dt * 0.3f; // Ki=0.3

        // 积分限幅（限制在最大输出的10%）
        float max_integral = PID->max_output * 0.1f / 0.3f; // 反算限幅
        PID->integral = fmaxf(fminf(PID->integral, max_integral), -max_integral);
    }
    else
    {
        PID->integral *= 0.5f; // 快速释放历史积分
    }

    // 合成输出（新增积分项）
    output += PID->ki * PID->integral;

    //===== 增强型安全限幅 =====//

    output = fmaxf(fminf(output, PID->max_output), -PID->max_output);
    output = 0.6f * output + 0.4f * PID->last_output; // 调整惯性系数
    PID->last_output = output;

    //===== 末端微调防抖 =====//
    if (abs_error < 5.0f)
    {
        output *= 0.3f; // 极近端输出衰减
    }

    PID->prev_error = error;
    return output;
}
// 这里是角度环参数的保留，我先保存在这里，也不知道要干啥烦死了。

float PID_calc_z(float target_position, float init_omega, PIDController *PID)
{

    /////////////
    if (PID->first_call == true)
    {
        PID->prev_position = init_omega;
        PID->first_call = false;

        PID->last_call_time = HAL_GetTick();
        return 0;

    } // 这里需要进行一个巨大的修改，就是每一次使用

    // 初始化逻辑

    float current_time = HAL_GetTick();
    float dt = (current_time - PID->last_call_time) / 1000.0f;
    dt = fmaxf(fminf(dt, 0.1f), 0.005f);
    PID->last_call_time = current_time;
    float current_position = omega;
    float error = target_position - current_position;
    float output = 0;

    /******************** 图像特征优化区 ▼▼▼ ​********************/
    // ▲ V段优化：抑制初始速度衰减
    float velocity = (current_position - PID->prev_position) / dt;
    float decay_factor = 1.0f - 0.35f * powf(fabs(velocity) / 180.0f, 0.55f);
    float filtered_deriv = 0.032f * ((error - PID->prev_error) / dt) + 0.198f * PID->deriv_buf[0] + 0.468f * PID->deriv_buf[1] + 0.198f * PID->deriv_buf[2] + 0.032f * PID->deriv_buf[3];
    filtered_deriv = 0.65f * decay_factor * (0.68f * filtered_deriv + 0.32f * PID->prev_filtered);

    // ▲ O-F段优化：增强中期动力
    float dynamic_kp = PID->kp;
    float dynamic_kd = PID->kd;

    float active_ki = PID->ki;

    /******************** 核心优化区 ▼▼▼ ​********************/
    float error_ratio = fabs(error) / (fabs(target_position - init_omega) + 0.01f);
    // 现在我的想法比较简单，就是直接在这里新增一个90度旋转的控制逻辑，就直接在这个分类里面进行

    /******************** 新增稳态维持模块 ▼▼▼ ​********************/
    // 当误差绝对值<3度且目标为0时激活稳态模式
    if (fabs(error) < 10.0f && fabs(target_position - init_omega) < 5.0f)
    {
        // 独立参数配置（与原有error_ratio逻辑解耦）
        float steady_kp = 14.4 / 10.345 * 3 * 0.6; // 降比例增益防振荡
        float steady_ki = steady_kp * 0.1;         // 强化积分作用
        float steady_kd = 4.175 * 0.12 * 2.0;      // 降微分防噪声

        // 速度预测前馈（抑制扰动）
        float velocity = (current_position - PID->prev_position) / dt;
        float feedforward = -velocity * 0.8f; // 反向补偿速度扰动

        // 带死区的积分控制（误差<0.5度时生效）
        if (fabs(error) > 0.5f)
        {
            PID->integral += steady_ki * error * dt;
        }

        // 积分限幅（max_output的15%）
        PID->integral = fmaxf(fminf(PID->integral, PID->max_output * 0.15f),
                              -PID->max_output * 0.15f);

        // 合成输出
        output = steady_kp * error + PID->integral + steady_kd * filtered_deriv + feedforward;
    }
    else if (fabs(target_position - init_omega) > 85.0f && fabs(target_position - init_omega) < 95.0f)
    {
        // 独立参数配置（与原有error_ratio逻辑解耦）
        float steady_kp = 20.0f * 0.717 / 10.345f; // 降比例增益防振荡
        float steady_ki = 0.033 / 10.345f;         // 强化积分作用
        float steady_kd = 14.0f * 2.25 / 10.345f;  // 降微分防噪声

        if (fabs(error) > 0.5f)
        {
            PID->integral += steady_ki * error * dt;
        }

        // 积分限幅（max_output的15%）
        PID->integral = fmaxf(fminf(PID->integral, PID->max_output * 0.15f),
                              -PID->max_output * 0.15f);

        // 合成输出
        output = steady_kp * error + PID->integral + steady_kd * filtered_deriv;
    }
    else
    {
        // 动态增益控制（根据图像特征优化）
        if (error_ratio > 0.4f)
        {                               // 超调抑制区（对应图像陡峭上升段）
            dynamic_kp = PID->kp * 0.6; // 主动降低比例增益40%
            dynamic_kd = PID->kd * 2.2; // 强化微分阻尼
            active_ki = PID->ki * 0.3;  // 抑制积分累积
        }
        else if (error_ratio > 0.15f)
        {                               // 动态平衡区
            dynamic_kp = PID->kp * 1.1; // 提升比例增益10%
            dynamic_kd = PID->kd * 1.4; // 适度阻尼
            active_ki = PID->ki * 0.9;  // 正常积分
        }
        else
        {                                // 精密控制区
            dynamic_kp = PID->kp * 0.95; // 微降比例防振荡
            dynamic_kd = PID->kd * 0.7;  // 降低微分
            active_ki = PID->ki * 1.6;   // 强化积分消除静差
        }

        // 计算输出
        output = dynamic_kp * error + PID->integral + dynamic_kd * filtered_deriv;

        // 速度补偿（抑制中期减速）
        if (error_ratio > 0.3f && velocity < 0.5f * (target_position - init_omega))
        {
            output += 0.3f * ((target_position - init_omega) - fabs(current_position));
        }

        // 梯度约束（优化）
        output = fmaxf(fminf(output, PID->last_output + PID->max_output * 0.1f),
                       PID->last_output - PID->max_output * 0.1f);
    }

    // 状态更新
    memmove(&PID->deriv_buf[1], PID->deriv_buf, 3 * sizeof(float));
    PID->deriv_buf[0] = filtered_deriv;
    PID->prev_filtered = filtered_deriv;
    PID->prev_error = error;
    PID->prev_position = current_position;
    PID->last_output = output;

    return fmaxf(fminf(output, PID->max_output), -PID->max_output);
}

float PID_calc_micro(float error_position, PIDController *PID)
{

    if (PID->first_call == true)
    {
        PID->first_call = false;

        PID->last_call_time = HAL_GetTick();
        return 0;
    }

    uint32_t current_time = HAL_GetTick();
    float actual_dt = (current_time - PID->last_call_time) / 1000.0f;
    actual_dt = fmaxf(fminf(actual_dt, 0.1f), 0.005f);
    // 这个是用于限制dt的，因为dt不能太大，也不能太小，所以需要限制
    // 这个是哪里来的莫名其妙的代码，我们的时间很长，不要被莫名其妙的限制幅度了

    PID->last_call_time = current_time;
    PID->dt = actual_dt;

    float error = error_position;
    float abs_error = fabsf(error);

    //===== 四阶巴特沃斯滤波（调整为8Hz截止频率） =====//

    float raw_deriv = (error - PID->prev_error) / actual_dt;
    float filtered_deriv = 0.032f * raw_deriv + 0.198f * PID->deriv_buf[0] + 0.468f * PID->deriv_buf[1] + 0.198f * PID->deriv_buf[2] + 0.032f * PID->deriv_buf[3];
    memmove(&PID->deriv_buf[1], PID->deriv_buf, 3 * sizeof(float));
    PID->deriv_buf[0] = filtered_deriv;

    //===== 输出合成（使用已声明的comp_term） =====//
    float output = (PID->kp) * error + (PID->kd) * filtered_deriv;

    ////之后积分项目进行更新的时候，需要把后面的积分项目清除

    //===== 条件积分模块 =====//
    if (fabs(error) < 2.0f)                         // 直接按照积分模块去那啥咯
    {                                               // 仅在误差<目标值的5%时激活
        PID->integral += error * PID->dt * PID->ki; // Ki=0.3

        // 积分限幅（限制在最大输出的10%）

        PID->integral = fmaxf(fminf(PID->integral, PID->max_output), -PID->max_output);
    }
    else
    {
        PID->integral *= 0.5f; // 快速释放历史积分
    }
    // 对积分项目进行了修改，integral现在是已经加上了积分增益的

    PID->integral += error * PID->dt * PID->ki;

    if (error * PID->prev_error < 0.0f)
    {
        PID->integral = 0.0f; // 快速释放历史积分
    }

    // 合成输出（新增积分项）
    output += PID->integral;

    //===== 增强型安全限幅 =====//

    output = fmaxf(fminf(output, PID->max_output), -PID->max_output);
    // output = 0.6f * output + 0.4f * PID->last_output; // 调整惯性系数

    PID->last_output = output;

    //===== 末端微调防抖 =====//
    // if (abs_error < 5.0f)
    // {
    //     output *= 0.3f; // 极近端输出衰减
    // }

    PID->prev_error = error;
    return output;
}

float PID_calc_Y_micro(float error_position, PIDController *PID)
{

    if (PID->first_call == true)
    {
        PID->first_call = false;

        PID->last_call_time = HAL_GetTick();
        return 0;
    }

    uint32_t current_time = HAL_GetTick();
    float actual_dt = (current_time - PID->last_call_time) / 1000.0f;
    actual_dt = fmaxf(fminf(actual_dt, 0.1f), 0.005f);
    // 这个是用于限制dt的，因为dt不能太大，也不能太小，所以需要限制
    // 这个是哪里来的莫名其妙的代码，我们的时间很长，不要被莫名其妙的限制幅度了

    PID->last_call_time = current_time;
    PID->dt = actual_dt;

    float error = error_position;
    float abs_error = fabsf(error);

    //===== 四阶巴特沃斯滤波（调整为8Hz截止频率） =====//

    float raw_deriv = (error - PID->prev_error) / actual_dt;
    float filtered_deriv = 0.032f * raw_deriv + 0.198f * PID->deriv_buf[0] + 0.468f * PID->deriv_buf[1] + 0.198f * PID->deriv_buf[2] + 0.032f * PID->deriv_buf[3];
    memmove(&PID->deriv_buf[1], PID->deriv_buf, 3 * sizeof(float));
    PID->deriv_buf[0] = filtered_deriv;

    //===== 输出合成（使用已声明的comp_term） =====//
    float output = (PID->kp) * error + (PID->kd) * filtered_deriv; // 此处可正常访问

    // 之后积分项目进行更新的时候，需要别的那啥

    //===== 条件积分模块 =====//
    if (fabsf(error) < 2.5f) // 直接按照积分模块去那啥咯
    {                        // 仅在误差<目标值的5%时激活
        PID->integral += error * PID->dt * PID->ki;

        // 积分限幅（限制在最大输出的10%）

        PID->integral = fmaxf(fminf(PID->integral, PID->max_output), -PID->max_output);
    }
    else
    {
        PID->integral *= 0.5f; // 快速释放历史积分
    }

    PID->integral += error * PID->dt * PID->ki;

    PID->integral = fmaxf(fminf(PID->integral, PID->max_output), -PID->max_output);

    if (error * PID->prev_error < 0.0f)
    {
        PID->integral = 0.0f; // 快速释放历史积分
    }

    // 合成输出（新增积分项）
    output += PID->integral;

    //===== 增强型安全限幅 =====//

    output = fmaxf(fminf(output, PID->max_output), -PID->max_output);
    // output = 0.6f * output + 0.4f * PID->last_output; // 调整惯性系数

    PID->last_output = output;

    PID->prev_error = error;
    return output;
}

float PID_calc_z_micro(float target_position, float init_omega, PIDController *PID)
{

    /////////////
    if (PID->first_call == true)
    {
        PID->prev_position = init_omega;
        PID->first_call = false;

        PID->last_call_time = HAL_GetTick();
        return 0;

    } // 这里需要进行一个巨大的修改，就是每一次使用

    // 初始化逻辑

    float current_time = HAL_GetTick();
    float dt = (current_time - PID->last_call_time) / 1000.0f;
    dt = fmaxf(fminf(dt, 0.1f), 0.005f);
    PID->last_call_time = current_time;
    float current_position = omega;
    float error = target_position - current_position;
    float output = 0;

    // 自己加上的

    /******************** 图像特征优化区 ▼▼▼ ​********************/
    // ▲ V段优化：抑制初始速度衰减
    float raw_deriv = (error - PID->prev_error) / dt;
    float velocity = (current_position - PID->prev_position) / dt;
    float decay_factor = 1.0f - 0.35f * powf(fabs(velocity) / 180.0f, 0.55f);
    float filtered_deriv = 0.032f * ((error - PID->prev_error) / dt) + 0.198f * PID->deriv_buf[0] + 0.468f * PID->deriv_buf[1] + 0.198f * PID->deriv_buf[2] + 0.032f * PID->deriv_buf[3];
    filtered_deriv = 0.65f * decay_factor * (0.68f * filtered_deriv + 0.32f * PID->prev_filtered);

    if (fabsf(error) < 2.5f) // 直接按照积分模块去那啥咯
    {                        // 仅在误差<目标值的5%时激活
        PID->integral += error * PID->dt * PID->ki;

        // 积分限幅（限制在最大输出的10%）

        PID->integral = fmaxf(fminf(PID->integral, PID->max_output), -PID->max_output);
    }
    else
    {
        PID->integral *= 0.5f; // 快速释放历史积分
    }

    //  ==== 积分项更新 ====
    PID->integral += PID->ki * error * dt;
    PID->integral = fmaxf(fminf(PID->integral, PID->max_output), -PID->max_output);

    if (error * PID->prev_error < 0.0f)
    {
        PID->integral = 0.0f; // 快速释放历史积分
    }

    output = PID->kp * error + PID->integral + PID->kd * filtered_deriv;

    // 状态更新
    memmove(&PID->deriv_buf[1], PID->deriv_buf, 3 * sizeof(float));
    PID->deriv_buf[0] = filtered_deriv;
    PID->prev_filtered = filtered_deriv;
    PID->prev_error = error;
    PID->prev_position = current_position;
    PID->last_output = output;

    return fmaxf(fminf(output, PID->max_output), -PID->max_output);
}


float PID_calc_micro_Fast(float error_position, PIDController *PID)
{

    if (PID->first_call == true)
    {
        PID->first_call = false;

        PID->last_call_time = HAL_GetTick();
        return 0;
    }

    uint32_t current_time = HAL_GetTick();
    float actual_dt = (current_time - PID->last_call_time) / 1000.0f;
    actual_dt = fmaxf(fminf(actual_dt, 0.1f), 0.005f);
    // 这个是用于限制dt的，因为dt不能太大，也不能太小，所以需要限制
    // 这个是哪里来的莫名其妙的代码，我们的时间很长，不要被莫名其妙的限制幅度了

    PID->last_call_time = current_time;
    PID->dt = actual_dt;

    float error = error_position;
    float abs_error = fabsf(error);

    //===== 四阶巴特沃斯滤波（调整为8Hz截止频率） =====//

    float raw_deriv = (error - PID->prev_error) / actual_dt;
    float filtered_deriv = 0.032f * raw_deriv + 0.198f * PID->deriv_buf[0] + 0.468f * PID->deriv_buf[1] + 0.198f * PID->deriv_buf[2] + 0.032f * PID->deriv_buf[3];
    memmove(&PID->deriv_buf[1], PID->deriv_buf, 3 * sizeof(float));
    PID->deriv_buf[0] = filtered_deriv;

    //===== 输出合成（使用已声明的comp_term） =====//
    float output = (PID->kp) * error + (PID->kd) * filtered_deriv;

    ////之后积分项目进行更新的时候，需要把后面的积分项目清除

    //===== 条件积分模块 =====//
    if (fabs(error) < 2.0f)                         // 直接按照积分模块去那啥咯
    {                                               // 仅在误差<目标值的5%时激活
        PID->integral += error * PID->dt * PID->ki; // Ki=0.3

        // 积分限幅（限制在最大输出的10%）

        PID->integral = fmaxf(fminf(PID->integral, PID->max_output), -PID->max_output);
    }
    else
    {
        PID->integral *= 0.5f; // 快速释放历史积分
    }
    // 对积分项目进行了修改，integral现在是已经加上了积分增益的

    PID->integral += error * PID->dt * PID->ki;

    if (error * PID->prev_error < 0.0f)
    {
        PID->integral = 0.0f; // 快速释放历史积分
    }

    // 合成输出（新增积分项）
    output += PID->integral;

    //===== 增强型安全限幅 =====//

    output = fmaxf(fminf(output, PID->max_output), -PID->max_output);
    // output = 0.6f * output + 0.4f * PID->last_output; // 调整惯性系数

    PID->last_output = output;

    //===== 末端微调防抖 =====//
    // if (abs_error < 5.0f)
    // {
    //     output *= 0.3f; // 极近端输出衰减
    // }

    PID->prev_error = error;
    return output;
}



float PID_calc_Y_micro_Fast(float error_position, PIDController *PID)
{

    if (PID->first_call == true)
    {
        PID->first_call = false;

        PID->last_call_time = HAL_GetTick();
        return 0;
    }

    uint32_t current_time = HAL_GetTick();
    float actual_dt = (current_time - PID->last_call_time) / 1000.0f;
    actual_dt = fmaxf(fminf(actual_dt, 0.1f), 0.005f);
    // 这个是用于限制dt的，因为dt不能太大，也不能太小，所以需要限制
    // 这个是哪里来的莫名其妙的代码，我们的时间很长，不要被莫名其妙的限制幅度了

    PID->last_call_time = current_time;
    PID->dt = actual_dt;

    float error = error_position;
    float abs_error = fabsf(error);

    //===== 四阶巴特沃斯滤波（调整为8Hz截止频率） =====//

    float raw_deriv = (error - PID->prev_error) / actual_dt;
    float filtered_deriv = 0.032f * raw_deriv + 0.198f * PID->deriv_buf[0] + 0.468f * PID->deriv_buf[1] + 0.198f * PID->deriv_buf[2] + 0.032f * PID->deriv_buf[3];
    memmove(&PID->deriv_buf[1], PID->deriv_buf, 3 * sizeof(float));
    PID->deriv_buf[0] = filtered_deriv;

    //===== 输出合成（使用已声明的comp_term） =====//
    float output = (PID->kp) * error + (PID->kd) * filtered_deriv; // 此处可正常访问

    // 之后积分项目进行更新的时候，需要别的那啥

    //===== 条件积分模块 =====//
    if (fabsf(error) < 2.5f) // 直接按照积分模块去那啥咯
    {                        // 仅在误差<目标值的5%时激活
        PID->integral += error * PID->dt * PID->ki;

        // 积分限幅（限制在最大输出的10%）

        PID->integral = fmaxf(fminf(PID->integral, PID->max_output), -PID->max_output);
    }
    else
    {
        PID->integral *= 0.5f; // 快速释放历史积分
    }

    PID->integral += error * PID->dt * PID->ki;

    PID->integral = fmaxf(fminf(PID->integral, PID->max_output), -PID->max_output);

    if (error * PID->prev_error < 0.0f)
    {
        PID->integral = 0.0f; // 快速释放历史积分
    }

    // 合成输出（新增积分项）
    output += PID->integral;

    //===== 增强型安全限幅 =====//

    output = fmaxf(fminf(output, PID->max_output), -PID->max_output);
    // output = 0.6f * output + 0.4f * PID->last_output; // 调整惯性系数

    PID->last_output = output;

    PID->prev_error = error;
    return output;
}


float PID_calc_z_micro_Fast(float target_position, float init_omega, PIDController *PID)
{

    /////////////
    if (PID->first_call == true)
    {
        PID->prev_position = init_omega;
        PID->first_call = false;

        PID->last_call_time = HAL_GetTick();
        return 0;

    } // 这里需要进行一个巨大的修改，就是每一次使用

    // 初始化逻辑

    float current_time = HAL_GetTick();
    float dt = (current_time - PID->last_call_time) / 1000.0f;
    dt = fmaxf(fminf(dt, 0.1f), 0.005f);
    PID->last_call_time = current_time;
    float current_position = omega;
    float error = target_position - current_position;
    float output = 0;

    // 自己加上的

    /******************** 图像特征优化区 ▼▼▼ ​********************/
    // ▲ V段优化：抑制初始速度衰减
    float raw_deriv = (error - PID->prev_error) / dt;
    float velocity = (current_position - PID->prev_position) / dt;
    float decay_factor = 1.0f - 0.35f * powf(fabs(velocity) / 180.0f, 0.55f);
    float filtered_deriv = 0.032f * ((error - PID->prev_error) / dt) + 0.198f * PID->deriv_buf[0] + 0.468f * PID->deriv_buf[1] + 0.198f * PID->deriv_buf[2] + 0.032f * PID->deriv_buf[3];
    filtered_deriv = 0.65f * decay_factor * (0.68f * filtered_deriv + 0.32f * PID->prev_filtered);

    if (fabsf(error) < 2.5f) // 直接按照积分模块去那啥咯
    {                        // 仅在误差<目标值的5%时激活
        PID->integral += error * PID->dt * PID->ki;

        // 积分限幅（限制在最大输出的10%）

        PID->integral = fmaxf(fminf(PID->integral, PID->max_output), -PID->max_output);
    }
    else
    {
        PID->integral *= 0.5f; // 快速释放历史积分
    }

    //  ==== 积分项更新 ====
    PID->integral += PID->ki * error * dt;
    PID->integral = fmaxf(fminf(PID->integral, PID->max_output), -PID->max_output);

    if (error * PID->prev_error < 0.0f)
    {
        PID->integral = 0.0f; // 快速释放历史积分
    }

    output = PID->kp * error + PID->integral + PID->kd * filtered_deriv;

    // 状态更新
    memmove(&PID->deriv_buf[1], PID->deriv_buf, 3 * sizeof(float));
    PID->deriv_buf[0] = filtered_deriv;
    PID->prev_filtered = filtered_deriv;
    PID->prev_error = error;
    PID->prev_position = current_position;
    PID->last_output = output;

    return fmaxf(fminf(output, PID->max_output), -PID->max_output);
}

//************************************************************************************************************ */

// 转到特定角度的函数
uint8_t Move_To_Position_Z(float target_omega, uint32_t TIMEOUT)
{
    // 仅修改此处：K → K_z
    float K_z = 1.0f; // 修改点1：变量名添加_z后缀

    // float omega_basic = omega;
    // float omega_ref = omega_basic + target_omega;
    uint32_t basic_time = HAL_GetTick();
    float current_w = 0;
    static float last_omega = 0;

    while (1)
    {
        current_w = PID_calc_z(target_omega, 0, outer_PID_omega_f);

        // float txBuffer_35_x[1] = {0};
        // txBuffer_35_x[0] = omega;

        // SendMultiFloat2Vofa(txBuffer_35_x, 1);

        // 麦轮运动学解算
        float Car_H = 1.67f;
        float Car_W = 2.38f;

        // 修改点2-5：K → K_z
        int16_t Vel1 = 4 * (int16_t)(K_z * (-current_w * (Car_H / 2 + Car_W / 2)));
        int16_t Vel2 = 4 * (int16_t)(K_z * (+current_w * (Car_H / 2 + Car_W / 2)));
        int16_t Vel3 = 4 * (int16_t)(K_z * (-current_w * (Car_H / 2 + Car_W / 2)));
        int16_t Vel4 = 4 * (int16_t)(K_z * (+current_w * (Car_H / 2 + Car_W / 2)));

        // 后续代码完全不变
        uint8_t acc = 150, snF = 1;

        Emm_V5_Send_Four_Vel_Control(Vel1, Vel2, Vel3, Vel4, acc, snF, 3);

        if (HAL_GetTick() - basic_time >= TIMEOUT)
        {
            PIDController_reset(outer_PID_omega_f);
            Emm_V5_Stop_Now(0x00, 0);
            break;
        }
        last_omega = omega;
    }
}

uint8_t Move_To_Position_X(float target_distance_x, uint32_t TIMEOUT_x)
{
    // 修改的变量（添加_x后缀）
    float error_x = 0.0f;
    float K_x = 1.0f;

    // 保留的变量（不修改）
    uint32_t start_time = HAL_GetTick();
    uint32_t current_time = start_time;
    uint8_t use_timeout = (TIMEOUT_x > 0);

    Update_Car_Status(10);

    // 修改的变量（添加_x后缀）
    float initial_distance_x = (motor1_status->distance_traveled +
                                motor2_status->distance_traveled +
                                motor3_status->distance_traveled +
                                motor4_status->distance_traveled) /
                               NUM_MOTORS;

    // 修改的变量（添加_x后缀）
    float target_displacement_x = initial_distance_x + target_distance_x;

    while (1)
    {
        Update_Car_Status(5);

        // 修改的变量（添加_x后缀）
        float current_distance_x = (motor1_status->distance_traveled +
                                    motor2_status->distance_traveled +
                                    motor3_status->distance_traveled +
                                    motor4_status->distance_traveled) /
                                   NUM_MOTORS;

        // 修改的变量（添加_x后缀）
        error_x = target_displacement_x - current_distance_x;

        // 修改的变量（添加_x后缀）
        float txBuffer_35_x[2] = {0};
        txBuffer_35_x[0] = error_x;
        txBuffer_35_x[1] = (motor1_status->current_speed + motor2_status->current_speed + motor3_status->current_speed + motor4_status->current_speed) / 4;
        txBuffer_35_x[2] = initial_distance_x;
        // txBuffer_35_x[3] = target_displacement_x
        SendMultiFloat2Vofa(txBuffer_35_x, 2);

        // 修改的变量（添加_x后缀）printf("");
        float target_speed_x = PID_calc(error_x, inner_PID_X);

        // 保留的变量（不修改）
        int16_t Vel1 = (int16_t)(K_x * target_speed_x);
        int16_t Vel2 = (int16_t)(K_x * target_speed_x);
        int16_t Vel3 = (int16_t)(K_x * target_speed_x);
        int16_t Vel4 = (int16_t)(K_x * target_speed_x);

        // 保留的变量（不修改）
        bool snF = 1;
        uint8_t acc = 150;

        Emm_V5_Send_Four_Vel_Control(Vel1, Vel2, Vel3, Vel4, acc, snF, 5);

        // 保留的变量（不修改）
        if (use_timeout)
        {
            current_time = HAL_GetTick();
            if (current_time - start_time >= TIMEOUT_x)
            {
                Emm_V5_Stop_Now(0x00, 0);
                break;
            }
        }
    }
}

uint8_t Move_To_Position_Y(float target_distance_y, uint32_t TIMEOUT_y)
{
    // 修改的变量（添加_y后缀）
    float error_y = 0.0f;
    float K_y = 1.0f;

    // 保留的变量（不修改）
    uint32_t start_time = HAL_GetTick();
    uint32_t current_time = start_time;
    uint8_t use_timeout = (TIMEOUT_y > 0);

    Update_Car_Status(5);

    // 修改的变量（添加_y后缀）（初始情况是取左边为正方向的，这个要注意好了）
    float initial_distance_y = (-motor1_status->distance_traveled +
                                motor2_status->distance_traveled +
                                motor3_status->distance_traveled -
                                motor4_status->distance_traveled) /
                               NUM_MOTORS;

    // 修改的变量（添加_y后缀）
    float target_displacement_y = initial_distance_y + target_distance_y;

    while (1)
    {
        Update_Car_Status(5);

        // 修改的变量（添加_y后缀）（这里注意进行了符号的修改，- + + - ）
        float current_distance_y = (-motor1_status->distance_traveled +
                                    motor2_status->distance_traveled +
                                    motor3_status->distance_traveled -
                                    motor4_status->distance_traveled) /
                                   NUM_MOTORS;

        // 修改的变量（添加_y后缀）
        error_y = target_displacement_y - current_distance_y;

        // 修改的变量（添加_y后缀）
        float txBuffer_35_y[1] = {error_y};
        SendMultiFloat2Vofa(txBuffer_35_y, 1);

        // 修改的变量（添加_y后缀）& 假设存在独立的Y轴PID控制器
        float actual_speed_y = PID_calc(error_y, inner_PID_Y);

        // 保留的变量（不修改）
        int16_t Vel1 = (int16_t)(-K_y * actual_speed_y);
        int16_t Vel2 = (int16_t)(+K_y * actual_speed_y);
        int16_t Vel3 = (int16_t)(+K_y * actual_speed_y);
        int16_t Vel4 = (int16_t)(-K_y * actual_speed_y);

        // 保留的变量（不修改）
        bool snF = 1;
        uint8_t acc = 20;

        Emm_V5_Send_Four_Vel_Control(Vel1, Vel2, Vel3, Vel4, acc, snF, 5);

        // 保留的变量（不修改）
        if (use_timeout)
        {
            current_time = HAL_GetTick();
            if (current_time - start_time >= TIMEOUT_y)
            {
                Emm_V5_Stop_Now(0x00, 0);
                break;
            }
        }
    }
}

void Move_To_Position_XYZ(float target_position_x, float target_position_y, float target_omega, uint32_t TIMEOUT)
{
    Reset_Car_Status();
    Update_Car_Status(5);
    PIDController_reset(inner_PID_X);
    PIDController_reset(outer_PID_omega_f);
    PIDController_reset(inner_PID_Y);
    PIDController_reset(inner_PID_X_Col);
    PIDController_reset(inner_PID_Y_Col);
    // 本次相对运动开始时候的角度
    float init_omega = omega;
    static uint8_t stability_counter = 0;
    uint8_t stability_counter_threshold = 5;
    float x_error_ref = 1.5f;
    float y_error_ref = 1.5f;

    if (target_position_x < 120.0f)
    {
        x_error_ref = 1.5f;
    }
    else if (target_position_x > 120.0f && target_position_x < 200.0f)
    {
        x_error_ref = 2.0f;
    }
    else if (target_position_x > 200.0f && target_position_x < 467.0f)
    {
        x_error_ref = 3.5f;
    }
    else if (target_position_x > 467.0f && target_position_x < 1000.0f)
    {
        x_error_ref = 6.5f;
    }
    else
    {
        x_error_ref = 10.0f;
    }

    if (target_position_y < 100.0f)
    {
        y_error_ref = 1.5f;
    }
    else if (target_position_y > 100.0f && target_position_y < 200.0f)
    {
        y_error_ref = 2.0f;
    }
    else if (target_position_y > 200.0f && target_position_y < 467.0f)
    {
        y_error_ref = 3.5f;
    }
    else if (target_position_y > 467.0f && target_position_y < 1000.0f)
    {
        y_error_ref = 6.5f;
    }
    else
    {
        y_error_ref = 10.0f;
    }

    // 公共参数
    uint32_t start_time = HAL_GetTick();

    while (1)
    { ///////////////////////////////这里进行了修改，

        if ((fabs(car->current_map_position_x - target_position_x) < x_error_ref) && (fabs(car->current_map_position_y - target_position_y) < y_error_ref) && (fabs(omega - target_omega) < 0.8f))
        {
            // 使用静态计数器记录连续满足条件的次数
            stability_counter++;

            // 当连续满足精度要求的次数达到阈值后，才认为真正到达目标位置
            if (stability_counter >= stability_counter_threshold) // 假设连续10次检测都满足条件
            {

                Car_Stop(1);
                break;
            }
        }
        else
        {
            // 一旦不满足条件，立即重置计数器
            stability_counter = 0;
        }

        if (HAL_GetTick() - start_time >= TIMEOUT)
        {

            Car_Stop(1);
            break;
        }
        Update_Car_Status(4);

        // vofa监测
        float txBuffer_35_x[26] = {0};
        // 计算出来的位移差值
        txBuffer_35_x[0] = target_position_x - car->current_map_position_x;
        // 角度差值
        txBuffer_35_x[1] = target_omega - omega;

        txBuffer_35_x[2] = current_angle_speed;
        txBuffer_35_x[3] = car->target_angle_speed;

        txBuffer_35_x[4] = car->current_map_position_x;
        txBuffer_35_x[5] = car->current_map_Vx;
        txBuffer_35_x[6] = car->target_map_Vx;
        txBuffer_35_x[7] = car->current_car_Vx;

        txBuffer_35_x[8] = car->current_map_position_y;
        txBuffer_35_x[9] = car->current_map_Vy;
        txBuffer_35_x[10] = car->target_map_Vy;
        txBuffer_35_x[11] = car->current_car_Vy;

        txBuffer_35_x[12] = car->delta_car_position_x;
        txBuffer_35_x[13] = car->delta_car_position_y;

        txBuffer_35_x[14] = car->current_Vel1;
        txBuffer_35_x[15] = car->current_Vel2;
        txBuffer_35_x[16] = car->current_Vel3;
        txBuffer_35_x[17] = car->current_Vel4;

        txBuffer_35_x[18] = motor1_status->distance_traveled;
        txBuffer_35_x[19] = motor2_status->distance_traveled;
        txBuffer_35_x[20] = motor3_status->distance_traveled;
        txBuffer_35_x[21] = motor4_status->distance_traveled;
        txBuffer_35_x[22] = motor1_status->real_angle;

        txBuffer_35_x[23] = inner_PID_X->dt;
        txBuffer_35_x[24] = car->target_car_Vx;
        txBuffer_35_x[25] = car->target_car_Vy;

        SendMultiFloat2Vofa(txBuffer_35_x, 26);
        /// 这里再加上一个判断逻辑，就是看是否都有输入

        if (car->car_face == Forward || car->car_face == Back)
        {
            car->target_map_Vx = PID_calc(target_position_x - car->current_map_position_x, inner_PID_X);

            car->target_map_Vy = PID_calc_Y(target_position_y - car->current_map_position_y, inner_PID_Y);
        }
        else
        {
            car->target_map_Vx = PID_calc(target_position_x - car->current_map_position_x, inner_PID_Y);

            car->target_map_Vy = PID_calc_Y(target_position_y - car->current_map_position_y, inner_PID_X);
        }

        float target_map_all_speed = sqrt(car->target_map_Vx * car->target_map_Vx + car->target_map_Vy * car->target_map_Vy);

        // 这个是一个简单的按照速度进行分配的函数，也相当于耦合吧
        car->target_map_Vx *= fabs(car->target_map_Vx) / target_map_all_speed;
        car->target_map_Vy *= fabs(car->target_map_Vy) / target_map_all_speed;

        // 希望在这里加一个新的算法控制逻辑
        //  target_angle_speed计算

        car->target_angle_speed = PID_calc_z(target_omega, init_omega, outer_PID_omega_f);

        // car->target_map_Vx = 500;
        // car->target_map_Vy =0;
        Publish_motor_speed();
    }
    stability_counter = 0;

    PIDController_reset(outer_PID_omega_f);
    PIDController_reset(inner_PID_X);
    PIDController_reset(inner_PID_Y);
    PIDController_reset(inner_PID_X_Col);
    PIDController_reset(inner_PID_Y_Col);
    HAL_Delay(50);
}



void Move_To_Position_XYZ_Color(float target_position_x, float target_position_y, float target_omega, uint32_t TIMEOUT)
{
    Reset_Car_Status();
    Update_Car_Status(5);
    PIDController_reset(inner_PID_X);
    PIDController_reset(outer_PID_omega_f);
    PIDController_reset(inner_PID_Y);
    PIDController_reset(inner_PID_X_Col);
    PIDController_reset(inner_PID_Y_Col);
    // 本次相对运动开始时候的角度
    float init_omega = omega;
    static uint8_t stability_counter = 0;
    uint8_t stability_counter_threshold = 5;
    float x_error_ref = 6.0f;
    float y_error_ref = 6.0f;

    

    // 公共参数
    uint32_t start_time = HAL_GetTick();

    while (1)
    { ///////////////////////////////这里进行了修改，

        if ((fabs(car->current_map_position_x - target_position_x) < x_error_ref) && (fabs(car->current_map_position_y - target_position_y) < y_error_ref) && (fabs(omega - target_omega) < 1.5f))
        {
            // 使用静态计数器记录连续满足条件的次数
            stability_counter++;

            // 当连续满足精度要求的次数达到阈值后，才认为真正到达目标位置
            if (stability_counter >= stability_counter_threshold) // 假设连续10次检测都满足条件
            {

                Car_Stop(1);
                break;
            }
        }
        else
        {
            // 一旦不满足条件，立即重置计数器
            stability_counter = 0;
        }

        if (HAL_GetTick() - start_time >= TIMEOUT)
        {

            Car_Stop(1);
            break;
        }
        Update_Car_Status(4);

        // vofa监测
        float txBuffer_35_x[26] = {0};
        // 计算出来的位移差值
        txBuffer_35_x[0] = target_position_x - car->current_map_position_x;
        // 角度差值
        txBuffer_35_x[1] = target_omega - omega;

        txBuffer_35_x[2] = current_angle_speed;
        txBuffer_35_x[3] = car->target_angle_speed;

        txBuffer_35_x[4] = car->current_map_position_x;
        txBuffer_35_x[5] = car->current_map_Vx;
        txBuffer_35_x[6] = car->target_map_Vx;
        txBuffer_35_x[7] = car->current_car_Vx;

        txBuffer_35_x[8] = car->current_map_position_y;
        txBuffer_35_x[9] = car->current_map_Vy;
        txBuffer_35_x[10] = car->target_map_Vy;
        txBuffer_35_x[11] = car->current_car_Vy;

        txBuffer_35_x[12] = car->delta_car_position_x;
        txBuffer_35_x[13] = car->delta_car_position_y;

        txBuffer_35_x[14] = car->current_Vel1;
        txBuffer_35_x[15] = car->current_Vel2;
        txBuffer_35_x[16] = car->current_Vel3;
        txBuffer_35_x[17] = car->current_Vel4;

        txBuffer_35_x[18] = motor1_status->distance_traveled;
        txBuffer_35_x[19] = motor2_status->distance_traveled;
        txBuffer_35_x[20] = motor3_status->distance_traveled;
        txBuffer_35_x[21] = motor4_status->distance_traveled;
        txBuffer_35_x[22] = motor1_status->real_angle;

        txBuffer_35_x[23] = inner_PID_X->dt;
        txBuffer_35_x[24] = car->target_car_Vx;
        txBuffer_35_x[25] = car->target_car_Vy;

        SendMultiFloat2Vofa(txBuffer_35_x, 26);
        /// 这里再加上一个判断逻辑，就是看是否都有输入

        if (car->car_face == Forward || car->car_face == Back)
        {
            car->target_map_Vx = PID_calc(target_position_x - car->current_map_position_x, inner_PID_X);

            car->target_map_Vy = PID_calc_Y(target_position_y - car->current_map_position_y, inner_PID_Y);
        }
        else
        {
            car->target_map_Vx = PID_calc(target_position_x - car->current_map_position_x, inner_PID_Y);

            car->target_map_Vy = PID_calc_Y(target_position_y - car->current_map_position_y, inner_PID_X);
        }

        float target_map_all_speed = sqrt(car->target_map_Vx * car->target_map_Vx + car->target_map_Vy * car->target_map_Vy);

        // 这个是一个简单的按照速度进行分配的函数，也相当于耦合吧
        car->target_map_Vx *= fabs(car->target_map_Vx) / target_map_all_speed;
        car->target_map_Vy *= fabs(car->target_map_Vy) / target_map_all_speed;

        // 希望在这里加一个新的算法控制逻辑
        //  target_angle_speed计算

        car->target_angle_speed = PID_calc_z(target_omega, init_omega, outer_PID_omega_f);

        // car->target_map_Vx = 500;
        // car->target_map_Vy =0;
        Publish_motor_speed();
    }
    stability_counter = 0;

    PIDController_reset(outer_PID_omega_f);
    PIDController_reset(inner_PID_X);
    PIDController_reset(inner_PID_Y);
    PIDController_reset(inner_PID_X_Col);
    PIDController_reset(inner_PID_Y_Col);
    HAL_Delay(50);

}

void Turn_To(float target_omega, float angle_error_limit, uint32_t TIMEOUT)
{
    Reset_Car_Status();
    float init_omega = omega;

    Move_TransfromZ(target_omega - init_omega);

    static uint8_t stability_counter = 0;
    uint8_t stability_counter_threshold = 5;
    uint32_t start_time = HAL_GetTick();

    while (1)
    { ///////////////////////////////这里进行了修改，

        if ((fabs(omega - target_omega) < angle_error_limit))
        {
            // 使用静态计数器记录连续满足条件的次数
            stability_counter++;

            // 当连续满足精度要求的次数达到阈值后，才认为真正到达目标位置
            if (stability_counter >= stability_counter_threshold) // 假设连续10次检测都满足条件
            {

                break;
            }
        }
        else
        {
            // 一旦不满足条件，立即重置计数器
            stability_counter = 0;
        }

        if (HAL_GetTick() - start_time >= TIMEOUT)
        {

            break;
        }
    }

    HAL_Delay(50);

}

/// @brief 在确认进入函数之前，要确保已经收到了上位机发来的数据
/// @param target_omega
/// @param TIMEOUT
/// @param      /////////这个超时最好设置成6000ms，多进行几次调整
void Car_Blog(uint32_t TIMEOUT, int8_t height)
{
    // 这个前面给的是相对的
    // 还有就是在精确的对准的时候，会有两个角度和高度
    // 所以会需要对应不同的k的数值
    Update_Car_Status(5);
    float target_position_x = 0.0f;
    float target_position_y = 0.0f;
    float target_omega = 0.0f;
    while (1)
    {
        if (fabs(raspi_date->ref_x) > 1.0f && fabs(raspi_date->ref_y) > 1.0f)
        {
            break;
        }
    }
    float pixel_x = raspi_date->ref_x;
    float pixel_y = raspi_date->ref_y; ////0.1228f
    float pixel_to_x = 0.4f;
    float pixel_to_y = 0.4f; // 取物块时候的参数
    /// 这个比例实际上是受到x方向是否有位移影响的，假设x方向有那啥，就会动的少一些，否则会多一些
    float target_position_x_relative = 0.0f;
    float target_position_y_relative = 0.0f;
    if (height == 0)
    {
        pixel_to_x = 0.4f;
        pixel_to_y = 0.4f; // 无聊盘取物块的参数
        target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[0]) * pixel_to_x;
        target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[0]) * pixel_to_y;
    }
    else if (height == 1)
    { ///////////////粗调高度
        pixel_to_x = 0.83f;
        pixel_to_y = 0.81f; // 抓取抬升高度
        ////这个y的比例实际上测出来，0.79是最精确的，但是最终因为还需要有x方向的移动，所以需要稍微调大一些
        target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[1]) * pixel_to_x;
        target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[1]) * pixel_to_y;
    }
    else if (height == 2)
    { ///////////////粗调高度
        pixel_to_x = 0.4f;
        pixel_to_y = 0.4f; // 抓取抬升高度
        ////这个y的比例实际上测出来，0.79是最精确的，但是最终因为还需要有x方向的移动，所以需要稍微调大一些
        target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[2]) * pixel_to_x;
        target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[2]) * pixel_to_y;
    }
    else if (height == 3)
    { ///////////////粗调高度
        pixel_to_x = 0.3f;
        pixel_to_y = 0.3f; // 抓取抬升高度
        ////这个y的比例实际上测出来，0.79是最精确的，但是最终因为还需要有x方向的移动，所以需要稍微调大一些
        target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[3]) * pixel_to_x;
        target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[3]) * pixel_to_y;
    }
    else if (height == 4)
    { ///////////////粗调高度
        pixel_to_x = 0.4f;
        pixel_to_y = 0.4f; // 抓取抬升高度
        ////这个y的比例实际上测出来，0.79是最精确的，但是最终因为还需要有x方向的移动，所以需要稍微调大一些
        target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[4]) * pixel_to_x;
        target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[4]) * pixel_to_y;
    }
    else if (height == 5)
    { ///////////////粗调高度
        pixel_to_x = 0.83f;
        pixel_to_y = 0.81f; // 抓取抬升高度
        ////这个y的比例实际上测出来，0.79是最精确的，但是最终因为还需要有x方向的移动，所以需要稍微调大一些
        target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[5]) * pixel_to_x;
        target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[5]) * pixel_to_y;
    }
    else if (height == 6)
    { ///////////////粗调高度
        pixel_to_x = 0.3f;
        pixel_to_y = 0.3f; // 抓取抬升高度
        ////这个y的比例实际上测出来，0.79是最精确的，但是最终因为还需要有x方向的移动，所以需要稍微调大一些
        target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[6]) * pixel_to_x;
        target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[6]) * pixel_to_y;
    }
    else if (height == 7)
    { ///////////////粗调高度
        pixel_to_x = 0.4f;
        pixel_to_y = 0.4f; // 抓取抬升高度
        ////这个y的比例实际上测出来，0.79是最精确的，但是最终因为还需要有x方向的移动，所以需要稍微调大一些
        target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[7]) * pixel_to_x;
        target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[7]) * pixel_to_y;
    }

    /// 换算到绝对坐标系并且交给movexyz函数执行
    /// 但是有一个问题，就是误差范围限制，可能需要作为参数输入

    /////还有一个可能的逻辑问题，就是假设是那一个角度
    switch (car->car_face)
    {
    case Forward:
        target_omega = 0.0f;
        target_position_x = target_position_x_relative;
        target_position_y = target_position_y_relative;
        break;
    case Left:
        target_omega = 90.0f;
        target_position_x = -target_position_y_relative;
        target_position_y = target_position_x_relative;
        break;
    case Back:
        target_omega = 180.0f;
        target_position_x = -target_position_x_relative;
        target_position_y = -target_position_y_relative;
        break;
    case Right:
        target_omega = 270.0f;
        target_position_x = target_position_y_relative;
        target_position_y = -target_position_x_relative;
        break;
    default:
        target_omega = 0.0f;
        target_position_x = target_position_x_relative;
        target_position_y = target_position_y_relative;
        break;
    }

    Move_To_Position_XYZ(target_position_x, target_position_y, target_omega, TIMEOUT);
}

void Car_Blog_Test(uint32_t TIMEOUT, enum Height height)
{
    // 这个前面给的是相对的
    // 还有就是在精确的对准的时候，会有两个角度和高度
    // 所以会需要对应不同的k的数值
    Update_Car_Status(5);
    float target_position_x = 0.0f;
    float target_position_y = 0.0f;
    float target_omega = 0.0f;
    while (1)
    {
        HAL_Delay(50); /// 在循环等待检测数据时候
        if (fabs(raspi_date->ref_x) > 1.0f && fabs(raspi_date->ref_y) > 1.0f)
        {
            break;
        }
    }
    float pixel_x = raspi_date->ref_x;
    float pixel_y = raspi_date->ref_y; 
    static const float pixel_to_x[8] = {0.4f, 0.83f, 0.3f, 0.4f, 0.4f, 0.83f, 0.3f, 0.4f};
    static const float pixel_to_y[8] = {0.4f, 0.81f, 0.3f, 0.4f, 0.4f, 0.81f, 0.3f, 0.4f}; // 取物块时候的参数
    /// 这个比例实际上是受到x方向是否有位移影响的，假设x方向有那啥，就会动的少一些，否则会多一些
    float target_position_x_relative = 0.0f;
    float target_position_y_relative = 0.0f;

    target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[height]) * pixel_to_x[height];
    target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[height]) * pixel_to_y[height];

    /// 换算到绝对坐标系并且交给movexyz函数执行
    /// 但是有一个问题，就是误差范围限制，可能需要作为参数输入

    /////还有一个可能的逻辑问题，就是假设是那一个角度
    switch (car->car_face)
    {
    case Forward:
        target_omega = 0.0f;
        target_position_x = target_position_x_relative;
        target_position_y = target_position_y_relative;
        break;
    case Left:
        target_omega = 90.0f;
        target_position_x = -target_position_y_relative;
        target_position_y = target_position_x_relative;
        break;
    case Back:
        target_omega = 180.0f;
        target_position_x = -target_position_x_relative;
        target_position_y = -target_position_y_relative;
        break;
    case Right:
        target_omega = 270.0f;
        target_position_x = target_position_y_relative;
        target_position_y = -target_position_x_relative;
        break;
    default:
        target_omega = 0.0f;
        target_position_x = target_position_x_relative;
        target_position_y = target_position_y_relative;
        break;
    }

    Move_To_Position_XYZ(target_position_x, target_position_y, target_omega, TIMEOUT);
}

void Car_Ring(uint32_t TIMEOUT)
{
    int16_t K = 1;              // 一个用于速度调整的参数
    uint32_t timeout = TIMEOUT; // 假如有外部的输入，那就用外部的
    // 还是不能这么写因为我们不能默认当前的角度就是我们想要的那个角度，所以要设置一个全局变量
    float init_omega = omega;
    static uint8_t stability_counter = 0;
    // stability_counter = 0;   //每次进入之前重置也是可以的
    uint8_t stability_counter_threshold = 5;

    uint32_t start_time = HAL_GetTick();

    float pixel_x = raspi_date->ref_x;
    float pixel_y = raspi_date->ref_y; ////0.1228f

    float target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[2]);
    float target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[2]);

    float target_omega = 0.0f;

    // Z轴角度控制
    float W = 0.0f;
    // float ref_Z = -(raspi_date->ref_z);//这个是之前测试出过问题的，值得重视

    // X轴位置控制
    float Vx = 0.0f;
    // Y轴位置控制
    float Vy = 0.0f;
    // 通过car_face来确定目标角度
    switch (car->car_face)
    {
        ////////这里进行了代码的修改
    case Forward:
        target_omega = 0.0f;
        break;
    case Left:
        target_omega = 90.0f;
        break;
    case Back:
        target_omega = 180.0f;
        break;
    case Right:
        target_omega = 270.0f;
        break;
    default:
        target_omega = 0.0f;
        break;
    }
    float angle = (omega - target_omega) * PI / 180.0f;

    Reset_Car_Status();

    PIDController_reset(outer_PID_f);
    PIDController_reset(outer_PID_Y_f);
    PIDController_reset(outer_PID_Z_f);

    while (1)
    {

        /////超时退出

        if (HAL_GetTick() - start_time > TIMEOUT)
        {
            Car_Stop(0);
            break;
        }

        // ========================= [非阻塞数据有效性检测] =========================
        if (fabs(raspi_date->ref_x) < 0.1f || fabs(raspi_date->ref_y) < 0.1f)
        {
            continue; // 跳过无效数据
        }

        /////运动的执行代码

        pixel_x = raspi_date->ref_x;
        pixel_y = raspi_date->ref_y; ////0.1228f

        target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[2]);
        target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[2]);

        if (fabs(target_position_x_relative) < 0.5f && fabs(target_position_y_relative) < 0.5f && fabs(target_omega - omega) < 2.0f)
        {
            // 使用静态计数器记录连续满足条件的次数
            stability_counter++;

            // 当连续满足精度要求的次数达到阈值后，才认为真正到达目标位置
            if (stability_counter >= stability_counter_threshold) // 假设连续10次检测都满足条件
            {

                Car_Stop(0);
                break;
            }
        }
        else
        {
            // 一旦不满足条件，立即重置计数器
            stability_counter = 0;
        }

        angle = (omega - target_omega) * PI / 180.0f;

        /// 需要注意，这里输入的应该是error，所以应该在那个输入的时候，就进行那啥

        Vx = PID_calc_micro(target_position_x_relative, outer_PID_f);

        // int16_t ref_Y = (int16_t)(packet_2->ref_y);
        //  Vy = (int16_t)(outer_PID_Y((float)ref_Y));
        // 这里暂时修改一下，要用omega的参数来实现校准

        Vy = PID_calc_Y_micro(target_position_y_relative, outer_PID_Y_f);

        /// 这个角度是负数的需要时刻铭记
        W = PID_calc_z_micro(target_omega, init_omega, outer_PID_Z_f);

        // 麦轮运动学解算
        float Car_H = 1.67f;
        float Car_W = 2.38f;
        float Vel1 = (K * (-W * (Car_H / 2 + Car_W / 2))) + K * Vx - K * Vy;
        float Vel2 = (K * (+W * (Car_H / 2 + Car_W / 2))) + K * Vx + K * Vy;
        float Vel3 = (K * (-W * (Car_H / 2 + Car_W / 2))) + K * Vx + K * Vy;
        float Vel4 = (K * (+W * (Car_H / 2 + Car_W / 2))) + K * Vx - K * Vy;

        uint8_t acc = 10;
        bool snF = 1;
        uint32_t delay_ms = 5;

        Emm_V5_Send_Four_Vel_Control(Vel1, Vel2, Vel3, Vel4, acc, snF, delay_ms);

        /// 以下是调试用代码

        float txBuffer_35_x[10] = {0};
        // 计算出来的位移差值
        txBuffer_35_x[0] = target_position_x_relative;

        txBuffer_35_x[1] = target_position_y_relative;
        // 角度差值
        txBuffer_35_x[2] = target_omega - omega;

        txBuffer_35_x[3] = W;

        txBuffer_35_x[4] = Vx;

        txBuffer_35_x[5] = Vy;
        txBuffer_35_x[6] = raspi_date->ref_x;
        txBuffer_35_x[7] = raspi_date->ref_y;

        txBuffer_35_x[8] = raspi_date->vision_current_position_x[2];
        txBuffer_35_x[9] = raspi_date->vision_current_position_y[2];

        SendMultiFloat2Vofa(txBuffer_35_x, 10);
    }

    stability_counter = 0;

    PIDController_reset(outer_PID_f);
    PIDController_reset(outer_PID_Y_f);
    PIDController_reset(outer_PID_Z_f);

    HAL_Delay(50);
}


void Car_Ring_Timeout(uint32_t TIMEOUT)
{
    int16_t K = 1;              // 一个用于速度调整的参数
    uint32_t timeout = TIMEOUT; // 假如有外部的输入，那就用外部的
    // 还是不能这么写因为我们不能默认当前的角度就是我们想要的那个角度，所以要设置一个全局变量
    float init_omega = omega;
    static uint8_t stability_counter = 0;
    // stability_counter = 0;   //每次进入之前重置也是可以的
    uint8_t stability_counter_threshold = 5;
    stability_counter = 0; // 每次进入函数时重置

    uint32_t running_time = 0;
    volatile CommsState comms_state = STATE_NORMAL;  // 初始状态为正常
    uint32_t last_receive = last_receive_raspi_time; // 读取最新时间戳

    uint32_t current_time = HAL_GetTick();

    uint32_t last_control_time = 0; // 静态变量保持状态
    uint32_t current_control_time = 0;

    float pixel_x = raspi_date->ref_x;
    float pixel_y = raspi_date->ref_y; ////0.1228f

    float target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[2]);
    float target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[2]);

    float target_omega = 0.0f;

    // Z轴角度控制
    float W = 0.0f;
    // float ref_Z = -(raspi_date->ref_z);//这个是之前测试出过问题的，值得重视

    // X轴位置控制
    float Vx = 0.0f;
    // Y轴位置控制
    float Vy = 0.0f;
    // 通过car_face来确定目标角度
    switch (car->car_face)
    {
        ////////这里进行了代码的修改
    case Forward:
        target_omega = 0.0f;
        break;
    case Left:
        target_omega = 90.0f;
        break;
    case Back:
        target_omega = 180.0f;
        break;
    case Right:
        target_omega = 270.0f;
        break;
    default:
        target_omega = 0.0f;
        break;
    }
    float angle = (omega - target_omega) * PI / 180.0f;

    Reset_Car_Status();

    PIDController_reset(outer_PID_f);
    PIDController_reset(outer_PID_Y_f);
    PIDController_reset(outer_PID_Z_f);

    while (1)
    {

        current_time = HAL_GetTick();
        last_receive = last_receive_raspi_time; // 读取最新时间戳

        switch (comms_state)
        {
        case STATE_NORMAL:
            if (current_time - last_receive > 200)
            {
                comms_state = STATE_TIMEOUT;

                last_control_time = 0; // 重置控制时间戳

                Car_Stop(1);
            }
            break;

        case STATE_TIMEOUT:
            if (current_time - last_receive < 200)
            {
                comms_state = STATE_NORMAL;
            }
            break;
        }

        if (running_time > TIMEOUT)
        {
            Car_Stop(0);
            break;
        }

        // ========================= [非阻塞数据有效性检测] =========================
        if (fabs(raspi_date->ref_x) < 0.1f || fabs(raspi_date->ref_y) < 0.1f)
        {
            continue; // 跳过无效数据.continue是跳过当前循环，继续执行下一个循环
        }

        /////只有在正常数据情况下才执行代码

        if (comms_state == STATE_NORMAL)
        {

            pixel_x = raspi_date->ref_x;
            pixel_y = raspi_date->ref_y; ////0.1228f

            target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[2]);
            target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[2]);

            if (fabs(target_position_x_relative) < 0.8f && fabs(target_position_y_relative) < 0.8f && fabs(target_omega - omega) < 2.0f)
            {
                // 使用静态计数器记录连续满足条件的次数
                stability_counter++;

                // 当连续满足精度要求的次数达到阈值后，才认为真正到达目标位置
                if (stability_counter >= stability_counter_threshold) // 假设连续10次检测都满足条件
                {

                    Car_Stop(0);
                    break;
                }
            }
            else
            {
                // 一旦不满足条件，立即重置计数器
                stability_counter = 0;
            }

            angle = (omega - target_omega) * PI / 180.0f;

            /// 需要注意，这里输入的应该是error，所以应该在那个输入的时候，就进行那啥

            Vx = PID_calc_micro(target_position_x_relative, outer_PID_f);

            // int16_t ref_Y = (int16_t)(packet_2->ref_y);
            //  Vy = (int16_t)(outer_PID_Y((float)ref_Y));
            // 这里暂时修改一下，要用omega的参数来实现校准

            Vy = PID_calc_Y_micro(target_position_y_relative, outer_PID_Y_f);

            /// 这个角度是负数的需要时刻铭记
            W = PID_calc_z_micro(target_omega, init_omega, outer_PID_Z_f);

            // 麦轮运动学解算
            float Car_H = 1.67f;
            float Car_W = 2.38f;
            float Vel1 = (K * (-W * (Car_H / 2 + Car_W / 2))) + K * Vx - K * Vy;
            float Vel2 = (K * (+W * (Car_H / 2 + Car_W / 2))) + K * Vx + K * Vy;
            float Vel3 = (K * (-W * (Car_H / 2 + Car_W / 2))) + K * Vx + K * Vy;
            float Vel4 = (K * (+W * (Car_H / 2 + Car_W / 2))) + K * Vx - K * Vy;

            uint8_t acc = 10;
            bool snF = 1;
            uint32_t delay_ms = 5;

            Emm_V5_Send_Four_Vel_Control(Vel1, Vel2, Vel3, Vel4, acc, snF, delay_ms);

            current_control_time = HAL_GetTick();

            if (last_control_time != 0)
            {
                running_time += (current_control_time - last_control_time);
            }
            last_control_time = current_control_time; // 更新控制时间戳

            /// 以下是调试用代码

            float txBuffer_35_x[10] = {0};
            // 计算出来的位移差值
            txBuffer_35_x[0] = target_position_x_relative;

            txBuffer_35_x[1] = target_position_y_relative;
            // 角度差值
            txBuffer_35_x[2] = target_omega - omega;

            txBuffer_35_x[3] = W;

            txBuffer_35_x[4] = Vx;

            txBuffer_35_x[5] = Vy;
            txBuffer_35_x[6] = raspi_date->ref_x;
            txBuffer_35_x[7] = raspi_date->ref_y;

            txBuffer_35_x[8] = raspi_date->vision_current_position_x[2];
            txBuffer_35_x[9] = raspi_date->vision_current_position_y[2];

            SendMultiFloat2Vofa(txBuffer_35_x, 10);
        }
        ///////////每次执行控制之后，再进行一个时间戳的更新
    }

    stability_counter = 0;

    PIDController_reset(outer_PID_f);

    PIDController_reset(outer_PID_Y_f);
    
    PIDController_reset(outer_PID_Z_f);

    HAL_Delay(50);

}


void Car_Ring_Timeout_Test(uint32_t TIMEOUT, enum Height height)
{
    int16_t K = 1;              // 一个用于速度调整的参数
    uint32_t timeout = TIMEOUT; // 假如有外部的输入，那就用外部的
    // 还是不能这么写因为我们不能默认当前的角度就是我们想要的那个角度，所以要设置一个全局变量
    float init_omega = omega;
    static uint8_t stability_counter = 0;
    // stability_counter = 0;   //每次进入之前重置也是可以的
    uint8_t stability_counter_threshold = 5;
    stability_counter = 0; // 每次进入函数时重置

    uint32_t running_time = 0;
    volatile CommsState comms_state = STATE_NORMAL;  // 初始状态为正常
    uint32_t last_receive = last_receive_raspi_time; // 读取最新时间戳

    uint32_t current_time = HAL_GetTick();

    uint32_t last_control_time = 0; // 静态变量保持状态
    uint32_t current_control_time = 0;

    float pixel_x = raspi_date->ref_x;
    float pixel_y = raspi_date->ref_y; ////0.1228f

    float target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[height]);
    float target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[height]);

    float target_omega = 0.0f;

    // Z轴角度控制
    float W = 0.0f;
    // float ref_Z = -(raspi_date->ref_z);//这个是之前测试出过问题的，值得重视

    // X轴位置控制
    float Vx = 0.0f;
    // Y轴位置控制
    float Vy = 0.0f;
    // 通过car_face来确定目标角度
    switch (car->car_face)
    {
        ////////这里进行了代码的修改
    case Forward:
        target_omega = 0.0f;
        break;
    case Left:
        target_omega = 90.0f;
        break;
    case Back:
        target_omega = 180.0f;
        break;
    case Right:
        target_omega = 270.0f;
        break;
    default:
        target_omega = 0.0f;
        break;
    }
    float angle = (omega - target_omega) * PI / 180.0f;

    Reset_Car_Status();

    PIDController_reset(outer_PID_f);
    PIDController_reset(outer_PID_Y_f);
    PIDController_reset(outer_PID_Z_f);

    while (1)
    {

        current_time = HAL_GetTick();
        last_receive = last_receive_raspi_time; // 读取最新时间戳

        switch (comms_state)
        {
        case STATE_NORMAL:
            if (current_time - last_receive > 300)
            {
                comms_state = STATE_TIMEOUT;

                last_control_time = 0; // 重置控制时间戳

                Car_Stop(1);
            }
            break;

        case STATE_TIMEOUT:
            if (current_time - last_receive < 300)
            {
                comms_state = STATE_NORMAL;
            }
            break;
        }

        if (running_time > TIMEOUT)
        {
            Car_Stop(0);
            break;
        }

        // ========================= [非阻塞数据有效性检测] =========================
        if (fabs(raspi_date->ref_x) < 0.1f || fabs(raspi_date->ref_y) < 0.1f)
        {
            continue; // 跳过无效数据.continue是跳过当前循环，继续执行下一个循环
        }

        /////只有在正常数据情况下才执行代码

        if (comms_state == STATE_NORMAL)
        {

            pixel_x = raspi_date->ref_x;
            pixel_y = raspi_date->ref_y; ////0.1228f

            target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[height]);
            target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[height]);

            if (fabs(target_position_x_relative) < 0.8f && fabs(target_position_y_relative) < 0.8f && fabs(target_omega - omega) < 2.0f)
            {
                // 使用静态计数器记录连续满足条件的次数
                stability_counter++;

                // 当连续满足精度要求的次数达到阈值后，才认为真正到达目标位置
                if (stability_counter >= stability_counter_threshold) // 假设连续10次检测都满足条件
                {

                    Car_Stop(0);
                    break;
                }
            }
            else
            {
                // 一旦不满足条件，立即重置计数器
                stability_counter = 0;
            }

            angle = (omega - target_omega) * PI / 180.0f;

            /// 需要注意，这里输入的应该是error，所以应该在那个输入的时候，就进行那啥

            Vx = PID_calc_micro(target_position_x_relative, outer_PID_f);

            // int16_t ref_Y = (int16_t)(packet_2->ref_y);
            //  Vy = (int16_t)(outer_PID_Y((float)ref_Y));
            // 这里暂时修改一下，要用omega的参数来实现校准

            Vy = PID_calc_Y_micro(target_position_y_relative, outer_PID_Y_f);

            /// 这个角度是负数的需要时刻铭记
            W = PID_calc_z_micro(target_omega, init_omega, outer_PID_Z_f);

            // 麦轮运动学解算
            float Car_H = 1.67f;
            float Car_W = 2.38f;
            float Vel1 = (K * (-W * (Car_H / 2 + Car_W / 2))) + K * Vx - K * Vy;
            float Vel2 = (K * (+W * (Car_H / 2 + Car_W / 2))) + K * Vx + K * Vy;
            float Vel3 = (K * (-W * (Car_H / 2 + Car_W / 2))) + K * Vx + K * Vy;
            float Vel4 = (K * (+W * (Car_H / 2 + Car_W / 2))) + K * Vx - K * Vy;

            uint8_t acc = 10;
            bool snF = 1;
            uint32_t delay_ms = 5;

            Emm_V5_Send_Four_Vel_Control(Vel1, Vel2, Vel3, Vel4, acc, snF, delay_ms);

            current_control_time = HAL_GetTick();

            if (last_control_time != 0)
            {
                running_time += (current_control_time - last_control_time);
            }
            last_control_time = current_control_time; // 更新控制时间戳

            /// 以下是调试用代码

            float txBuffer_35_x[10] = {0};
            // 计算出来的位移差值
            txBuffer_35_x[0] = target_position_x_relative;

            txBuffer_35_x[1] = target_position_y_relative;
            // 角度差值
            txBuffer_35_x[2] = target_omega - omega;

            txBuffer_35_x[3] = W;

            txBuffer_35_x[4] = Vx;

            txBuffer_35_x[5] = Vy;
            txBuffer_35_x[6] = raspi_date->ref_x;
            txBuffer_35_x[7] = raspi_date->ref_y;

            txBuffer_35_x[8] = raspi_date->vision_current_position_x[height];
            txBuffer_35_x[9] = raspi_date->vision_current_position_y[height];

            SendMultiFloat2Vofa(txBuffer_35_x, 10);
        }
        ///////////每次执行控制之后，再进行一个时间戳的更新
    }

    stability_counter = 0;

    PIDController_reset(outer_PID_f);
    PIDController_reset(outer_PID_Y_f);
    PIDController_reset(outer_PID_Z_f);

    HAL_Delay(50);

}

typedef enum
{
    OSC_POSITIVE, // 正向移动
    OSC_NEGATIVE, // 负向移动（2倍振幅）
    OSC_RETURN,   // 返回正向
    OSC_STOP,     // 方向切换停止缓冲
    OSC_COMPLETE  // 完成周期退出

} OscState;

void Car_Swap_and_Wait(float delta_x, float delta_y, float Vel_x, float Vel_y, bool continue_or_stop,
                       uint8_t task_id_send, uint8_t task_state_send,
                       uint8_t task_id_wait, uint8_t task_state_wait)
{
    // ----------------- 初始化静态变量（每次调用重置）-----------------
    static OscState current_state;
    static uint32_t stage_start_time;
    static bool exit_triggered;
    static OscState prev_state;

    current_state = OSC_POSITIVE;
    stage_start_time = HAL_GetTick();
    exit_triggered = false;
    prev_state = OSC_POSITIVE;
    // -------------------------------------------------------------

    // ----------------- 参数校验与方向确定 -----------------
    bool is_x_axis = (delta_x != 0); // 确定运动轴（仅单轴有效）
    float delta = is_x_axis ? fabsf(delta_x) : fabsf(delta_y);
    float vel = is_x_axis ? Vel_x : Vel_y;

    if (delta <= 0.001f || vel <= 0.001f)
    { // 避免除以零
        Error_Handler();
        return;
    }
    // ---------------------------------------------------

    while (1)
    {
        // 1. 退出条件检测（非阻塞）
        if (!exit_triggered && raspi_date->taskID == task_id_wait && raspi_date->taskstate == task_state_wait)
        {
            exit_triggered = true;
            if (!continue_or_stop)
            {
                Car_Stop(0); // 缓停
                return;      // 立即退出
            }
        }

        // 2. 状态机核心逻辑（分轴控制速度）
        switch (current_state)
        {
        case OSC_POSITIVE:
            // 正向速度（根据轴选择X/Y速度）

            car->target_map_Vx = is_x_axis ? vel : 0;
            car->target_map_Vy = is_x_axis ? 0 : vel;

            Publish_motor_speed();

            if (HAL_GetTick() - stage_start_time >= (uint32_t)(delta / vel * 1000))
            {
                current_state = OSC_STOP;
                stage_start_time = HAL_GetTick();
            }
            break;

        case OSC_NEGATIVE:
            // 负向速度（2倍振幅，速度绝对值相同）
            car->target_map_Vx = is_x_axis ? vel : 0;
            car->target_map_Vy = is_x_axis ? 0 : vel;

            Publish_motor_speed();

            if (HAL_GetTick() - stage_start_time >= (uint32_t)((2 * delta) / vel * 1000))
            {
                current_state = OSC_STOP;
                stage_start_time = HAL_GetTick();
            }
            break;

        case OSC_RETURN:
            // 返回正向速度
            car->target_map_Vx = is_x_axis ? vel : 0;
            car->target_map_Vy = is_x_axis ? 0 : vel;

            Publish_motor_speed();

            if (HAL_GetTick() - stage_start_time >= (uint32_t)(delta / vel * 1000))
            {
                current_state = OSC_STOP;
                stage_start_time = HAL_GetTick();
            }
            break;

        case OSC_STOP:
            // 方向切换缓冲（100ms缓停）
            Car_Stop(1);
            if (HAL_GetTick() - stage_start_time > 100)
            {
                // 状态切换逻辑
                if (prev_state == OSC_POSITIVE)
                {
                    current_state = OSC_NEGATIVE;
                }
                else if (prev_state == OSC_NEGATIVE)
                {
                    current_state = OSC_RETURN;
                }
                else if (prev_state == OSC_RETURN)
                {
                    current_state = (exit_triggered && continue_or_stop) ? OSC_COMPLETE : OSC_POSITIVE;
                }
                prev_state = current_state;
                stage_start_time = HAL_GetTick();
            }
            break;

        case OSC_COMPLETE:
            Car_Stop(0);
            return;
        }

        // 3. 发送任务状态（非阻塞）
        Send_Task_Status(task_id_send, task_state_send);

        HAL_Delay(10);
    }
}


void Car_Ring_Fast(uint32_t TIMEOUT)
{
    int16_t K = 1;              // 一个用于速度调整的参数
    uint32_t timeout = TIMEOUT; // 假如有外部的输入，那就用外部的
    // 还是不能这么写因为我们不能默认当前的角度就是我们想要的那个角度，所以要设置一个全局变量
    float init_omega = omega;
    static uint8_t stability_counter = 0;
    // stability_counter = 0;   //每次进入之前重置也是可以的
    uint8_t stability_counter_threshold = 5;

    uint32_t start_time = HAL_GetTick();

    float pixel_x = raspi_date->ref_x;
    float pixel_y = raspi_date->ref_y; ////0.1228f

    float target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[2]);
    float target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[2]);

    float target_omega = 0.0f;

    // Z轴角度控制
    float W = 0.0f;
    // float ref_Z = -(raspi_date->ref_z);//这个是之前测试出过问题的，值得重视

    // X轴位置控制
    float Vx = 0.0f;
    // Y轴位置控制
    float Vy = 0.0f;
    // 通过car_face来确定目标角度
    switch (car->car_face)
    {
        ////////这里进行了代码的修改
    case Forward:
        target_omega = 0.0f;
        break;
    case Left:
        target_omega = 90.0f;
        break;
    case Back:
        target_omega = 180.0f;
        break;
    case Right:
        target_omega = 270.0f;
        break;
    default:
        target_omega = 0.0f;
        break;
    }
    float angle = (omega - target_omega) * PI / 180.0f;

    Reset_Car_Status();

    PIDController_reset(outer_PID_f_Fast);
    PIDController_reset(outer_PID_Y_f_Fast);
    PIDController_reset(outer_PID_Z_f_Fast);

    while (1)
    {

        /////超时退出

        if (HAL_GetTick() - start_time > TIMEOUT)
        {
            Car_Stop(0);
            break;
        }

        // ========================= [非阻塞数据有效性检测] =========================
        if (fabs(raspi_date->ref_x) < 0.1f || fabs(raspi_date->ref_y) < 0.1f)
        {
            continue; // 跳过无效数据
        }

        /////运动的执行代码

        pixel_x = raspi_date->ref_x;
        pixel_y = raspi_date->ref_y; ////0.1228f

        target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[2]);
        target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[2]);

        if (fabs(target_position_x_relative) < 0.8f && fabs(target_position_y_relative) < 0.8f && fabs(target_omega - omega) < 2.0f)
        {
            // 使用静态计数器记录连续满足条件的次数
            stability_counter++;

            // 当连续满足精度要求的次数达到阈值后，才认为真正到达目标位置
            if (stability_counter >= stability_counter_threshold) // 假设连续10次检测都满足条件
            {

                Car_Stop(0);
                break;
            }
        }
        else
        {
            // 一旦不满足条件，立即重置计数器
            stability_counter = 0;
        }

        angle = (omega - target_omega) * PI / 180.0f;

        /// 需要注意，这里输入的应该是error，所以应该在那个输入的时候，就进行那啥

        Vx = PID_calc_micro_Fast(target_position_x_relative, outer_PID_f_Fast);

        // int16_t ref_Y = (int16_t)(packet_2->ref_y);
        //  Vy = (int16_t)(outer_PID_Y((float)ref_Y));
        // 这里暂时修改一下，要用omega的参数来实现校准

        Vy = PID_calc_Y_micro_Fast(target_position_y_relative, outer_PID_Y_f_Fast);

        /// 这个角度是负数的需要时刻铭记
        W = PID_calc_z_micro_Fast(target_omega, init_omega, outer_PID_Z_f_Fast);

       

        // 麦轮运动学解算
        float Car_H = 1.67f;
        float Car_W = 2.38f;
        float Vel1 = (K * (-W * (Car_H / 2 + Car_W / 2))) + K * Vx - K * Vy;
        float Vel2 = (K * (+W * (Car_H / 2 + Car_W / 2))) + K * Vx + K * Vy;
        float Vel3 = (K * (-W * (Car_H / 2 + Car_W / 2))) + K * Vx + K * Vy;
        float Vel4 = (K * (+W * (Car_H / 2 + Car_W / 2))) + K * Vx - K * Vy;

        uint8_t acc = 10;
        bool snF = 1;
        uint32_t delay_ms = 5;

        Emm_V5_Send_Four_Vel_Control(Vel1, Vel2, Vel3, Vel4, acc, snF, delay_ms);

        Vy = 0.0f;
        W = 0.0f;

        /// 以下是调试用代码

        float txBuffer_35_x[10] = {0};
        // 计算出来的位移差值
        txBuffer_35_x[0] = target_position_x_relative;

        txBuffer_35_x[1] = target_position_y_relative;
        // 角度差值
        txBuffer_35_x[2] = target_omega - omega;

        txBuffer_35_x[3] = W;

        txBuffer_35_x[4] = Vx;

        txBuffer_35_x[5] = Vy;
        txBuffer_35_x[6] = raspi_date->ref_x;
        txBuffer_35_x[7] = raspi_date->ref_y;

        txBuffer_35_x[8] = raspi_date->vision_current_position_x[2];
        txBuffer_35_x[9] = raspi_date->vision_current_position_y[2];

        SendMultiFloat2Vofa(txBuffer_35_x, 10);
    }

    stability_counter = 0;

    PIDController_reset(outer_PID_f_Fast);
    PIDController_reset(outer_PID_Y_f_Fast);
    PIDController_reset(outer_PID_Z_f_Fast);

    HAL_Delay(50);
}


void Car_Ring_Test(uint32_t TIMEOUT)
{
    Send_and_Wait_for_Cmd(0, 0, 1, 1);

    Car_Ring_Timeout(TIMEOUT);

    uint32_t current_time = HAL_GetTick();

    while (1)
    {
        if (HAL_GetTick() - current_time > 2000)
        {
            break;

            
        }
        Send_Task_Status(1, 1);

        HAL_Delay(10);
    }
    
    

    //Send_and_Wait_for_Cmd(, , , );

}

void Car_Ring_Timeout_Fast( uint32_t TIMEOUT , enum Height height )
{
    
    int16_t K = 1;              // 一个用于速度调整的参数
    uint32_t timeout = TIMEOUT; // 假如有外部的输入，那就用外部的
    // 还是不能这么写因为我们不能默认当前的角度就是我们想要的那个角度，所以要设置一个全局变量
    float init_omega = omega;
    static uint8_t stability_counter = 0;
    // stability_counter = 0;   //每次进入之前重置也是可以的
    uint8_t stability_counter_threshold = 5;
    stability_counter = 0; // 每次进入函数时重置

    uint32_t running_time = 0;
    volatile CommsState comms_state = STATE_NORMAL;  // 初始状态为正常
    uint32_t last_receive = last_receive_raspi_time; // 读取最新时间戳

    uint32_t current_time = HAL_GetTick();

    uint32_t last_control_time = 0; // 静态变量保持状态
    uint32_t current_control_time = 0;

    float pixel_x = raspi_date->ref_x;
    float pixel_y = raspi_date->ref_y; ////0.1228f

    float target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[height]);
    float target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[height]);

    float target_omega = 0.0f;

    // Z轴角度控制
    float W = 0.0f;
    // float ref_Z = -(raspi_date->ref_z);//这个是之前测试出过问题的，值得重视

    // X轴位置控制
    float Vx = 0.0f;
    // Y轴位置控制
    float Vy = 0.0f;

    static float last_ref_x = 0 ;
    static float last_ref_y = 0 ;

    // 通过car_face来确定目标角度
    switch (car->car_face)
    {
        ////////这里进行了代码的修改
    case Forward:
        target_omega = 0.0f;
        break;
    case Left:
        target_omega = 90.0f;
        break;
    case Back:
        target_omega = 180.0f;
        break;
    case Right:
        target_omega = 270.0f;
        break;
    default:
        target_omega = 0.0f;
        break;
    }
    float angle = (omega - target_omega) * PI / 180.0f;

    Reset_Car_Status();

    PIDController_reset(outer_PID_f_Fast);
    PIDController_reset(outer_PID_Y_f_Fast);
    PIDController_reset(outer_PID_Z_f_Fast);

    //在进入函数之前，进行pid的调试尝试，这个pid的参数，可能需要根据
    static const float pixel_to_x[8] = {0.4f, 0.83f, 0.3f, 0.4f, 0.4f, 0.83f, 0.3f, 0.4f};
    static const float pixel_to_y[8] = {0.4f, 0.81f, 0.3f, 0.4f, 0.4f, 0.81f, 0.3f, 0.4f}; 

    float Vx_high_to_low = pixel_to_x[height] / pixel_to_x[2];

    float Vy_high_to_low = pixel_to_y[height] / pixel_to_y[2];

    while (1)
    {

        current_time = HAL_GetTick();
        last_receive = last_receive_raspi_time; // 读取最新时间戳

        switch (comms_state)
        {
        case STATE_NORMAL:
            if (current_time - last_receive > 300)
            {
                comms_state = STATE_TIMEOUT;

                last_control_time = 0; // 重置控制时间戳

                Car_Stop(1);
            }
            break;

        case STATE_TIMEOUT:
            if (current_time - last_receive <= 300)
            {
                comms_state = STATE_NORMAL;
            }
            break;
        }

        if (running_time > TIMEOUT)
        {
            Car_Stop(0);
            break;
        }

        // ========================= [非阻塞数据有效性检测] =========================
        if (fabs(raspi_date->ref_x) < 0.1f || fabs(raspi_date->ref_y) < 0.1f)
        {
            continue; // 跳过无效数据.continue是跳过当前循环，继续执行下一个循环
        }

        /////只有在正常数据情况下才执行代码，我这里的建议是需要根据pixel_to_x的参数对输入进行归一化
        //也就是先测试出最底层的pixel_to_x的参数，然后别的参数就依据此进行放大，这样就不错了，这样pid参数也不需要调节
        

        if (comms_state == STATE_NORMAL)
        {

            pixel_x = raspi_date->ref_x;
            pixel_y = raspi_date->ref_y; ////0.1228f

            target_position_x_relative = (pixel_x - raspi_date->vision_current_position_x[height]) * Vx_high_to_low ;
            target_position_y_relative = (pixel_y - raspi_date->vision_current_position_y[height]) * Vy_high_to_low ;

            


            if (fabs(target_position_x_relative) < 0.5f && fabs(target_position_y_relative) < 0.5f && fabs(target_omega - omega) < 2.0f)
            {
                // 使用静态计数器记录连续满足条件的次数
                stability_counter++;

                // 当连续满足精度要求的次数达到阈值后，才认为真正到达目标位置
                if (stability_counter >= stability_counter_threshold) // 假设连续10次检测都满足条件
                {

                    Car_Stop(0);
                    break;
                }
            }
            else
            {
                // 一旦不满足条件，立即重置计数器
                stability_counter = 0;
            }

            angle = (omega - target_omega) * PI / 180.0f;

            /// 需要注意，这里输入的应该是error，所以应该在那个输入的时候，就进行那啥

            Vx = PID_calc_micro_Fast(target_position_x_relative, outer_PID_f_Fast);

            // int16_t ref_Y = (int16_t)(packet_2->ref_y);
            //  Vy = (int16_t)(outer_PID_Y((float)ref_Y));
            // 这里暂时修改一下，要用omega的参数来实现校准

            Vy = PID_calc_Y_micro_Fast(target_position_y_relative, outer_PID_Y_f_Fast);

            /// 这个角度是负数的需要时刻铭记
            W = PID_calc_z_micro_Fast(target_omega, init_omega, outer_PID_Z_f_Fast);

            //////在控制之前实现对数据的检测
            if(fabs(last_ref_x - pixel_x) > 50.0f || fabs(last_ref_y - pixel_y) > 50.0f)
            {
                Vx = 0.0f;
                Vy = 0.0f;
                W = 0.0f;
            }

            // 麦轮运动学解算
            float Car_H = 1.67f;
            float Car_W = 2.38f;
            float Vel1 = (K * (-W * (Car_H / 2 + Car_W / 2))) + K * Vx - K * Vy;
            float Vel2 = (K * (+W * (Car_H / 2 + Car_W / 2))) + K * Vx + K * Vy;
            float Vel3 = (K * (-W * (Car_H / 2 + Car_W / 2))) + K * Vx + K * Vy;
            float Vel4 = (K * (+W * (Car_H / 2 + Car_W / 2))) + K * Vx - K * Vy;

            uint8_t acc = 10;
            bool snF = 1;
            uint32_t delay_ms = 5;

            Emm_V5_Send_Four_Vel_Control(Vel1, Vel2, Vel3, Vel4, acc, snF, delay_ms);

            current_control_time = HAL_GetTick();

            if (last_control_time != 0)
            {
                running_time += (current_control_time - last_control_time);
            }
            last_control_time = current_control_time; // 更新控制时间戳

            last_ref_x = pixel_x ;

            last_ref_y = pixel_y ;

            /// 以下是调试用代码

            float txBuffer_35_x[10] = {0};
            // 计算出来的位移差值
            txBuffer_35_x[0] = target_position_x_relative;

            txBuffer_35_x[1] = target_position_y_relative;
            // 角度差值
            txBuffer_35_x[2] = target_omega - omega;

            txBuffer_35_x[3] = W;

            txBuffer_35_x[4] = Vx;

            txBuffer_35_x[5] = Vy;
            txBuffer_35_x[6] = raspi_date->ref_x;
            txBuffer_35_x[7] = raspi_date->ref_y;

            txBuffer_35_x[8] = raspi_date->vision_current_position_x[height];
            txBuffer_35_x[9] = raspi_date->vision_current_position_y[height];

            SendMultiFloat2Vofa(txBuffer_35_x, 10);


        }
       
    }

    stability_counter = 0;

    last_ref_x = 0 ;
    last_ref_y = 0 ;

    PIDController_reset(outer_PID_f_Fast);
    PIDController_reset(outer_PID_Y_f_Fast);
    PIDController_reset(outer_PID_Z_f_Fast);

    HAL_Delay(50);


}





////////////////这个的宏定义放在最上面了

/**
 * @brief 小车位姿闭环控制函数
 * @param x 目标世界坐标x，单位mm
 * @param y 目标世界坐标y，单位mm
 * @param angle 目标方向角，单位度(deg)
 */
void CarMoveTo(float x, float y, float angle, uint32_t TIMEOUT)
{
    // 不再需要单位转换，因为直接使用mm为单位
    float targetX = x;
    float targetY = y;
    float targetA = angle; // 单位是度

    // 这个是最基准的用于计算的pv
    float PVX1_Base = 0.0f;
    float PVX2_Base = 0.0f;
    float PVY1_Base = 0.0f;
    float PVY2_Base = 0.0f;
    /// PVX1_Base = 0.0f;
    if (abs(targetX) < 310.0f)
    {
        PVX1_Base = 10.0f;
    }
    else if (abs(targetX) > 310.0f && abs(targetX) < 495.0f)
    {
        PVX1_Base = 10.0f - 0.005f * (abs(targetX) - 300.0f);
    }
    else
    {
        PVX1_Base = 9.0f;
    }
    ////PVX2_Base = 0.0f;
    if (abs(targetX) < 310.0f)
    {
        PVX2_Base = 0.15f;
    }
    else if (abs(targetX) > 310.0f && abs(targetX) < 495.0f)
    {
        PVX2_Base = 0.15f - 0.00025f * (abs(targetX) - 300.0f);
    }
    else if (abs(targetX) > 495.0f && abs(targetX) < 795.0f)
    {
        PVX2_Base = 0.10f - 0.0001f * (abs(targetX) - 500.0f);
    }
    else if (abs(targetX) > 795.0f && abs(targetX) < 1695.0f)
    {
        PVX2_Base = 0.07f - 0.0000222f * (abs(targetX) - 800.0f);
    }
    else if (abs(targetX) > 1695.0f && abs(targetX) < 1800.0f)
    {
        PVX2_Base = 0.05f;
    }
    else
    {
        PVX2_Base = 0.04f;
    }

    ////PVY1_Base = 0.0f;
    if (abs(targetY) < 310.0f)
    {
        PVY1_Base = 5.0f;
    }
    else if (abs(targetY) > 310.0f && abs(targetY) < 495.0f)
    {
        PVY1_Base = 5.0f - 0.0025f * (abs(targetY) - 300.0f);
    }
    else
    {
        PVY1_Base = 4.5f;
    }
    /// PVY2_Base = 0.0f;
    if (abs(targetY) < 310.0f)
    {
        PVY2_Base = 0.15f;
    }
    else if (abs(targetY) >= 310.0f && abs(targetY) < 495.0f)
    {
        PVY2_Base = 0.15f - 0.00025f * (abs(targetY) - 300.0f);
    }
    else if (abs(targetY) >= 495.0f && abs(targetY) < 795.0f)
    {
        PVY2_Base = 0.10f - 0.0001f * (abs(targetY) - 500.0f);
    }
    else if (abs(targetY) >= 795.0f && abs(targetY) < 1695.0f)
    {
        PVY2_Base = 0.07f - 0.0000222f * (abs(targetY) - 800.0f);
    }
    else if (abs(targetY) >= 1695.0f && abs(targetY) < 1795.0f)
    {
        PVY2_Base = 0.04f;
    }
    else
    {
        PVY2_Base = 0.034f; // 输入不在范围内
    }

    float PVX1 = 0.0f;
    float PVX2 = 0.0f;
    float PVY1 = 0.0f;
    float PVY2 = 0.0f;

    // 这里进行一个比较简单的协同，就根据比例来调节起步速度

    // if (fabs(targetX) > 5.0f && fabs(targetY) > 5.0f)
    // {
    //     if (fabs(targetX) < fabs(targetY))
    //     {
    //         PVX1_Base *= 2 * fabs(targetX) / fabs(targetY) + fabs(targetX);
    //     }
    //     else
    //     {
    //         PVY1_Base *= 2 * fabs(targetY) / fabs(targetX) + fabs(targetY);
    //     }
    // }

    // 然后每次根据car_status.car_face来判断
    if (car->car_face == Forward || car->car_face == Back)
    {
        PVX1 = PVX1_Base;
        PVX2 = PVX2_Base;
        PVY1 = PVY1_Base;
        PVY2 = PVY2_Base;
    }
    else
    {
        PVX1 = PVY1_Base;
        PVX2 = PVY2_Base;
        PVY1 = PVX1_Base;
        PVY2 = PVX2_Base;
    }

    ////这里是进行测试的代码，或许通过这个来调节

    // 定义精度阈值
    float eX = POS_PRECISION_X * 1.0f; // mm，保持原来的阈值大小
    float eY = POS_PRECISION_Y * 1.0f; // mm，保持原来的阈值大小
    float eA = ANG_PRECISION * 1.0f;   // 转换为度的阈值
    // 连续满足条件的计数器
    uint16_t stability_counter = 0;
    uint16_t stability_threshold = 3;
    // 重置PID控制器
    PIDController_reset(inner_PID_X);
    PIDController_reset(inner_PID_Y);
    PIDController_reset(outer_PID_omega_f);
    Reset_Car_Status();
    Update_Car_Status(5);


    
    uint32_t startTime = HAL_GetTick();
    ////////////////先给较大的超时，看看调节的怎么样
    uint32_t timeout = TIMEOUT; // 10秒超时
    float stop = 0.0f;
    float init_omega = omega;

    while (1)
    {
        // 获取当前位置数据
        ///
        Update_Car_Status(5);

        // 计算误差
        float lossX = targetX - car->current_map_position_x;
        float lossY = targetY - car->current_map_position_y;
        float lossA = targetA - omega; // 直接使用度作为单位

        // 调试信息
        float txBuffer[6] = {
            lossX, lossY, lossA, car->target_map_Vx, car->target_map_Vy, omega};
        // car->current_map_position_x, car->current_map_position_y, omega};
        // SendMultiFloat2Vofa(txBuffer, 6);

        // 检查是否达到目标精度
        if ((fabs(lossX) < eX) && (fabs(lossY) < eY) && (fabs(lossA) < eA))
        {
            // 使用静态计数器记录连续满足条件的次数
            stability_counter++;

            // 当连续满足精度要求的次数达到阈值后，才认为真正到达目标位置
            if (stability_counter >= stability_threshold) // 假设连续10次检测都满足条件
            {

                // SendMultiFloat2Vofa(txBuffer, 6);

                // 停止小车
                Car_Stop(1);

                break;

                // uint8_t acc =150 ;
                // bool snF = true ;
                // uint32_t delay_ms = 5 ;
                // Emm_V5_Vel_Control_Self(0,0.0,acc,snF);

                // car->target_map_Vx = 0;
                // car->target_map_Vy = 0;
                // Publish_motor_speed();
                // while (1)
                // {
                //     //Update_Car_Status(10);

                //     SendMultiFloat2Vofa(txBuffer, 6);
                //     HAL_Delay(10);
                //     // if (HAL_GetTick() - startTime > timeout)
                //     // {

                //          break;
                //     // }
                // }

                // 重置计数器供下次使用

                break;
            }
        }
        else
        {
            // 一旦不满足条件，立即重置计数器
            stability_counter = 0;
        }

        // 检查是否超时
        if (HAL_GetTick() - startTime > timeout)
        {

            Car_Stop(1);
            break;
        }

        // 计算X方向控制速度
        float vx = 0;
        if (fabs(lossX) > eX)
        {
            if (targetX != 0)
            {
                float sign = (lossX > 0) ? 1.0f : -1.0f;
                float dynamic_factor = 1.0f - 0.6f * (fabs(lossX) / fabs(targetX)); // 动态因子
                vx = sign * PVX1 * fabs(targetX) * powf(PVX2 / PVX1, (targetX - sign * fabs(lossX)) / targetX) * dynamic_factor;
            }
            else
            {
                vx = PVX1 * lossX * fabs(lossY) / fabs(targetY);
            }
        }

        // 计算Y方向控制速度
        float vy = 0;
        if (fabs(lossY) > eY)
        {
            if (targetY != 0)
            {
                float sign = (lossY > 0) ? 1.0f : -1.0f;
                float dynamic_factor = 1.0f - 0.6f * (fabs(lossY) / fabs(targetY)); // 动态因子
                vy = sign * PVY1 * fabs(targetY) * powf(PVY2 / PVY1, (targetY - sign * fabs(lossY)) / targetY);
            }
            else
            {
                vy = PVY1 * lossY * fabs(lossX) / fabs(targetX);
            }
        }

        // 计算角速度控制
        float vSpin = 0;
        if (fabs(lossA) > eA)
        {
            // // 角度计算时不需要转换单位
            // if (targetA != 0)
            // {
            //     float sign = (targetA*lossA > 0) ? 1.0f : -1.0f;
            //     vSpin = sign * PVA1 * fabs(targetA) *
            //            powf(PVA2/PVA1, (fabs(targetA)-fabs(lossA))/fabs(targetA));
            // }
            // else
            // {
            //     vSpin = PVA1 * lossA;
            // }
            vSpin = PID_calc_z(targetA, init_omega, outer_PID_omega_f);
        }

        // 转换到车体坐标系
        float angle_rad = omega * PI / 180.0f; // 转换为弧度用于三角函数计算
        car->target_map_Vx = vx;
        car->target_map_Vy = vy;
        car->target_angle_speed = vSpin; // 已经是度/秒，不需要转换

        // 这个是速度分配·

        float target_map_all_speed = sqrt(car->target_map_Vx * car->target_map_Vx + car->target_map_Vy * car->target_map_Vy);
        car->target_map_Vx *= fabs(car->target_map_Vx) / target_map_all_speed;
        car->target_map_Vy *= fabs(car->target_map_Vy) / target_map_all_speed;
        ////500mm限制幅度太小了，还是不太好
        // 限幅
        car->target_map_Vx = fmaxf(-1500.0f, fminf(1500.0f, car->target_map_Vx));
        car->target_map_Vy = fmaxf(-1500.0f, fminf(1500.0f, car->target_map_Vy));
        car->target_angle_speed = fmaxf(-70.0f, fminf(70.0f, car->target_angle_speed));

        // 调试信息
        float txBuffer_1[3] = {
            car->target_map_Vx, car->target_map_Vy, omega};
        SendMultiFloat2Vofa(txBuffer_1, 3);

        // 发布控制命令
        Publish_motor_speed();

        // 控制周期
        HAL_Delay(50);
    }
    // 用于检测到位的计数需要清零
    stability_counter = 0;

    // 重置PID控制器
    PIDController_reset(inner_PID_X);
    PIDController_reset(inner_PID_Y);
    PIDController_reset(outer_PID_omega_f);
    HAL_Delay(50);
}

/**
 * @brief 实现小车在指定方向上的平移运动
 * @param dirc：平移方向，机器人坐标系，0=X方向，1=Y方向
 * @param v：平移速度，单位mm/s
 * @param distance：平移距离，单位mm
 * @param eA：角度控制精度，单位度
 * @param pA：角度控制比例系数
 */
void CarTrans(int dirc, float v, float distance, float eA, float pA)
{
    PIDController_reset(inner_PID_X);
    PIDController_reset(inner_PID_Y);
    PIDController_reset(outer_PID_omega_f);
    Reset_Car_Status();
    // 获取当前位置数据
    Update_Car_Status(5);
    // 记录初始角度和初始位置
    float initial_angle = omega;
    float initial_position_x = car->current_map_position_x;
    float initial_position_y = car->current_map_position_y;

    // 计算初始位置和目标位置差
    float travel_distance = 0.0f;

    // 初始化稳定计数器
    uint16_t stability_counter = 0;
    uint16_t stability_threshold = 3;

    while (1)
    {
        // 更新车辆状态
        Update_Car_Status(5);

        // 计算已行驶距离
        if (dirc == 0)
        { // X方向
            travel_distance = fabs(car->current_map_position_x - initial_position_x);
        }
        else
        { // Y方向
            travel_distance = fabs(car->current_map_position_y - initial_position_y);
        }

        // 检查是否到达目标距离
        if (travel_distance >= distance)
        {
            stability_counter++;
            if (stability_counter >= stability_threshold)
            {
                // 停车

                Car_Stop(1);
                break;
            }
        }
        else
        {
            stability_counter = 0;
        }

        // 计算角度误差
        float angle_error = initial_angle - omega;

        // 角度误差修正（处理角度跨越±180度的情况）
        if (angle_error > 180.0f)
            angle_error -= 360.0f;
        if (angle_error < -180.0f)
            angle_error += 360.0f;

        // 调试信息
        float txBuffer[4] = {
            travel_distance, distance, angle_error, omega};
        SendMultiFloat2Vofa(txBuffer, 4);

        // 计算速度分量
        float vx = 0.0f, vy = 0.0f;
        if (dirc == 0)
        { // X方向
            vx = v;
            vy = 0;
        }
        else
        { // Y方向
            vx = 0;
            vy = v;
        }

        // 如果角度偏差大于阈值，启用角度控制
        float target_angle_speed = 0.0f;
        if (fabs(angle_error) > eA)
        {
            target_angle_speed = pA * angle_error;
        }

        // 转换到车体坐标系
        float angle_rad = omega * PI / 180.0f;
        car->target_map_Vx = vx * cosf(angle_rad) + vy * sinf(angle_rad);
        car->target_map_Vy = -vx * sinf(angle_rad) + vy * cosf(angle_rad);
        car->target_angle_speed = target_angle_speed;

        // 限幅
        car->target_map_Vx = fmaxf(-500.0f, fminf(500.0f, car->target_map_Vx));
        car->target_map_Vy = fmaxf(-500.0f, fminf(500.0f, car->target_map_Vy));
        car->target_angle_speed = fmaxf(-70.0f, fminf(70.0f, car->target_angle_speed));

        // 发布控制命令
        Publish_motor_speed();

        // 适当延时
        HAL_Delay(5);
    }

    // 重置PID控制器
    PIDController_reset(inner_PID_X);
    PIDController_reset(inner_PID_Y);
    PIDController_reset(outer_PID_omega_f);
}

/**
 * @brief 实现小车的侧向漂移运动
 * @param r：漂移半径，单位mm
 * @param angle：漂移角度，单位度
 * @param v：漂移速度，左为正，单位mm/s
 */
void CarDrift(float r, float angle, float v)
{
    // 重置PID控制器和车辆状态
    PIDController_reset(inner_PID_X);
    PIDController_reset(inner_PID_Y);
    PIDController_reset(outer_PID_omega_f);
    Reset_Car_Status();
    HAL_Delay(60);
    // 记录初始角度
    float initial_angle = omega;

    // 计算角速度
    float angular_velocity = v / r / pi * 180.0f;

    // 设置车辆速度
    car->target_car_Vx = 0; // 漂移时向后移动
    car->target_car_Vy = v; // 无侧向初始速度
    car->target_angle_speed = angular_velocity;
    // 限幅
    car->target_car_Vx = fmaxf(-500.0f, fminf(500.0f, car->target_car_Vx));
    car->target_car_Vy = fmaxf(-500.0f, fminf(500.0f, car->target_car_Vy));
    car->target_angle_speed = fmaxf(-70.0f, fminf(70.0f, car->target_angle_speed));
    // 根据car_speed计算motor_speed
    float Car_H = 1.670f; // 前后轮心距
    float Car_W = 2.380f; // 左右轮心距
    float Kx = 1;
    float Ky = 1;
    float Kz = 1.74 * 1; // 1.74将target和current单位统一

    car->target_Vel1 = Kx * car->target_car_Vx - Ky * car->target_car_Vy + Kz * (-car->target_angle_speed * (Car_H / 2 + Car_W / 2));
    car->target_Vel2 = Kx * car->target_car_Vx + Ky * car->target_car_Vy + Kz * (+car->target_angle_speed * (Car_H / 2 + Car_W / 2));
    car->target_Vel3 = Kx * car->target_car_Vx + Ky * car->target_car_Vy + Kz * (-car->target_angle_speed * (Car_H / 2 + Car_W / 2));
    car->target_Vel4 = Kx * car->target_car_Vx - Ky * car->target_car_Vy + Kz * (+car->target_angle_speed * (Car_H / 2 + Car_W / 2));

    uint8_t acc = 150;
    bool snF = true;
    uint32_t delay_ms = 10;
    Emm_V5_Send_Four_Vel_Control(car->target_Vel1, car->target_Vel2, car->target_Vel3, car->target_Vel4, acc, snF, delay_ms);

    // 主循环控制漂移过程
    while (1)
    {

        // 判断是否达到目标漂移角度
        if ((v * angle > 0 && (omega - initial_angle) > fabs(angle)) ||
            (v * angle < 0 && (omega - initial_angle) < -fabs(angle)))
        {
            // 达到目标角度，停止漂移

            Car_Stop(0);
            break;
        }

        // 适当延时
        HAL_Delay(5);
    }

    // 重置PID控制器
    PIDController_reset(inner_PID_X);
    PIDController_reset(inner_PID_Y);
    PIDController_reset(outer_PID_omega_f);
    HAL_Delay(200);
}
// 辅助函数：符号函数
float sign(float val)
{
    if (val > 0)
        return 1.0f;
    if (val < 0)
        return -1.0f;
    return 0.0f;
}

