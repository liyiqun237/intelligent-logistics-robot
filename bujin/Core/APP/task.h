#ifndef TASK_H
#define TASK_H
#include "car.h"
#include "math.h"

//**********************************结构体声明*******************************************//
typedef struct
{
  float ref_x;     // 
  float ref_y;     // 
  int32_t ref_z;      // （本来传过来的应该是uint16_t 转换之后我自己留一个int32_t）（-26946，36108）
  uint8_t taskID;     // 
  uint8_t taskstate;  //
  uint8_t order1[3]; //
  uint8_t order2[3];
  float vision_current_position_x[8] ;
  float vision_current_position_y[8] ;
  /////不书写枚举变量了，
} Raspi_Date;
// 颜色序号（假如颜色的序号改变了，就需要在这里修改枚举变量）
enum Color
{
  RED = 1,
  GREEN = 2,
  BLUE = 3,  
}; 
// 任务序号

typedef struct {
    float delta_position_x;
    float delta_position_y;
    // 可扩展更多维度
} Delta_Position_XY;
typedef enum {
    TASK_IDLE = 0,
    TASK_1,
    TASK_2,
    TASK_3,
    TASK_4,
    TASK_5,
    TASK_6,
    TASK_7,
    TASK_8,
    TASK_9,
    TASK_10,
    TASK_11,
    TASK_12,
    TASK_13,
    TASK_MAX
} TaskState;
// 任务完成状态
enum 
{
  task_unfinished = 0,
  task_finished = 1,
}; 



extern Raspi_Date *raspi_date;
extern volatile uint32_t last_receive_raspi_time;

 Delta_Position_XY Coulor_feedback(enum Color color);


//**********************************全局变量声明*******************************************//

void Send_and_Wait_for_Cmd(uint8_t task_id_send, uint8_t task_state_send, uint8_t task_id_wait, uint8_t task_state_wait);
void Send_Task_Status(uint8_t task_id_send, uint8_t task_state_send);
void Wait_for_Task_Status(uint8_t task_id_wait, uint8_t task_state_wait);
/// @brief 1为从启停区    2为精确从精确调整到板子
/// @param stop_or_precise /////////////
//////这个仅仅包含跑图的逻辑，其实就是把位置移动的东西进行了封装
//////一个套用原来的函数进行颜色之间的位移的函数

///单独颜色之间的移动，movetocolor能够实现颜色逻辑之间的运动
///使用1，那就是rough的调整，不使用，那就是正常的位移函数

/// @brief 这个数值给1，那就是可以rough，假设给0，那就是精确的调整，也就是颜色之间的运动，那就要给1
/// @param  
/// @param delta_x 
/// @param delta_y 
/// @param TIMEOUT 
/// @param is_rough 
void Reset_to_Color_and_Move(enum Color color , float delta_x, float delta_y, uint32_t TIMEOUT , bool is_rough);



void Stop_to_Order(bool is_swap);

/// @brief 初赛实际会被调用和执行的函数
/// @param Rounds 
void Order_or_Precise_to_Plate(uint8_t Rounds);

void Plate_to_Rough(uint8_t Rounds);

void Rough_to_Precise(uint8_t Rounds);

void Test_Run_and_Blog(void);

void Test_Run_Only(void);





//////////////////决赛执行的函数

void Stop_to_Rough(uint8_t Rounds);

void Rough_to_Plate(uint8_t Rounds);

void Plate_to_Precise(uint8_t Rounds);

void Precise_to_Plate(uint8_t Rounds);

void Precise_to_Stop(uint8_t Rounds);

///决赛测试跑图和校准的函数

void run_function(void);



#endif