#include "Emm_V5.h"

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

/**
 * @brief    将当前位置清零
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0x0A; // 功能码
  cmd[2] = 0x6D; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令
  usart_SendCmd(cmd, 4);
}

/**
 * @brief    解除堵转保护
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0x0E; // 功能码
  cmd[2] = 0x52; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令
  usart_SendCmd(cmd, 4);
}

/**
 * @brief    读取系统参数
 * @param    addr  ：电机地址
 * @param    s     ：系统参数类型
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[i] = addr;
  ++i; // 地址

  switch (s) // 功能码
  {
  case S_VER:
    cmd[i] = 0x1F;
    ++i;
    break;
  case S_RL:
    cmd[i] = 0x20;
    ++i;
    break;
  case S_PID:
    cmd[i] = 0x21;
    ++i;
    break;
  case S_VBUS:
    cmd[i] = 0x24;
    ++i;
    break;
  case S_CPHA:
    cmd[i] = 0x27;
    ++i;
    break;
  case S_ENCL:
    cmd[i] = 0x31;
    ++i;
    break;
  case S_TPOS:
    cmd[i] = 0x33;
    ++i;
    break;
  case S_VEL:
    cmd[i] = 0x35;
    ++i;
    break;
  case S_CPOS:
    cmd[i] = 0x36;
    ++i;
    break;
  case S_PERR:
    cmd[i] = 0x37;
    ++i;
    break;
  case S_FLAG:
    cmd[i] = 0x3A;
    ++i;
    break;
  case S_ORG:
    cmd[i] = 0x3B;
    ++i;
    break;
  case S_Conf:
    cmd[i] = 0x42;
    ++i;
    cmd[i] = 0x6C;
    ++i;
    break;
  case S_State:
    cmd[i] = 0x43;
    ++i;
    cmd[i] = 0x7A;
    ++i;
    break;
  default:
    break;
  }

  cmd[i] = 0x6B;
  ++i; // 校验字节

  // 发送命令
  usart_SendCmd(cmd, i);
}

/**
 * @brief    修改开环/闭环控制模式
 * @param    addr     ：电机地址
 * @param    svF      ：是否存储标志，false为不存储，true为存储
 * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr;      // 地址
  cmd[1] = 0x46;      // 功能码
  cmd[2] = 0x69;      // 辅助码
  cmd[3] = svF;       // 是否存储标志，false为不存储，true为存储
  cmd[4] = ctrl_mode; // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  cmd[5] = 0x6B;      // 校验字节

  // 发送命令
  usart_SendCmd(cmd, 6);
}

/**
 * @brief    使能信号控制
 * @param    addr  ：电机地址
 * @param    state ：使能状态     ，true为使能电机，false为关闭电机
 * @param    snF   ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr;           // 地址
  cmd[1] = 0xF3;           // 功能码
  cmd[2] = 0xAB;           // 辅助码
  cmd[3] = (uint8_t)state; // 使能状态
  cmd[4] = snF;            // 多机同步运动标志
  cmd[5] = 0x6B;           // 校验字节

  // 发送命令
  usart_SendCmd(cmd, 6);
}

/**
 * @brief    速度模式
 * @param    addr：电机地址
 * @param    dir ：方向       ，0为CW，其余值为CCW
 * @param    vel ：速度       ，范围0 - 5000RPM
 * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr;                // 地址
  cmd[1] = 0xF6;                // 功能码
  cmd[2] = dir;                 // 方向
  cmd[3] = (uint8_t)(vel >> 8); // 速度(RPM)高8位字节
  cmd[4] = (uint8_t)(vel >> 0); // 速度(RPM)低8位字节
  cmd[5] = acc;                 // 加速度，注意：0是直接启动
  cmd[6] = snF;                 // 多机同步运动标志
  cmd[7] = 0x6B;                // 校验字节

  // HAL_UART_Transmit(&huart3, cmd, 8 ,  0xFFFF);
  //  发送命令
  // usart_SendCmd(cmd, 8);
  HAL_UART_Transmit(&huart1, cmd, 8 , 0xFFFF);
}

/**
 * @brief    位置模式
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
 * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF) // clk是脉冲数目
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr;                 // 地址
  cmd[1] = 0xFD;                 // 功能码
  cmd[2] = dir;                  // 方向 （所以说这个是不能输入复数的，假如要后退，就要换方向）（这个方向，0是正向，那么一就是反向咯）
  cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
  cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
  cmd[5] = acc;                  // 加速度，注意：0是直接启动
  cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
  cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
  cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
  cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )
  cmd[10] = raF;                 // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] = snF;                 // 多机同步运动标志，false为不启用，true为启用
  cmd[12] = 0x6B;                // 校验字节（）这个多机同步是啥意思呢，感觉应该是需要使用的

  // 发送命令
  // HAL_UART_Transmit(&huart3, cmd, 13 , 0xFFFF);
  // usart_SendCmd(cmd, 13);
  HAL_UART_Transmit(&huart1, cmd, 13 ,0xFFFF);
  // HAL_UART_Transmit(&huart3, cmd, 13 ,  0xFFFF);
}

/**
 * @brief    立即停止（所有控制模式都通用）
 * @param    addr  ：电机地址
 * @param    snF   ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0xFE; // 功能码
  cmd[2] = 0x98; // 辅助码
  cmd[3] = snF;  // 多机同步运动标志
  cmd[4] = 0x6B; // 校验字节

  // 发送命令
  HAL_UART_Transmit(&huart1, cmd, 5 ,0xFFFF);
}

/**
 * @brief    多机同步运动
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Synchronous_motion(uint8_t addr) // 进行多机同步指令的发送。
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0xFF; // 功能码
  cmd[2] = 0x66; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令
  HAL_UART_Transmit(&huart1, cmd, 4 ,0xFFFF);
}

void Emm_V5_Pos_Control_Self(uint8_t addr, uint16_t vel, uint8_t acc, int32_t clk, bool raF, bool snF) // 这中间原来一个dir的参数在函数自己内部计算
{
  uint8_t dir = 0;
  uint32_t clk2 = 0;          // 因为格式问题进行的转换
  if (addr == 1 || addr == 3) // 这里先根据位号确定正方向
  {
    dir = 0;
  }
  if (addr == 2 || addr == 4)
  {
    dir = 1;
  }
  // 之后在根据输入数值的正负  决定要正方向转还是反方向转。
  if (clk >= 0)
  {
    clk2 = (uint32_t)clk;
  }
  if (clk < 0) // 你这里没有修改clk的数值啊，还是没有实现反向。
  {
    if (dir == 0) // 修改运动方向,因为要向反方向运动。
    {
      dir = 1;
    }
    else if (dir == 1)
    {
      dir = 0;
    }
    clk2 = (uint32_t)(-clk);
  }
   int32_t xifen = 4;//////这个4是细分数目
  clk2 = xifen * clk2;
  // 要好好理解这里的比例，这个最后换算出来的clk3的单位是mm
  // 假如我们默认在那个电机的控制中，这个mm和那个速度的mm是相同的
  // 数据的依据是，假设我们是16细分，那么给3200个脉冲，就能转一圈，
  uint32_t clk3 = (uint32_t)(clk2 * 3200 / (pi * diameter));
  Emm_V5_Pos_Control(addr, dir, vel, acc, clk3, raF, snF); // 多级同步标志位设置为1，
}

void Emm_V5_Vel_Control_Self(uint8_t addr, float  vel, uint8_t acc, bool snF) // 同样dir由内部地址和clk决定
{
  uint8_t dir = 0;
  float  vel2 = 0.0f ; // 因为格式问题进行的转换

  if (addr == 1 || addr == 3) // 这里先根据位号确定正方向
  {
    dir = 0; // 0，cw
  }
  if (addr == 2 || addr == 4)
  {
    dir = 1;
  }
  // 之后在根据输入数值的正负  决定要正方向转还是反方向转。
  if (vel >= 0)
  {
    vel2 =   vel ;
  }
  else if (vel < 0) // 你这里没有修改clk的数值啊，还是没有实现反向。
  {
    if (dir == 0) // 修改运动方向,因为要向反方向运动。
    {
      dir = 1;
    }
    else if (dir == 1)
    {
      dir = 0;
    }
    vel2 =  -vel;
  }

  uint16_t vel3 = (uint16_t)(vel2 * 60 / (pi * diameter)) ; //(换算成mm/s)
  Emm_V5_Vel_Control(addr, dir, vel3, acc, snF); // 相比上面没有相对/绝对也就是raf，因为速度就是直接给的，也没有clk
}


//修改细分
void Emm_V5_Change_Subdivision_Up(void)
{

  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = 0x00; // 地址(全部修改)
  cmd[1] = 0x84; // 功能码
  cmd[2] = 0x8A; // 说明书上这么写的
  cmd[3] = 0X01; // 保存设置//
  cmd[4] = 0X40; // 32细分注意0X00是256细分（原来原来是给100走100mm，现在可以改一下100走10mm）
  cmd[5] = 0X6B; // 标志位

  // 发送命令
  HAL_UART_Transmit(&huart1, cmd, 6 ,0xFFFF);
}

void Emm_V5_Change_Subdivision_Down(void)
{

  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = 0x00; // 地址(全部修改)
  cmd[1] = 0x84; // 功能码
  cmd[2] = 0x8A; // 说明书上这么写的
  cmd[3] = 0X01; // 保存设置//注意0X00是256细分
  cmd[4] = 0X10; // 16细分（原来原来是给100走100mm，现在可以改一下100走10mm）
  cmd[5] = 0X6B; // 标志位

  // 发送命令
  HAL_UART_Transmit(&huart1, cmd, 6 ,0xFFFF);
}

void Emm_V5_Solve_Stop(void)
{
  uint8_t cmd[16] = {0};

  cmd[0] = 0x00; // 地址(全部修改)
  cmd[1] = 0x0E; // 功能码
  cmd[2] = 0x52; // 说明书上这么写的
  cmd[3] = 0x6B; // 保存设置//注意0X00是256细分

  usart_SendCmd(cmd, 4);
}

void send_four_cmd(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[64] = {0};

  // 第一次赋值
  cmd[0] = addr;                 // 地址
  cmd[1] = 0xFD;                 // 功能码
  cmd[2] = dir;                  // 方向
  cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
  cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
  cmd[5] = acc;                  // 加速度
  cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
  cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
  cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
  cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )
  cmd[10] = raF;                 // 相位/绝对标志
  cmd[11] = snF;                 // 多机同步运动标志
  cmd[12] = 0x6B;                // 校验字节

  // //第二次赋值
  // cmd[13] =  addr + 1;                  // 地址（递增）
  // cmd[14] =  0xFD;                      // 功能码
  // cmd[15] =  dir;                       // 方向
  // cmd[16] =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  // cmd[17] =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节
  // cmd[18] =  acc;                       // 加速度
  // cmd[19] =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  // cmd[20] =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  // cmd[21] =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  // cmd[22] =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  // cmd[23] =  raF;                       // 相位/绝对标志
  // cmd[24] =  snF;                       // 多机同步运动标志
  // cmd[25] =  0x6B;                      // 校验字节

  // // 第三次赋值
  // cmd[26] =  addr + 2;                  // 地址（递增）
  // cmd[27] =  0xFD;                      // 功能码
  // cmd[28] =  dir;                       // 方向
  // cmd[29] =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  // cmd[30] =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节
  // cmd[31] =  acc;                       // 加速度
  // cmd[32] =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  // cmd[33] =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  // cmd[34] =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  // cmd[35] =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  // cmd[36] =  raF;                       // 相位/绝对标志
  // cmd[37] =  snF;                       // 多机同步运动标志
  // cmd[38] =  0x6B;                      // 校验字节

  // // 第四次赋值
  // cmd[39] =  addr + 3;                  // 地址（递增）
  // cmd[40] =  0xFD;                      // 功能码
  // cmd[41] =  dir;                       // 方向
  // cmd[42] =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  // cmd[43] =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节
  // cmd[44] =  acc;                       // 加速度
  // cmd[45] =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  // cmd[46] =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  // cmd[47] =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  // cmd[48] =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  // cmd[49] =  raF;                       // 相位/绝对标志
  // cmd[50] =  snF;                       // 多机同步运动标志
  // cmd[51] =  0x6B;                      // 校验字节

  HAL_UART_Transmit(&huart1, cmd, 13 ,0xFFFF);
}

Motor_Status *motor1_status;
Motor_Status *motor2_status;
Motor_Status *motor3_status;
Motor_Status *motor4_status;

Motor_Status *init_motor_status(uint8_t address)
{
  Motor_Status *motor_status = (Motor_Status *)malloc(sizeof(Motor_Status));
  if (motor_status == NULL)
  {
    // fprintf(stderr, "Memory allocation failed\n");
    return NULL;
  }
  motor_status->address = address;
  motor_status->command = 0;
  motor_status->data_length = 0;
  motor_status->param_count = 0;
  motor_status->bus_voltage = 0;
  motor_status->bus_current = 0;
  motor_status->encoder_value = 0;
  motor_status->target_position = 0;
  motor_status->real_speed = 0;
  motor_status->real_position = 0;
  motor_status->position_error = 0;
  motor_status->ready_status = 0;
  motor_status->motor_status = 0;
  motor_status->check_bit = 0;
  motor_status->target_angle = 0;
  motor_status->real_rpm = 0;
  motor_status->real_angle = 0;
  motor_status->position_error_angle = 0;
  motor_status->last_distance_traveled = 0;
  motor_status->last_last_distance_traveled = 0;
  motor_status->distance_traveled = 0;
  motor_status->delta_distance_traveled = 0;
  motor_status->current_speed = 0 ;
  motor_status->first_run = true ;
  return motor_status;
}
//初始化电机状态
void Emm_V5_Init(void)
{
  motor1_status = init_motor_status(1);
  motor2_status = init_motor_status(2);
  motor3_status = init_motor_status(3);
  motor4_status = init_motor_status(4);

}

/// @brief 这个是在回调函数中被调用，用于解析收到的信息
/// @param data
/// @param status
void ParseMotor_Status(uint8_t *data, Motor_Status *status) {
  //  // 检查是否长时间没有接收到有效数据

  // 解析地址
  status->address = data[0];

  // 判断是否需要翻转符号（地址2或4时触发）
  int invert_sign = (status->address == 2 || status->address == 4) ? -1 : 1;

  // 解析命令
  status->command = data[1];

  // 解析数据长度
  status->data_length = data[2];

  // 解析参数个数
  status->param_count = data[3];

  // 解析总线电压（2字节大端序）
  status->bus_voltage = (data[4] << 8) | data[5];

  // 解析总线电流（2字节大端序）
  status->bus_current = (data[6] << 8) | data[7];

  // 解析编码器值（2字节大端序）
  status->encoder_value = (data[8] << 8) | data[9];

  // ---------- 符号修正部分开始 ----------
  // 解析目标位置（4字节大端序）  //这里大概就是那个意思，使用一个更大的int取接收这个函数
  uint32_t target_position = (data[11] << 24) | (data[12] << 16) | (data[13] << 8) | data[14];
  int64_t target_position_int = (int64_t)(target_position);
  
  status->target_position = (data[10] == 0x01) ? -target_position_int : target_position_int ;
  status->target_position *= invert_sign; // 符号翻转

  // 解析实时转速（2字节大端序）
  uint16_t real_speed = (data[16] << 8) | data[17];
  int32_t real_speed_int = (int32_t)(real_speed);

  status->real_speed = (data[15] == 0x01) ? -real_speed_int : real_speed_int ;
  status->real_speed *= invert_sign; // 符号翻转

  // 解析实时位置（4字节大端序）
  uint32_t real_position = (data[19] << 24) | (data[20] << 16) | (data[21] << 8) | data[22];
  int64_t real_position_int = (int64_t)(real_position);
  status->real_position = (data[18] == 0x01) ? -real_position_int : real_position_int ;
  status->real_position *= invert_sign; // 符号翻转
  //status->real_position *= 360.0f / 65535.0f ;

  // 解析位置误差（4字节大端序）
  uint32_t position_error = (data[24] << 24) | (data[25] << 16) | (data[26] << 8) | data[27];
  int64_t position_error_int = (int64_t)(position_error);
  status->position_error = (data[23] == 0x01) ? -position_error_int : position_error_int ;
  status->position_error *= invert_sign; // 符号翻转
  // ---------- 符号修正部分结束 ----------

  // 计算目标位置角度（自动继承符号）
  status->target_angle = (float)status->target_position * 360.0f / 65536.0f;

  // 计算实时转速（RPM，自动继承符号）
  status->real_rpm = (float)status->real_speed;

  // 计算实时位置角度（自动继承符号）
  status->real_angle = (float)status->real_position * 360.0f / 65536.0f;

  // 计算位置误差角度（自动继承符号）
  status->position_error_angle = (float)status->position_error * 360.0f / 65536.0f;

  

  // 计算路程和速度（自动继承符号）
  status->distance_traveled = (float)status->real_position * (pi * diameter) / 65536.0f;

  if(status->first_run == true)
  {
    status->first_run = false;

    status->last_distance_traveled =  status->distance_traveled   ;


    
  }

  status->current_speed = (float)status->real_speed * (pi * diameter) / 60.0f;

  // 解析状态标志
  status->ready_status = data[28];
  status->motor_status = data[29];
  status->check_bit = data[30];

  float current_time = HAL_GetTick();
  //更新lastdistance
  if(current_time - last_reveive_motor_time > 1000)
  {
    status->last_distance_traveled = 0;
  }

  status->delta_distance_traveled = status->distance_traveled- status->last_distance_traveled;
  // status->last_last_distance_traveled = status->last_distance_traveled;
  status->last_distance_traveled = status->distance_traveled;
}

void ConvertStatusToRawData(const Motor_Status *status, uint8_t *data, size_t data_size)
{
  if (data_size < 31)
  { // 确保数组足够大
    return;
  }

  // 填充地址
  data[0] = status->address;

  // 填充命令
  data[1] = status->command;

  // 填充数据长度
  data[2] = status->data_length;

  // 填充参数个数
  data[3] = status->param_count;

  // 填充总线电压（2 字节，大端序）
  data[4] = (status->bus_voltage >> 8) & 0xFF;
  data[5] = status->bus_voltage & 0xFF;

  // 填充总线电流（2 字节，大端序）
  data[6] = (status->bus_current >> 8) & 0xFF;
  data[7] = status->bus_current & 0xFF;

  // 填充编码器值（2 字节，大端序）
  data[8] = (status->encoder_value >> 8) & 0xFF;
  data[9] = status->encoder_value & 0xFF;

  // 填充目标位置符号位
  if (status->target_position < 0)
  {
    data[10] = 0x01;
  }
  else
  {
    data[10] = 0x00;
  }
  // 填充目标位置（4 字节，大端序）
  int32_t abs_target_position = (status->target_position < 0) ? -status->target_position : status->target_position;
  data[11] = (abs_target_position >> 24) & 0xFF;
  data[12] = (abs_target_position >> 16) & 0xFF;
  data[13] = (abs_target_position >> 8) & 0xFF;
  data[14] = abs_target_position & 0xFF;

  // 填充实时转速符号位
  if (status->real_speed < 0)
  {
    data[15] = 0x01;
  }
  else
  {
    data[15] = 0x00;
  }
  // 填充实时转速（2 字节，大端序）
  int16_t abs_real_speed = (status->real_speed < 0) ? -status->real_speed : status->real_speed;
  data[16] = (abs_real_speed >> 8) & 0xFF;
  data[17] = abs_real_speed & 0xFF;

  // 填充实时位置符号位
  if (status->real_position < 0)
  {
    data[18] = 0x01;
  }
  else
  {
    data[18] = 0x00;
  }
  // 填充实时位置（4 字节，大端序）
  int32_t abs_real_position = (status->real_position < 0) ? -status->real_position : status->real_position;
  data[19] = (abs_real_position >> 24) & 0xFF;
  data[20] = (abs_real_position >> 16) & 0xFF;
  data[21] = (abs_real_position >> 8) & 0xFF;
  data[22] = abs_real_position & 0xFF;

  // 填充位置误差符号位
  if (status->position_error < 0)
  {
    data[23] = 0x01;
  }
  else
  {
    data[23] = 0x00;
  }
  // 填充位置误差（4 字节，大端序）
  int32_t abs_position_error = (status->position_error < 0) ? -status->position_error : status->position_error;
  data[24] = (abs_position_error >> 24) & 0xFF;
  data[25] = (abs_position_error >> 16) & 0xFF;
  data[26] = (abs_position_error >> 8) & 0xFF;
  data[27] = abs_position_error & 0xFF;

  // 填充就绪状态标志
  data[28] = status->ready_status;

  // 填充电机状态标志
  data[29] = status->motor_status;

  // 填充结束位
  data[30] = status->check_bit;
}

void Emm_V5_Send_Four_Vel_Control( float Vel1, float Vel2, float Vel3, float Vel4, uint8_t acc, bool snF ,uint32_t delay_ms)
{
  Emm_V5_Vel_Control_Self(1, Vel1, acc, snF);
  HAL_Delay(delay_ms);
  Emm_V5_Vel_Control_Self(2, Vel2, acc, snF);
  HAL_Delay(delay_ms);
  Emm_V5_Vel_Control_Self(3, Vel3, acc, snF);
  HAL_Delay(delay_ms);
  Emm_V5_Vel_Control_Self(4, Vel4, acc, snF);
  HAL_Delay(delay_ms);
  Emm_V5_Synchronous_motion(0);
  HAL_Delay( delay_ms);
}

void Emm_V5_Clear_Angel(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0x0A; // 功能码
  cmd[2] = 0x6D; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令
  HAL_UART_Transmit(&huart1, cmd, 4  ,0xFFFF);
}

void Emm_V5_Clear_All_Angel(void)
{
  Emm_V5_Clear_Angel(0x01);
  HAL_Delay(10);
  Emm_V5_Clear_Angel(0x02);
  HAL_Delay(10);
  Emm_V5_Clear_Angel(0x03);
  HAL_Delay(10);
  Emm_V5_Clear_Angel(0x04);
  HAL_Delay(10);

}

void Emm_V5_GetMotor_Status(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0x43; // 功能码
  cmd[2] = 0x7A; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令

  HAL_UART_Transmit(&huart1, cmd, 4  ,0xFFFF);

}
