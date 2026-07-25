#ifndef __EMM_V5_H
#define __EMM_V5_H

// #include "main.h"
#include "stdbool.h"
#include "stdint.h"
#include "usart.h"

/**********************************************************
***	Emm_V5.0�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

#define		ABS(x)		((x) > 0 ? (x) : -(x)) 

#define pi  3.14159265358979323846f
#define diameter 98.46f

#define MOTOR1_ADDRESS 0x01
#define MOTOR2_ADDRESS 0x02
#define MOTOR3_ADDRESS 0x03
#define MOTOR4_ADDRESS 0x04

#define NUM_MOTORS 4

typedef enum {
	S_VER   = 0,			/* ��ȡ�̼��汾�Ͷ�Ӧ��Ӳ���汾 */
	S_RL    = 1,			/* ��ȡ��ȡ���������� */
	S_PID   = 2,			/* ��ȡPID���� */
	S_VBUS  = 3,			/* ��ȡ���ߵ�ѹ */
	S_CPHA  = 5,			/* ��ȡ����� */
	S_ENCL  = 7,			/* ��ȡ�������Ի�У׼��ı�����ֵ */
	S_TPOS  = 8,			/* ��ȡ���Ŀ��λ�ýǶ� */
	S_VEL   = 9,			/* ��ȡ���ʵʱת�� */
	S_CPOS  = 10,			/* ��ȡ���ʵʱλ�ýǶ� */
	S_PERR  = 11,			/* ��ȡ���λ�����Ƕ� */
	S_FLAG  = 13,			/* ��ȡʹ��/��λ/��ת״̬��־λ */
	S_Conf  = 14,			/* ��ȡ�������� */
	S_State = 15,			/* ��ȡϵͳ״̬���� */
	S_ORG   = 16,     /* ��ȡ���ڻ���/����ʧ��״̬��־λ */
}SysParams_t;


typedef struct {
    uint8_t address;         // 设备地址，表示系统中设备的唯一标识（1 字节）
    uint8_t command;         // 命令类型，表示当前接收到的命令（1 字节）
    uint16_t data_length;    // 数据长度，表示数据包中有效数据的字节数（2 字节）
    uint16_t param_count;    // 参数个数，表示数据包中包含的参数数量（2 字节）
    uint16_t bus_voltage;    // 总线电压，表示当前系统总线的电压值（2 字节，大端序）
    uint16_t bus_current;    // 总线电流，表示当前系统总线的电流值（2 字节，大端序）
    uint16_t encoder_value;  // 编码器值，表示电机编码器的当前读数（2 字节，大端序）
    int64_t target_position; // 目标位置，表示电机需要达到的目标位置（4 字节，大端序，带符号）
    int32_t real_speed;      // 实时转速，表示电机的当前转速（2 字节，大端序，带符号）
    int64_t real_position;   // 实时位置，表示电机的当前位置（4 字节，大端序，带符号）
    int64_t position_error;  // 位置误差，表示目标位置与当前位置的差值（4 字节，大端序，带符号）
    uint8_t ready_status;    // 就绪状态标志，表示设备是否处于就绪状态（1 字节）
    uint8_t motor_status;    // 电机状态标志，表示电机的当前运行状态（1 字节）
    uint8_t check_bit;      // 校验位，用于验证数据包的完整性（1 字节）
    float target_angle;     // 电机目标位置（角度），单位：度（°）
    float real_rpm;         // 电机实时转速，单位：转/分钟（RPM）
    float real_angle;       // 电机实时位置（角度），单位：度（°）
    float position_error_angle; // 电机位置误差（角度），单位：度（°）
    float distance_traveled; // 电机走过的路程，单位：毫米（mm）
    float last_distance_traveled;//上一次获取的电机走过的路程，单位：毫米（mm）
    float last_last_distance_traveled;//上上一次获取的电机走过的路程，单位：毫米（mm）//避免电机重上电编码器归零重转
    float delta_distance_traveled;//上一次获取的电机走过的路程和本次获取的电机走过的路程之差，单位：毫米（mm）
    float current_speed;    // 电机当前的速度，单位：毫米/秒（mm/s）
    bool first_run ;
} Motor_Status;

// 声明四个全局变量，分别用于存储四个电机的信息
extern Motor_Status *  motor1_status;
extern Motor_Status *  motor2_status;
extern Motor_Status *  motor3_status;
extern Motor_Status *  motor4_status;

void Emm_V5_Init(void);

void Emm_V5_GetMotor_Status(uint8_t address);

void ParseMotor_Status(uint8_t *data, Motor_Status *status);

void ConvertStatusToRawData(const Motor_Status *status, uint8_t *data, size_t data_size);
void Emm_V5_Send_Four_Vel_Control( float Vel1, float Vel2, float Vel3, float Vel4, uint8_t acc, bool snF ,uint32_t delay_ms);
void Emm_V5_GetMotor_Status(uint8_t address);








/**********************************************************
*** ע�⣺ÿ�������Ĳ����ľ���˵��������Ķ�Ӧ������ע��˵��
**********************************************************/
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr); // ����ǰλ������
void Emm_V5_Reset_Clog_Pro(uint8_t addr); // �����ת����
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s); // ��ȡ����
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode); // ���������޸Ŀ���/�ջ�����ģʽ
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF); // ���ʹ�ܿ���
//目前是3200个脉冲一圈，假如要调整成毫米的话
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF); // �ٶ�ģʽ����

void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF); // λ��ģʽ����
void Emm_V5_Stop_Now(uint8_t addr, bool snF); // �õ������ֹͣ�˶�
void Emm_V5_Synchronous_motion(uint8_t addr); // �������ͬ����ʼ�˶�//这个的意思是开启同步运动，直接00地址广播即可
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF); // ���õ�Ȧ��������λ��
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // �޸Ļ������
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF); // �������������
void Emm_V5_Origin_Interrupt(uint8_t addr); // ǿ���жϲ��˳�����
//自己改的位置控制函数，不同轮子正方向设置，还有clk可以给负数然后反向转
void Emm_V5_Pos_Control_Self(uint8_t addr,  uint16_t vel, uint8_t acc, int32_t clk, bool raF, bool snF);
//速度控制函数，预处理正方向和vel为负数的情况
void Emm_V5_Vel_Control_Self(uint8_t addr, float  vel, uint8_t acc, bool snF);
//修改细分为160，在进入微调模式使用
void Emm_V5_Change_Subdivision_Up(void);
//修改细分为16，大范围位移使用
void Emm_V5_Change_Subdivision_Up(void);
void Emm_V5_Solve_Stop(void);

void send_four_cmd(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);

void Emm_V5_Clear_Angel(uint8_t addr);
void Emm_V5_Clear_All_Angel(void);

#endif
