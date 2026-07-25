/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "bsp.h"
#include "app.h"
#include "Emm_V5.h"
#include "stdatomic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PACKET_TYPE_SHORT_HEADER 0x01
#define PACKET_TYPE_LONG_HEADER 0x02
#define SHORT_PACKET_LENGTH 6
#define LONG_PACKET_LENGTH 10
#define RASPI_DATE_TERMINATOR 0xFF
#define RX_BUFFER_SIZE 6 // 数据位长度

typedef enum
{
  WAITING_FOR_HEADER,
  RECEIVING_SHORT_PACKET,
  RECEIVING_LONG_PACKET,
  PACKET_RECEIVED
} UART_ReceiveState;
UART_ReceiveState receiveState;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 现在的问题其实是两次进入函数，所以试着加长接受区域，就不会那啥了
// 这个rxcmd1[4]目前没有使用，不会读取电机的反馈，到时候要看返回数据的话，就直接把数据通过串口3发送出去

/// 电机控制串口

#define RXCMD1_DMA_SIZE 31
uint8_t txcmd1[16];                  // 这个是用于给点击发送信号的，所以只要求16位数         // 要可能在开机的时候需要发送指令再次校准，这个到时候写一个专门的函数进行通讯协议
uint8_t rxcmd1_dma[RXCMD1_DMA_SIZE]; //   实际上不需要使用这个，只要在。。。期间不打开DMA 就行了DMA 写入的缓冲区
uint8_t rxcmd1_app[RXCMD1_DMA_SIZE]; // 应用程序读取的缓冲区

// 数据通信格式(任务码，x,y,z,状态)
// 发送格式(状态，完成状态)
// 上位机通信串口
uint8_t rxcmd2[16];  // 接收树莓派的数据
uint8_t txcmd_2[16]; // 发送给树莓派的数据

/// 远程调试串口
uint8_t rxcmd3[16];       // 接收远程串口的数据
uint8_t txcmd3[16] = {0}; // 发送给远程串口的数据

// 陀螺仪串口
#define RXCMD5_DMA_SIZE 33
uint8_t txcmd5[RXCMD5_DMA_SIZE];          // 要可能在开机的时候需要发送指令再次校准，这个到时候写一个专门的函数进行通讯协议
uint8_t rxcmd5_dma[RXCMD5_DMA_SIZE];      // DMA 写入的缓冲区
uint8_t rxcmd5_app[RXCMD5_DMA_SIZE];      // 应用程序读取的缓冲区
volatile bool new_data_available = false; // 标志位，表示新数据是否可用

uint8_t received_byte;
Raspi_Date packet_var2; // 这就是设置一个中间变量，最后反正只要用packet1
Raspi_Date *raspi_date = &packet_var2;

// 用于检测跳变的omega
float omega_1 = 0;
float omega = 0;                      // 陀螺仪反馈的角度
float current_angle_speed = 0;        // 陀螺仪反馈的角速度
uint32_t last_receive_hwt_time = 0;   // 上一次接收陀螺仪数据的时间
uint32_t last_reveive_motor_time = 0; // 上一次接收电机数据的时间

volatile uint32_t last_receive_raspi_time = 0; // 原子变量，中断中更新
uint32_t running_time = 0;                     // 有效运行时间

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // bsp_Init();

  // app_init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  ////

  ///////
  // HAL_UART_Transmit(&huart3, txcmd3, 16, 100);

  __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);

  HAL_UART_Receive_IT(&huart2, rxcmd2, 1);
  /// 这个既可以是满了的中断，也可以是空闲的中断，要在两个接收中断服务函数里面都写这个
  HAL_UART_Receive_DMA(&huart5, rxcmd5_dma, RXCMD5_DMA_SIZE);

  HAL_UART_Receive_DMA(&huart1, rxcmd1_dma, RXCMD1_DMA_SIZE);

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  // HAL_UART_Receive_IT(&huart5, &received_byte, 1);  // 启动接收中断

  memset(rxcmd5_dma, 0, RXCMD5_DMA_SIZE);

  memset(rxcmd1_dma, 0, RXCMD1_DMA_SIZE);

  bsp_Init();
  app_init();

  // stm32启动指令（发给树莓派）
  /// 这个是之前的启动代码
  // txcmd_2[0] = 0x00;
  // txcmd_2[1] = 0x01;
  // HAL_UART_Transmit(&huart2, txcmd_2, 2, 0xFFFF);
  // HAL_Delay(1000);
  uint32_t test_time = HAL_GetTick();

  ////////////////////////////这里需要加一个卡死条件
  /* USER CODE END 2 */

  /* Init scheduler */
  // osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  // MX_FREERTOS_Init();

  /* Start scheduler */
  // osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    ///////////////////这个end while后面不能写东西，不然下次加载vscode会报错
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //初赛代码，

    Stop_to_Order(1);

    Order_or_Precise_to_Plate(1);

    Plate_to_Rough(1);

    Rough_to_Precise(1);

    Order_or_Precise_to_Plate(2);

    Plate_to_Rough(2);

    Rough_to_Precise(2);


   

  //Test_Run_Only();

  //Test_Run_and_Blog();

  //Car_Ring_Test(10000);

  //Car_Ring_Timeout_Fast(10000 , Hight_Accurate );


  

  





    


  HAL_Delay(10000000);

  app_init();


  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 这个是imu的dma接收
// void  HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//  if (huart == &huart5)
//  {

//   // 获取当前时间
//   lastReceiveTime = HAL_GetTick();

//   for (int i = 0; i < 22; i++) {
//       rxcmd5_app[i] = rxcmd5_dma[i];
//   }
//   memset(rxcmd5_dma,  0  , RXCMD5_DMA_SIZE);

//   // 设置新数据可用标志
//   new_data_available = true;

//   //HAL_UART_Transmit(&huart5, rxcmd5_app, 22 ,0xFF);

//   HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rxcmd5_dma , RXCMD5_DMA_SIZE  );
// }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    // 停止 DMA 接收
    // HAL_UART_DMAStop(&huart1);

    // 获取接收到的数据长度
    // uint16_t data_length = RXCMD1_DMA_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    for (int i = 0; i < RXCMD1_DMA_SIZE; i++)
    {
      rxcmd1_app[i] = rxcmd1_dma[i];
    }

    memset(rxcmd1_dma, 0, RXCMD1_DMA_SIZE);

    if (rxcmd1_app[0] >= 0x00 && rxcmd1_app[30] == 0x6B)
    {
      // 超时检测变量
      last_reveive_motor_time = HAL_GetTick();
      // 根据地址判断是哪个电机的数据
      uint8_t motor_id = rxcmd1_app[0]; // 假设第一个字节是电机 ID
      switch (motor_id)
      {
      case 1:
        ParseMotor_Status(rxcmd1_app, motor1_status);
        break;
      case 2:
        ParseMotor_Status(rxcmd1_app, motor2_status);
        break;
      case 3:
        ParseMotor_Status(rxcmd1_app, motor3_status);
        break;
      case 4:
        ParseMotor_Status(rxcmd1_app, motor4_status);
        break;
      default:

        break;
      }
    }
    else
    {
      memset(rxcmd1_app, 0, RXCMD1_DMA_SIZE);
    }

    HAL_UART_Transmit(&huart5, rxcmd1_app, 1, 0xFFFF);
    memset(rxcmd1_app, 0, RXCMD1_DMA_SIZE);
    HAL_UART_Receive_DMA(&huart1, rxcmd1_dma, RXCMD1_DMA_SIZE);
  }

  if (huart == &huart5)
  {
    static int wrap_count = 0;   // 静态本地变量，记录圈数
    static float prev_omega = 0; // 静态本地变量，记录上一次的角度
    uint8_t txcmd_33[0] = {0};

    // 获取当前时间
    last_receive_hwt_time = HAL_GetTick();

    for (int i = 0; i < RXCMD5_DMA_SIZE; i++)
    {
      rxcmd5_app[i] = rxcmd5_dma[i];
    }

    //
    /// 这里想写一个无线串口的转发，但是这是每次收到之后，不是按照“包”
    ////HAL_UART_Transmit(&huart3, rxcmd5_app, RXCMD5_DMA_SIZE ,0xFFFF);
    //
    memset(rxcmd5_dma, 0, RXCMD5_DMA_SIZE);

    for (int i = 0; i < RXCMD5_DMA_SIZE; i++)
    {
      if (rxcmd5_app[i] == 0x2E)
      {
        if (rxcmd5_app[i + 1] == 0x2E && rxcmd5_app[i + 2] == 0x2E)
        {
          memset(rxcmd5_app, 0, RXCMD5_DMA_SIZE);
        }
      }
    }

    for (int i = 0; i < RXCMD5_DMA_SIZE - 7; i++)
    {

      if (rxcmd5_app[i] == 0x55 && rxcmd5_app[i + 1] == 0x53)

      {
        // 计算当前角度（ - 180 到 180 度）

        omega_1 = ((float)((int16_t)(rxcmd5_app[i + 7] << 8) | rxcmd5_app[i + 6])) / 32768 * 180;
        // omega_1 = ((float)((int16_t)(rx_buffer[5] << 8) | rx_buffer[6])) / 32768 * 180;

        // new_data_available = false; // 清除标志

        // txcmd_33[0] = (uint8_t)(omega_1);

        // HAL_UART_Transmit(&huart3, txcmd_33, 1, 0xFFFF);
        // HAL_UART_Transmit(&huart3, txcmd_33 , 1, 0xFFFF);

        // 检测跳变并更新圈数
        if (prev_omega < 180 && prev_omega > 90 && omega_1 < -90 && omega_1 > -180)
        {
          wrap_count++; // 正向跳变（经过 180 度）
        }
        else if (prev_omega < -90 && prev_omega > -180 && omega_1 > 90 && omega_1 < 180)
        {
          wrap_count--; // 负向跳变（经过 -180 度）
        }
        else
        {
          wrap_count = wrap_count;
        }

        if (omega_1 != 180 && omega_1 != -180)
        {
          // 计算实际角度
          omega = omega_1 + wrap_count * 360.0f;

          // uint8_t txcmd_33[1] = {0};

          // txcmd_33[0] = (uint8_t)(omega);

          // HAL_UART_Transmit(&huart3, txcmd_33, 1, 0xFF);

          // 更新上一次的角度
          prev_omega = omega_1;

          // return omega;
          //         }
        }
      }

      if (rxcmd5_app[i] == 0x55 && rxcmd5_app[i + 1] == 0x52)
      {
        current_angle_speed = ((float)((int16_t)(rxcmd5_app[i + 7] << 8) | rxcmd5_app[i + 6])) / 32768 * 2000;
        // omega_1 = ((float)((int16_t)(rx_buffer[5] << 8) | rx_buffer[6])) / 32768 * 180;

        // float ceshi[2] = {0};

        // ceshi[0] = current_angle_speed;
        // ceshi[1] = omega ;

        // // new_data_available = false; // 清除标志
        // SendMultiFloat2Vofa(ceshi, 2);
      }
    }
    // }

    // HAL_UART_Transmit(&huart5, rxcmd5_app, 22 ,0xFF);

    HAL_UART_Receive_DMA(&huart5, rxcmd5_dma, RXCMD5_DMA_SIZE);
  }

  if (huart == &huart2)
  {

    if (receiveState == WAITING_FOR_HEADER)
    {
      // 检查帧头
      if (rxcmd2[0] == PACKET_TYPE_SHORT_HEADER)
      {
        receiveState = RECEIVING_SHORT_PACKET;
        // 不需要设置receivedLength，因为包长度是固定的
        HAL_UART_Receive_IT(&huart2, &rxcmd2[1], SHORT_PACKET_LENGTH - 1); // 减去帧头长度
      }
      else if (rxcmd2[0] == PACKET_TYPE_LONG_HEADER)
      {
        receiveState = RECEIVING_LONG_PACKET;
        // 不需要设置receivedLength，因为包长度是固定的
        HAL_UART_Receive_IT(&huart2, &rxcmd2[1], LONG_PACKET_LENGTH - 1); // 减去帧头长度
      }
      else
      {
        // 未知的帧头，重置状态并继续等待新的帧头
        receiveState = WAITING_FOR_HEADER;
        HAL_UART_Receive_IT(&huart2, rxcmd2, 1);
      }
    }
    else if (receiveState == RECEIVING_SHORT_PACKET || receiveState == RECEIVING_LONG_PACKET)
    {
      // 数据包接收完成
      receiveState = PACKET_RECEIVED;

      // 在这里处理接收到的数据包
      if (rxcmd2[0] == PACKET_TYPE_SHORT_HEADER)
      {
        if (rxcmd2[SHORT_PACKET_LENGTH - 1] == RASPI_DATE_TERMINATOR)
        { // 收到不回复

          // 解算order
          uint16_t ref_order1_recieved = (rxcmd2[1] << 8) | rxcmd2[2];
          uint16_t ref_order2_recieved = (rxcmd2[3] << 8) | rxcmd2[4];

          uint16_t ref_order1_recieved_original = ref_order1_recieved;
          uint16_t ref_order2_recieved_original = ref_order2_recieved;

          raspi_date->order1[2] = ref_order1_recieved % 10;
          ref_order1_recieved /= 10;
          raspi_date->order1[1] = ref_order1_recieved % 10;
          ref_order1_recieved /= 10;
          raspi_date->order1[0] = ref_order1_recieved;

          raspi_date->order2[2] = ref_order2_recieved % 10;
          ref_order2_recieved /= 10;
          raspi_date->order2[1] = ref_order2_recieved % 10;
          ref_order2_recieved /= 10;
          raspi_date->order2[0] = ref_order2_recieved;
        }
        else
        {
          // 收错回复
          uint8_t Reply[1] = {0x00};
          Reply[0] = 0xFF;
          HAL_UART_Transmit(&huart2, Reply, 1, 0xFFFF);
          memset(rxcmd2, 0, sizeof(rxcmd2));
        }
      }
      else if (rxcmd2[0] == PACKET_TYPE_LONG_HEADER)
      {
        if (rxcmd2[LONG_PACKET_LENGTH - 1] == RASPI_DATE_TERMINATOR)
        { // 收到不回复
          // 解算树莓派cmd数据
          raspi_date->ref_y = (float)((rxcmd2[1] << 8) | rxcmd2[2]);
          raspi_date->ref_y = raspi_date->ref_y * 640 / 1000; // 这个修正逻辑看起来是有问题的，因为ref_x还没有被赋值。
          raspi_date->ref_x = (float)((rxcmd2[3] << 8) | rxcmd2[4]);
          raspi_date->ref_x = raspi_date->ref_x * 480 / 1000; // 同样，这个修正逻辑也是有问题的。
          raspi_date->ref_z = (int32_t)((rxcmd2[5] << 8) | rxcmd2[6]);
          raspi_date->ref_z = raspi_date->ref_z - 26946; // 这个修正逻辑可能也是不正确的，取决于你的具体需求。
          raspi_date->taskID = rxcmd2[7];
          raspi_date->taskstate = rxcmd2[8];

          last_receive_raspi_time = HAL_GetTick();

          // ////用于计算超出时间的逻辑
          // atomic_store(&last_receive_raspi_time, HAL_GetTick());
        }
        else
        {
          uint8_t Reply[1] = {0x00};
          Reply[0] = 0xFF;
          HAL_UART_Transmit(&huart2, Reply, 1, 0xFFFF);
          memset(rxcmd2, 0, sizeof(rxcmd2));
        }
      }
      // 准备接收下一个数据包
      receiveState = WAITING_FOR_HEADER;
      HAL_UART_Receive_IT(&huart2, rxcmd2, 1); // 开始等待新的帧头
    }
  }
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
