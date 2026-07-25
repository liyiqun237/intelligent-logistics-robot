#include "task.h"

Delta_Position_XY Coulor_feedback(enum Color color) // 测试没有问题，只要每次进行一个新的任务之前，把颜色重置回绿色就行
{
    static enum Color color_last = GREEN; // 直接默认位置在绿色前面，然后通过计算，算出需要的位移,
    enum Color color_now = color;
    Delta_Position_XY delta_position = {0, 0};

    Update_Car_Status(5);

    switch (car->car_face)
    {
    case Forward:
        delta_position.delta_position_x = 0.0f;
        delta_position.delta_position_y = (float)((color - color_last) * 150.0f);
        break;
    case Left:

        delta_position.delta_position_x = (float)(-(color - color_last) * 150.0f);
        delta_position.delta_position_y = 0.0f;
        break;
    case Back:

        delta_position.delta_position_x = 0.0f;
        delta_position.delta_position_y = (float)(-(color - color_last) * 150.0f);
        break;
    case Right:

        delta_position.delta_position_x = (float)((color - color_last) * 150.0f);
        delta_position.delta_position_y = 0.0f;
        break;
    default:

        delta_position.delta_position_x = 0.0f;
        delta_position.delta_position_y = 0.0f;
        break;
    }
    color_last = color;

    return delta_position;
}

void Reset_to_Color_and_Move(enum Color color, float delta_x, float delta_y, uint32_t TIMEOUT , bool is_rough)
{
    //////////////////这个就是会完全执行控制逻辑
    HAL_Delay(10);

    Delta_Position_XY delta_position = Coulor_feedback(color);
    ////这个延时缓解是结构体赋值完成后可能造成的卡顿
    HAL_Delay(10);

    Update_Car_Status(5);

    float target_omega = 0;

    if(car->car_face == Forward)
    {
        target_omega = 0;
    }
    else if(car->car_face == Left)
    {
        target_omega = 90;
    }
    else if(car->car_face == Back)
    {
        target_omega = 180;
    }
    else if(car->car_face == Right)
    {
        target_omega = 270;
    }

    if(is_rough)
    {
        Move_To_Position_XYZ_Color(delta_position.delta_position_x + delta_x, delta_position.delta_position_y + delta_y, target_omega , TIMEOUT);
    }
    else
    {
        Move_To_Position_XYZ(delta_position.delta_position_x + delta_x, delta_position.delta_position_y + delta_y, target_omega , TIMEOUT);
    }

}

void Send_and_Wait_for_Cmd(uint8_t task_id_send, uint8_t task_state_send, uint8_t task_id_wait, uint8_t task_state_wait)
{
    uint32_t start_time = HAL_GetTick();
    while (1)
    {

        if (raspi_date->taskID == task_id_wait && raspi_date->taskstate == task_state_wait)
        {
            break;
        }
        else
        {
            txcmd_2[0] = task_id_send;
            txcmd_2[1] = task_state_send;
            HAL_UART_Transmit(&huart2, txcmd_2, 2, 0xFF);
        }

        HAL_Delay(100);
    }
}

void Send_Task_Status(uint8_t task_id_send, uint8_t task_state_send)
{
    txcmd_2[0] = task_id_send;
    txcmd_2[1] = task_state_send;
    HAL_UART_Transmit(&huart2, txcmd_2, 2, 0xFF);
}

void Wait_for_Task_Status(uint8_t task_id_wait, uint8_t task_state_wait)
{
    while (1)
    {

        if (raspi_date->taskID == task_id_wait && raspi_date->taskstate == task_state_wait)
        {
            break;
        }
        

        HAL_Delay(50);
    }

}

void Stop_to_Order(bool is_swap)
{

    Move_To_Position_XYZ(-190, 190, 0, 1000);
    /////可能需要套等待

    Move_To_Position_XYZ( 0, 350 , 0, 1500 );

    if (is_swap)
    {
        uint32_t start_time = HAL_GetTick();

        while (1)
        {

            if (HAL_GetTick() - start_time > 2000)
            {
                Move_To_Position_XYZ(0, 100, 0, 500);

                Move_To_Position_XYZ(0, -100, 0, 500);

                start_time = HAL_GetTick();
            }

            if (raspi_date->order1[0] != 0 && raspi_date->order1[1] != 0 && raspi_date->order1[2] != 0)
            {
                break;
            }

            HAL_Delay(100);
        }
    }
    
}

void Order_or_Precise_to_Plate(uint8_t Rounds)
{
    Delta_Position_XY delta_position = {0, 0};
    if (Rounds == 1)
    {

        /////可能需要套等待
        Move_To_Position_XYZ(55, 930, 0, 2500);
        Send_and_Wait_for_Cmd(1, 0, 1, 1);
        Car_Blog(1000, 0); /////这个是抓取的高度
        Send_and_Wait_for_Cmd(1, 2, 1, 3);
    }
    if (Rounds == 2)
    {
        delta_position = Coulor_feedback(GREEN);
        HAL_Delay(100);


        ///这个880其实是给的多了，之后这个可以进行修改。


        Move_To_Position_XYZ(880 + delta_position.delta_position_x, -100 + delta_position.delta_position_y, 90, 3000);
        Turn_To(0, 1.0, 1700);
        Move_To_Position_XYZ( 45 , -360 , 0 , 2000);
        Send_and_Wait_for_Cmd(1, 0, 1, 1);
        Car_Blog(1000, 0);
        Send_and_Wait_for_Cmd(1, 2, 1, 3);
        
    }
}

/// @brief
/// @param Rounds
void Plate_to_Rough(uint8_t Rounds)
{
    Move_To_Position_XYZ(-80, -420, 0, 2000); // 这里首先要多走20
    Turn_To(180, 1.0, 2000);
    // Move_To_Position_XYZ(-1700, 0, 180, 3000);
    CarMoveTo(-1700, 0, 180, 3000);
    Delta_Position_XY delta_position = Coulor_feedback(GREEN);
    //////这里进行了方向和位置的转变
    if (Rounds == 1)
    { ////////移动到对应颜色前
        delta_position = Coulor_feedback(raspi_date->order1[0]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);
        /// 等待并且进行car_blog
        Send_and_Wait_for_Cmd(2, 0, 2, 1);
        Car_Blog(1000, 1);
        Send_and_Wait_for_Cmd(2, 2, 2, 3);
        Car_Ring(5000);
        Send_and_Wait_for_Cmd(2, 4, 2, 5);

        delta_position = Coulor_feedback(raspi_date->order1[1]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);

        Send_and_Wait_for_Cmd(2, 6, 2, 7);
        Car_Ring(5000);
        Send_and_Wait_for_Cmd(2, 8, 2, 9);

        delta_position = Coulor_feedback(raspi_date->order1[2]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);

        Send_and_Wait_for_Cmd(2, 10, 2, 11);
        Car_Ring(5000);
        Send_and_Wait_for_Cmd(2, 12, 2, 13);

        ////再抓起来

        delta_position = Coulor_feedback(raspi_date->order1[0]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);

        Send_and_Wait_for_Cmd(2, 14, 2, 15);

        delta_position = Coulor_feedback(raspi_date->order1[1]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);

        Send_and_Wait_for_Cmd(2, 16, 2, 17);

        delta_position = Coulor_feedback(raspi_date->order1[2]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);
    }
    else if (Rounds == 2)
    {
        delta_position = Coulor_feedback(raspi_date->order2[0]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);
        /// 等待并且进行car_blog
        Send_and_Wait_for_Cmd(2, 0, 2, 1);
        Car_Blog(1000, 1);
        Send_and_Wait_for_Cmd(2, 2, 2, 3);
        Car_Ring(5000);
        Send_and_Wait_for_Cmd(2, 4, 2, 5);

        delta_position = Coulor_feedback(raspi_date->order2[1]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);

        Send_and_Wait_for_Cmd(2, 6, 2, 7);
        Car_Ring(5000);
        Send_and_Wait_for_Cmd(2, 8, 2, 9);

        delta_position = Coulor_feedback(raspi_date->order2[2]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);

        Send_and_Wait_for_Cmd(2, 10, 2, 11);
        Car_Ring(5000);
        Send_and_Wait_for_Cmd(2, 12, 2, 13);

        ////再抓起来

        delta_position = Coulor_feedback(raspi_date->order2[0]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);

        Send_and_Wait_for_Cmd(2, 14, 2, 15);

        delta_position = Coulor_feedback(raspi_date->order2[1]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);

        Send_and_Wait_for_Cmd(2, 16, 2, 17);

        delta_position = Coulor_feedback(raspi_date->order2[2]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);
    }
    Send_and_Wait_for_Cmd(2, 18, 2, 19);

}

void Rough_to_Precise(uint8_t Rounds)
{
    Delta_Position_XY delta_position = {0, 0};
    delta_position = Coulor_feedback(GREEN);
    Move_To_Position_XYZ(100 + delta_position.delta_position_x, 800 + delta_position.delta_position_y, 180, 3000);

    Turn_To(90, 1.0, 1700);

    Coulor_feedback(GREEN);

    /// 移动到对应颜色前面的代码

    if (Rounds == 1)
    {
        delta_position = Coulor_feedback(raspi_date->order1[0]);
    }
    else if (Rounds == 2)
    {
        delta_position = Coulor_feedback(raspi_date->order2[0]);
    }
    Move_To_Position_XYZ(800 + delta_position.delta_position_x, 70 + delta_position.delta_position_y, 90, 3000);

    Send_and_Wait_for_Cmd(3, 0, 3, 1);

    Car_Blog(1000, 1);

    Send_and_Wait_for_Cmd(3, 2, 3, 3);

    if (Rounds == 1)
    {

        Car_Ring(5000);
        Send_and_Wait_for_Cmd(3, 4, 3, 5);

        delta_position = Coulor_feedback(raspi_date->order1[1]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 90, 2000);

        Send_and_Wait_for_Cmd(3, 6, 3, 7);
        Car_Ring(5000);
        Send_and_Wait_for_Cmd(3, 8, 3, 9);

        delta_position = Coulor_feedback(raspi_date->order1[2]);
        Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 90, 2000);

        Send_and_Wait_for_Cmd(3, 10, 3, 11);
        Car_Ring(5000);
        Send_and_Wait_for_Cmd(3, 12, 3, 13);
    }
    if (Rounds == 2)
    {
        delta_position = Coulor_feedback(raspi_date->order2[1]);
        Move_To_Position_XYZ(delta_position.delta_position_x, delta_position.delta_position_y, 90, 2000);

        Send_and_Wait_for_Cmd(3, 4, 3, 5);

        delta_position = Coulor_feedback(raspi_date->order2[2]);
        Move_To_Position_XYZ(delta_position.delta_position_x, delta_position.delta_position_y, 90, 2000);

        Send_and_Wait_for_Cmd(3, 6, 3, 7);

        delta_position = Coulor_feedback(GREEN);

        /// go back，放到了这里的最后
        Move_To_Position_XYZ(880 + delta_position.delta_position_x, -100 + delta_position.delta_position_y, 90, 3000);
        Move_To_Position_XYZ(0, -1450, 90, 3000);
        Move_To_Position_XYZ(175, -370, 90, 3000); // 测190给200
    }
}

void Test_Run_and_Blog(void)
{
    Move_To_Position_XYZ(-190, 190 , 0, 1000);
    /////可能需要套等待

    Move_To_Position_XYZ( 0, 350 , 0, 1500);

    Move_To_Position_XYZ(55, 930, 0, 2500);
    Send_and_Wait_for_Cmd(1, 0, 1, 1);
    Car_Blog(1000, 0); /////这个是抓取的高度

    Move_To_Position_XYZ(-80, -420, 0, 2000); // 这里首先要多走20
    Turn_To(180, 1.0, 2000);
    // Move_To_Position_XYZ(-1700, 0, 180, 3000);
    CarMoveTo(-1700, 0, 180, 3000);

    Coulor_feedback(GREEN);

     Delta_Position_XY delta_position = {0, 0};

    delta_position = Coulor_feedback(raspi_date->order1[0]);

    Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 3000);



    Send_and_Wait_for_Cmd(2, 0, 2, 1);
    Car_Blog(1000, 1);

   


    delta_position = Coulor_feedback(GREEN);

    Move_To_Position_XYZ(100 + delta_position.delta_position_x, 800 + delta_position.delta_position_y, 180, 3000);

    Turn_To(90, 1.0, 1700);

    delta_position = Coulor_feedback(raspi_date->order1[0]);


    /// 移动到对应颜色前面的代码

    Move_To_Position_XYZ(800 + delta_position.delta_position_x, 70 + delta_position.delta_position_y, 90, 3000);

    Send_and_Wait_for_Cmd(3, 0, 3, 1);

    Car_Blog(1000, 1);

    delta_position = Coulor_feedback(GREEN);

    HAL_Delay(100);

    Move_To_Position_XYZ(880 + delta_position.delta_position_x, -100 + delta_position.delta_position_y, 90, 3000);
    Turn_To(0, 1.0, 1700);
    Move_To_Position_XYZ(45, -360, 0, 2000);
    Send_and_Wait_for_Cmd(1, 0, 1, 1);
    Car_Blog(1000, 0);

    Move_To_Position_XYZ(-80, -420, 0, 2000); // 这里首先要多走20
    Turn_To(180, 1.0, 2000);
    // Move_To_Position_XYZ(-1700, 0, 180, 3000);
    CarMoveTo(-1700, 0, 180, 3000);

    delta_position = Coulor_feedback(raspi_date->order2[0]);

    Move_To_Position_XYZ_Color(delta_position.delta_position_x, delta_position.delta_position_y, 180, 3000);

    Send_and_Wait_for_Cmd(2, 0, 2, 1);
    Car_Blog(1000, 1);

    

    Coulor_feedback(GREEN);

    

    Move_To_Position_XYZ(100 + delta_position.delta_position_x, 800 + delta_position.delta_position_y, 180, 3000);

    Turn_To(90, 1.0, 1700);

    Coulor_feedback(GREEN);

    delta_position = Coulor_feedback(raspi_date->order2[0]);

    /// 移动到对应颜色前面的代码

    Move_To_Position_XYZ(800 + delta_position.delta_position_x, 70 + delta_position.delta_position_y, 90, 3000);

    Send_and_Wait_for_Cmd(3, 0, 3, 1);

    Car_Blog(1000, 1);

    delta_position = Coulor_feedback(GREEN);

    /// go back，放到了这里的最后
    Move_To_Position_XYZ(880 + delta_position.delta_position_x, -100 + delta_position.delta_position_y, 90, 3000);
    Move_To_Position_XYZ(0, -1450, 90, 3000);
    Move_To_Position_XYZ(200, -370, 90, 3000); // 测190给200

}


void Test_Run_Only(void)
{
    Move_To_Position_XYZ(-190, 190 , 0, 1000);
    /////可能需要套等待

    Move_To_Position_XYZ( 0, 350 , 0, 1500);

    Move_To_Position_XYZ(55, 930, 0, 2500);

    

    Move_To_Position_XYZ(-80, -420, 0, 2000); // 这里首先要多走20

    Turn_To(180, 1.0, 2000);
    // Move_To_Position_XYZ(-1700, 0, 180, 3000);

    ////这里和之前的代码有了修改，这个走的多了一些
    CarMoveTo(-1750, 0, 180, 3000);

    




    Move_To_Position_XYZ(100 , 800 , 180, 3000);

    Turn_To(90, 1.0, 1700);

    


    /// 移动到对应颜色前面的代码

    Move_To_Position_XYZ(800 , 70 , 90, 3000);

    

    HAL_Delay(100);

    Move_To_Position_XYZ(880 , -100  , 90, 3000);

    Turn_To(0, 1.0, 1700);

    Move_To_Position_XYZ(45, -360, 0, 2000);
    
    Move_To_Position_XYZ(-80, -420, 0, 2000); // 这里首先要多走20

    Turn_To(180, 1.0, 2000);
    // Move_To_Position_XYZ(-1700, 0, 180, 3000);
    CarMoveTo(-1750, 0, 180, 3000);

    

    

    

    

    

    Move_To_Position_XYZ(100 , 800 , 180, 3000);

    Turn_To(90, 1.0, 1700);

    
    
    /// 移动到对应颜色前面的代码

    Move_To_Position_XYZ(800 , 70 , 90, 3000);


    /// go back，放到了这里的最后
    Move_To_Position_XYZ(880 , -100 , 90, 3000);

    Move_To_Position_XYZ(0, -1450, 90, 3000);

    Move_To_Position_XYZ(200, -390, 90, 3000); // 测190给200

}


void Test_Mechanical_Heart(void)
{
    while (1)
    {
        if (raspi_date->taskID == 1 && raspi_date->taskstate == 1)
        {
            break;
        }
        HAL_Delay(50);
    }

    Car_Ring(5000);
}



void Stop_to_Rough(uint8_t Rounds)
{
    Move_To_Position_XYZ(-190, 540, 0, 2000);
    /////可能需要套等待
    Move_To_Position_XYZ(0, 510, 0, 2000);

    Turn_To(180, 1.0, 1700);

    // Move_To_Position_XYZ(-1620, 0, 180, 3000);////这个数值需要测量

    CarMoveTo(-1620, 0, 180, 3000);

    Delta_Position_XY delta_position = Coulor_feedback(GREEN);
    HAL_Delay(100);
    //////这里进行了方向和位置的转变

    delta_position = Coulor_feedback(raspi_date->order1[0]);
    HAL_Delay(100);
    Move_To_Position_XYZ(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);

    /// 等待并且进行car_blog
    Send_and_Wait_for_Cmd(1, 0, 1, 1);
    Car_Blog(1000, 1);
    Send_and_Wait_for_Cmd(1, 2, 1, 3);
    Car_Ring(5000);
    Send_and_Wait_for_Cmd(1, 4, 1, 5);

    delta_position = Coulor_feedback(raspi_date->order1[1]);
    HAL_Delay(100);
    Move_To_Position_XYZ(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);

    /// 等待并且进行car_blog
    Send_and_Wait_for_Cmd(1, 6, 1, 7);

    delta_position = Coulor_feedback(raspi_date->order1[2]);
    HAL_Delay(100);
    Move_To_Position_XYZ(delta_position.delta_position_x, delta_position.delta_position_y, 180, 2000);

    /// 等待并且进行car_blog
    Send_and_Wait_for_Cmd(1, 8, 1, 9);
}

void Rough_to_Plate(uint8_t Rounds)
{

    Delta_Position_XY delta_position = Coulor_feedback(GREEN);
    HAL_Delay(100);
    Move_To_Position_XYZ(50 + delta_position.delta_position_x, delta_position.delta_position_y, 180, 1000);

    // Turn_To(270, 1.0, 1700);

    Turn_To(0, 1.0, 1700);

    // CarMoveTo( 1320, -40 , 270, 2500);

    // CarMoveTo( 1320 -  400 , 0 , 0, 2500);

    Move_To_Position_XYZ(1320 - 400, 0, 0, 2500);

    Move_To_Position_XYZ(0, -350, 0, 2000);

    Send_and_Wait_for_Cmd(2, 0, 2, 1);

    HAL_Delay(50);

    Car_Blog(3000, 1);

    Send_and_Wait_for_Cmd(2, 2, 2, 3);

    Car_Ring(5000);

    Send_and_Wait_for_Cmd(2, 4, 2, 5);
}

void Plate_to_Precise(uint8_t Rounds)
{
    //  Move_To_Position_XYZ(-425, 0, 270, 2000);

    // Move_To_Position_XYZ(0, 750, 270, 2000);//容易踩到边上

    Move_To_Position_XYZ(-50, 325 + 800, 0, 2000);

    Delta_Position_XY delta_position = Coulor_feedback(GREEN);

    HAL_Delay(100);

    Turn_To(90, 1.0, 1700);

    //////这里进行了方向和位置的转变

    delta_position = Coulor_feedback(raspi_date->order2[0]);
    HAL_Delay(100);
    Move_To_Position_XYZ(delta_position.delta_position_x, delta_position.delta_position_y, 90, 2000);

    /// 等待并且进行car_blog
    Send_and_Wait_for_Cmd(1, 0, 1, 1);
    Car_Blog(1000, 1);
    Send_and_Wait_for_Cmd(1, 2, 1, 3);
    Car_Ring(5000);
    Send_and_Wait_for_Cmd(1, 4, 1, 5);

    delta_position = Coulor_feedback(raspi_date->order2[1]);
    HAL_Delay(100);
    Move_To_Position_XYZ(delta_position.delta_position_x, delta_position.delta_position_y, 90, 2000);

    /// 等待并且进行car_blog
    Send_and_Wait_for_Cmd(1, 6, 1, 7);

    delta_position = Coulor_feedback(raspi_date->order2[2]);
    HAL_Delay(100);
    Move_To_Position_XYZ(delta_position.delta_position_x, delta_position.delta_position_y, 90, 2000);

    /// 等待并且进行car_blog
    Send_and_Wait_for_Cmd(1, 8, 1, 9);
}

void Precise_to_Plate(uint8_t Rounds)
{
    Delta_Position_XY delta_position = Coulor_feedback(GREEN);
    HAL_Delay(100);
    Move_To_Position_XYZ(delta_position.delta_position_x, -60 + delta_position.delta_position_y, 90, 1000);

    Turn_To(0, 1.0, 1700);

    Move_To_Position_XYZ(0, -1100, 0, 2000);

    Send_and_Wait_for_Cmd(2, 0, 2, 1);

    Car_Blog(3000, 1);

    Send_and_Wait_for_Cmd(2, 2, 2, 3);

    Car_Ring(5000);

    Send_and_Wait_for_Cmd(2, 4, 2, 5);

    Move_To_Position_XYZ(0, -500, 0, 2000);

    Move_To_Position_XYZ(750, -250, 0, 3000);

    Move_To_Position_XYZ(250, -250, 0, 3000);
}

void run_function(void)
{
    Move_To_Position_XYZ(-190, 540, 0, 2000);
    /////可能需要套等待
    Move_To_Position_XYZ(0, 510, 0, 2000);

    Turn_To(180, 1.0, 1700);

    CarMoveTo(-1620, 0, 180, 3000); ////这个数值需要测量

    Turn_To(270, 1.0, 1700);

    CarMoveTo(1230, 0, 270, 2500);

    Move_To_Position_XYZ(-350, 0, 270, 2000);

    Move_To_Position_XYZ(0, 750, 270, 2000); // 容易踩到边上

    Turn_To(90, 1.0, 1700);

    Turn_To(0, 1.0, 1700);

    Move_To_Position_XYZ(0, -1100, 0, 2000);

    Move_To_Position_XYZ(0, -500, 0, 2000);

    Move_To_Position_XYZ(1000, -250, 0, 3000);
}

