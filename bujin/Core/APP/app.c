#include "app.h"





void app_init(void)
{
    omega_zero();
    initializePIDController();
    initializeCar();
    //物料盘抓取高度
    ///这三个新的稳差都需要先进行测试，机械中心的位置进行了修改。
    raspi_date->vision_current_position_x[0] = 177.0f; // 前后范围0-480
    raspi_date->vision_current_position_y[0] = 312.0f; // 左右范围0-640
    //抓取物料后抬升高度的视觉中心（粗调高度）
    raspi_date->vision_current_position_x[1] = 210.0f; // 前后范围0-480
    raspi_date->vision_current_position_y[1] = 312.0f; // 左右范围0-640
    //抓取物料精调高度(这个只有机械中心是必要的，那个比例是没有意义的)
    raspi_date->vision_current_position_x[2] = 195.1f; // 前后范围0-480//194.0  194.9
    raspi_date->vision_current_position_y[2] = 311.7f; // 左右范围0-640 314.7   312.2

    //理论上和转盘夹取是一个高度
    raspi_date->vision_current_position_x[3] = 177.0f; // 前后范围0-480
    raspi_date->vision_current_position_y[3] = 312.0f; // 左右范围0-640

    ///决赛的相关数据
    raspi_date->vision_current_position_x[4] = 177.0f; // 前后范围0-480
    raspi_date->vision_current_position_y[4] = 312.0f; // 左右范围0-640
    //抓取物料后抬升高度的视觉中心（粗调高度）
    raspi_date->vision_current_position_x[5] = 210.0f; // 前后范围0-480
    raspi_date->vision_current_position_y[5] = 312.0f; // 左右范围0-640
    //抓取物料精调高度
    raspi_date->vision_current_position_x[6] = 194.9f; // 前后范围0-480//194.0   ///没改动精度之前是194.5
    raspi_date->vision_current_position_y[6] = 312.2f; // 左右范围0-640 314.7

    raspi_date->vision_current_position_x[7] = 177.0f; // 前后范围0-480
    raspi_date->vision_current_position_y[7] = 312.0f; // 左右范围0-640


    /////增加就会靠后，减少就会靠前
    ///增加就会片右，减少就会偏左
}