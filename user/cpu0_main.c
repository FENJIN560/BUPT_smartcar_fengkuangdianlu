/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.4
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    all_init();

//    Set_motor_pwm(800,800);
    // 此处编写用户代码 例如外设初始化代码等
	cpu_wait_event_ready();         // 等待所有核心初始化完毕
	while (TRUE)
	{
        // 此处编写需要循环执行的代码

	    //负压
	    //Set_fuya_pwm(4000);
	    //手机调参
	    //get_data_BT();

	    //图像处理
        if(mt9v03x_finish_flag_1)
        {
            mt9v03x_finish_flag_1 = 0;
            image_binarization(mt9v03x_image_1);
            image_filter(bin_image);
            search_line();
            node_proc(bin_image);
            calc_image_error();
            ips114_displayimage03x((const uint8 *)bin_image, MT9V03X_1_W, MT9V03X_1_H);                       // 显示原始图像
            drawkline();
        }
//        draw_rect_box();
//        ips200_show_float(0,100,image_err,2,1);
//        ips200_show_int(0,80,left_encoder,3);
//        ips200_show_int(40,80,right_encoder,3);
//        ips200_show_float(0,120,yaw_total,3,1);
//        ips200_show_int(0,140,run_flag,1);
//        ips200_show_float(20,140,pid_turn.Kp,2,1);
//        ips200_show_float(60,140,pid_turn.Kd,2,1);
//        ips200_show_float(100,140,pid_turn.Kp2,2,1);
//        ips200_show_float(140,140,pid_turn.Kd2,2,3);
//        ips200_show_int(0,160,threshold_mid,3);
//
//        ips200_show_int(0,180,total_dist,4);
//        ips200_show_int(40,180,l_r_range,3);
        // 此处编写需要循环执行的代码
	}
}

#pragma section all restore
// **************************** 代码区域 ****************************
// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：屏幕不显示
//      如果使用主板测试，主板必须要用电池供电 检查屏幕供电引脚电压
//      检查屏幕是不是插错位置了 检查引脚对应关系
//      如果对应引脚都正确 检查一下是否有引脚波形不对 需要有示波器
//      无法完成波形测试则复制一个GPIO例程将屏幕所有IO初始化为GPIO翻转电平 看看是否受控
// 问题2：显示 reinit 字样
//      检查接线是否正常
//      主板供电是否使用电量充足的电池供电
// 问题2：显示图像杂乱 错位
//      检查摄像头信号线是否有松动
