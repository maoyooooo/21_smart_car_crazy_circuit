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
* 文件名称          zf_device_mt9v03x_double
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
* 2023-04-28       pudding            增加中文注释说明
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                  ------------------------------------
*                  模块管脚             单片机管脚
*                  SCL                查看 zf_device_mt9v03x_double.h 中 MT9V03X_COF_IIC_SCL 宏定义
*                  SDA                查看 zf_device_mt9v03x_double.h 中 MT9V03X_COF_IIC_SDA 宏定义
*                  PCLK               查看 zf_device_mt9v03x_double.h 中 MT9V03X_PCLK_PIN 宏定义
*                  VSY                查看 zf_device_mt9v03x_double.h 中 MT9V03X_VSYNC_PIN 宏定义
*                  D0-D7              查看 zf_device_mt9v03x_double.h 中 MT9V03X_DATA_PIN 宏定义 从该定义开始的连续八个引脚
*                  VCC                3.3V电源
*                  GND                电源地
*                  其余引脚悬空
*                  ------------------------------------
********************************************************************************************************************/
#include "IfxStm.h"
#include "zf_common_interrupt.h"
#include "zf_common_debug.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_delay.h"
#include "zf_driver_dma.h"
#include "zf_driver_exti.h"
#include "zf_common_fifo.h"
#include "zf_driver_gpio.h"
#include "zf_driver_uart.h"
#include "zf_driver_timer.h"
#include "zf_device_camera.h"
#include "zf_device_config.h"
#include "zf_device_mt9v03x_double.h"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

            vuint8      mt9v03x_finish_flag_1;                          // 总钻风摄像头1 单幅图像采集完成标志位
IFX_ALIGN(4) uint8      mt9v03x_image_1[MT9V03X_1_H][MT9V03X_1_W];      // 总钻风摄像头1 图像数据存储数组
            uint32      mt9v03x_fps_1;                                // 总钻风摄像头实际采集帧率


int16   time_out_1 = MT9V03X_1_INIT_TIMEOUT;                            // 定义超时溢出时长


uint8   mt9v03x_lost_flag_1 = 1;                              
uint8   mt9v03x_dma_int_num_1;                                
uint8   mt9v03x_dma_init_flag_1;                              
uint8   mt9v03x_link_list_num_1;                                        // 当前DMA链表数量


// mt9v03x_double_init_type_enum camera_work_type;                          // 当前摄像头工作类型
/*
soft_iic_info_struct mt9v03x_iic_struct_1;
soft_iic_info_struct mt9v03x_iic_struct_2;

uint8   mt9v03x_dma_state[2];
uint8   mt9v03x_gather_flag;
uint32  mt9v03x_time[2];
*/

// 需要配置到摄像头的数据 不允许在这修改参数
 int16 mt9v03x_set_confing_buffer_1[MT9V03X_DOUBLE_CONFIG_FINISH][2]=
{
    {MT9V03X_DOUBLE_INIT,              0},                              // 摄像头开始初始化
    {MT9V03X_DOUBLE_AUTO_EXP,          MT9V03X_1_AUTO_EXP_DEF},         // 自动曝光设置   范围1-63 0为关闭 如果自动曝光开启  EXP_TIME命令设置的数据将会变为最大曝光时间，也就是自动曝光时间的上限
    {MT9V03X_DOUBLE_EXP_TIME,          MT9V03X_1_EXP_TIME_DEF},         // 曝光时间      摄像头收到后会自动计算出最大曝光时间，如果设置过大则设置为计算出来的最大曝光值
    {MT9V03X_DOUBLE_FPS,               MT9V03X_1_FPS_DEF},              // 图像帧率      摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS
    {MT9V03X_DOUBLE_SET_COL,           MT9V03X_1_W},                    // 图像列数量    范围1-752
    {MT9V03X_DOUBLE_SET_ROW,           MT9V03X_1_H},                    // 图像行数量    范围1-480
    {MT9V03X_DOUBLE_LR_OFFSET,         MT9V03X_1_LR_OFFSET_DEF},        // 图像左右偏移量  正值 右偏移   负值 左偏移  列为188 376 752时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
    {MT9V03X_DOUBLE_UD_OFFSET,         MT9V03X_1_UD_OFFSET_DEF},        // 图像上下偏移量  正值 上偏移   负值 下偏移  行为120 240 480时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
    {MT9V03X_DOUBLE_GAIN,              MT9V03X_1_GAIN_DEF},             // 图像增益      范围16-64     增益可以在曝光时间固定的情况下改变图像亮暗程度
    {MT9V03X_DOUBLE_PCLK_MODE,         MT9V03X_1_PCLK_MODE_DEF},        // 像素时钟模式   仅总钻风MT9V034 V2.0以及以上版本支持该命令
};
/*
 int16 mt9v03x_set_confing_buffer_2[MT9V03X_DOUBLE_CONFIG_FINISH][2]=
{
    {MT9V03X_DOUBLE_INIT,              0},                              // 摄像头开始初始化
    {MT9V03X_DOUBLE_AUTO_EXP,          MT9V03X_2_AUTO_EXP_DEF},         // 自动曝光设置   范围1-63 0为关闭 如果自动曝光开启  EXP_TIME命令设置的数据将会变为最大曝光时间，也就是自动曝光时间的上限
    {MT9V03X_DOUBLE_EXP_TIME,          MT9V03X_2_EXP_TIME_DEF},         // 曝光时间      摄像头收到后会自动计算出最大曝光时间，如果设置过大则设置为计算出来的最大曝光值
    {MT9V03X_DOUBLE_FPS,               MT9V03X_2_FPS_DEF},              // 图像帧率      摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS
    {MT9V03X_DOUBLE_SET_COL,           MT9V03X_2_W},                    // 图像列数量    范围1-752
    {MT9V03X_DOUBLE_SET_ROW,           MT9V03X_2_H},                    // 图像行数量    范围1-480
    {MT9V03X_DOUBLE_LR_OFFSET,         MT9V03X_2_LR_OFFSET_DEF},        // 图像左右偏移量  正值 右偏移   负值 左偏移  列为188 376 752时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
    {MT9V03X_DOUBLE_UD_OFFSET,         MT9V03X_2_UD_OFFSET_DEF},        // 图像上下偏移量  正值 上偏移   负值 下偏移  行为120 240 480时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
    {MT9V03X_DOUBLE_GAIN,              MT9V03X_2_GAIN_DEF},             // 图像增益      范围16-64     增益可以在曝光时间固定的情况下改变图像亮暗程度
    {MT9V03X_DOUBLE_PCLK_MODE,         MT9V03X_2_PCLK_MODE_DEF},        // 像素时钟模式   仅总钻风MT9V034 V2.0以及以上版本支持该命令
};
*/
#pragma section all restore

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     配置摄像头内部配置信息 (UART 方式)
//-------------------------------------------------------------------------------------------------------------------
static uint8 mt9v03x_set_config (int16 buff[MT9V03X_DOUBLE_CONFIG_FINISH][2])
{
    uint8  return_state = 1;
    uint8  uart_buffer[4];
    uint16 temp = 0;
    uint16 timeout_count = 0;
    uint32 loop_count = MT9V03X_DOUBLE_GAIN;
    uint32 uart_buffer_index = 0;

    for(; MT9V03X_DOUBLE_SET_DATA > loop_count; loop_count --)
    {
        uart_buffer[0] = 0xA5;
        uart_buffer[1] = (uint8)buff[loop_count][0];
        temp = buff[loop_count][1];
        uart_buffer[2] = temp >> 8;
        uart_buffer[3] = (uint8)temp;
        mt9v03x_soft_uart_send_buff(uart_buffer, 4);
        system_delay_ms(2);
    }

    do
    {
        if(3 <= fifo_used(&camera_receiver_fifo))
        {
            uart_buffer_index = 3;
            fifo_read_buffer(&camera_receiver_fifo, uart_buffer, &uart_buffer_index, FIFO_READ_AND_CLEAN);
            if((0xff == uart_buffer[1]) || (0xff == uart_buffer[2]))
            {
                return_state = 0;
                break;
            }
        }
        system_delay_ms(1);
    }while(MT9V03X_1_INIT_TIMEOUT > timeout_count ++);
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取摄像头固件版本 (验证串口连通性)
//-------------------------------------------------------------------------------------------------------------------
uint16 mt9v03x_get_version (void)
{
    uint16 temp;
    uint8  uart_buffer[4];
    uint16 timeout_count = 0;
    uint16 return_value = 0;
    uint32 uart_buffer_index = 0;

    uart_buffer[0] = 0xA5;
    uart_buffer[1] = MT9V03X_DOUBLE_GET_STATUS;
    temp = MT9V03X_DOUBLE_GET_VERSION;
    uart_buffer[2] = temp >> 8;
    uart_buffer[3] = (uint8)temp;
    mt9v03x_soft_uart_send_buff(uart_buffer, 4);

    do
    {
        if(3 <= fifo_used(&camera_receiver_fifo))
        {
            uart_buffer_index = 3;
            fifo_read_buffer(&camera_receiver_fifo, uart_buffer, &uart_buffer_index, FIFO_READ_AND_CLEAN);
            return_value = uart_buffer[1] << 8 | uart_buffer[2];
            break;
        }
        system_delay_ms(1);
    }while(MT9V03X_1_INIT_TIMEOUT > timeout_count ++);
    return return_value;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      MT9V03X摄像头串口通信回调
//-------------------------------------------------------------------------------------------------------------------
static void mt9v03x_uart_handler (void)
{
    uint8 data = 0;
    uart_query_byte(MT9V03X_COF_UART, &data);
    if(0xA5 == data)
    {
        fifo_clear(&camera_receiver_fifo);
    }
    fifo_write_element(&camera_receiver_fifo, data);
}

/*
//-------------------------------------------------------------------------------------------------------------------
//  函数简介      MT9V03X 摄像头1 DMA 重置
//  参数说明      void
//  返回参数      void
//  使用示例      mt9v03x_dma_restart_1();
//-------------------------------------------------------------------------------------------------------------------
static void mt9v03x_dma_restart_1(void)
{
    dma_disable(MT9V03X_1_DMA_CH);
    IfxDma_resetChannel(&MODULE_DMA, MT9V03X_1_DMA_CH);
    dma_init(MT9V03X_1_DMA_CH,
             MT9V03X_1_DATA_ADD,
             mt9v03x_image_1[0],
             MT9V03X_1_PCLK_PIN,
             EXTI_TRIGGER_FALLING,
             MT9V03X_1_IMAGE_SIZE);           // 如果超频到300M 倒数第二个参数请设置为FALLING
    dma_enable(MT9V03X_1_DMA_CH);
}
*/



/*
//-------------------------------------------------------------------------------------------------------------------
//  函数简介      MT9V03X 摄像头2 DMA 重置
//  参数说明      void
//  返回参数      void
//  使用示例      mt9v03x_dma_restart_1();
//-------------------------------------------------------------------------------------------------------------------
static void mt9v03x_dma_restart_2(void)
{
    dma_disable(MT9V03X_2_DMA_CH);
    IfxDma_resetChannel(&MODULE_DMA, MT9V03X_2_DMA_CH);
    dma_init_2(MT9V03X_2_DMA_CH,
               MT9V03X_2_DATA_ADD,
               mt9v03x_image_2[0],
               MT9V03X_2_PCLK_PIN,
               EXTI_TRIGGER_FALLING,
               MT9V03X_2_IMAGE_SIZE);           // 如果超频到300M 倒数第二个参数请设置为FALLING
    dma_enable(MT9V03X_2_DMA_CH);
}
*/



/*
//-------------------------------------------------------------------------------------------------------------------
//  函数简介      MT9V03X摄像头场中断
//  参数说明      void
//  返回参数      void
//  使用示例      mt9v03x_vsync_handler();
//-------------------------------------------------------------------------------------------------------------------
static void mt9v03x_vsync_handler_1(void)
{
    Ifx_STM *module_num;
    uint32 temp_time = 0;
    static uint32 mt9v03x_fps_count = 0;

    if(gpio_get_level(P33_7))
    {
        if(camera_work_type == mt9v03x_double && mt9v03x_dma_state[0] && gpio_get_level(P10_3) == 0)
        {
            mt9v03x_dma_state[0] = 0;
            mt9v03x_dma_restart_2();

            mt9v03x_gather_flag = 2;
        }
        else if(mt9v03x_gather_flag == 0)
        {
            mt9v03x_dma_restart_1();

            mt9v03x_gather_flag = 1;
        }
    }
    else
    {
        if(mt9v03x_gather_flag == 1)
        {
            // 采集完成
            // 一副图像从采集开始到采集结束耗时3.8MS左右(50FPS、188*120分辨率)
            dma_disable(MT9V03X_1_DMA_CH);
            if(camera_work_type == mt9v03x_double)
            {
                if(gpio_get_level(P10_3))
                {
                    mt9v03x_dma_state[0] = 1;
                }
                else
                {
                    mt9v03x_dma_restart_2();
                    mt9v03x_gather_flag = 2;
                }
            }
            else
            {
                mt9v03x_dma_restart_1();
            }

            module_num = IfxStm_getAddress((IfxStm_Index)(IfxCpu_getCoreId()));

            if(mt9v03x_time[0] == 0)
            {
                mt9v03x_time[0] = IfxStm_getLower(module_num);
            }
            else
            {
                temp_time = (uint32)((uint64)(IfxStm_getLower(module_num) - mt9v03x_time[0]) * 1000 / IfxStm_getFrequency(module_num));
                if(temp_time >= 1000)
                {
                    mt9v03x_time[0] = IfxStm_getLower(module_num);
                    mt9v03x_fps[0] = mt9v03x_fps_count - 1;
                    mt9v03x_fps_count = 0;
                }
            }

            mt9v03x_fps_count ++;

            mt9v03x_finish_flag_1 = 1;

        }
    }
}
*/



//-------------------------------------------------------------------------------------------------------------------
//  函数简介      MT9V03X摄像头场中断 (已精简：剥离双摄交替逻辑，专注单摄 DMA)
//-------------------------------------------------------------------------------------------------------------------
static void mt9v03x_vsync_handler_1(void)
{
    exti_flag_clear(MT9V03X_1_VSYNC_PIN);
    mt9v03x_dma_int_num_1 = 0;
    
    if(mt9v03x_dma_init_flag_1)
    {
        mt9v03x_dma_init_flag_1 = 0;
        IfxDma_resetChannel(&MODULE_DMA, MT9V03X_1_DMA_CH);
        mt9v03x_link_list_num_1 = dma_init(MT9V03X_1_DMA_CH,
                                         MT9V03X_1_DATA_ADD,
                                         mt9v03x_image_1[0],
                                         MT9V03X_1_PCLK_PIN,
                                         EXTI_TRIGGER_RISING,
                                         MT9V03X_1_IMAGE_SIZE);
        dma_enable(MT9V03X_1_DMA_CH);
    }
    else
    {
        if(1 == mt9v03x_link_list_num_1)
        {
            dma_set_destination(MT9V03X_1_DMA_CH, mt9v03x_image_1[0]);
        }
        dma_enable(MT9V03X_1_DMA_CH);
    }
    mt9v03x_lost_flag_1 = 1;
}



/*
//-------------------------------------------------------------------------------------------------------------------
//  函数简介      MT9V03X摄像头场中断
//  参数说明      void
//  返回参数      void
//  使用示例      mt9v03x_vsync_handler();
//-------------------------------------------------------------------------------------------------------------------
static void mt9v03x_vsync_handler_2(void)
{
    Ifx_STM *module_num;
    uint32 temp_time = 0;
    static uint32 mt9v03x_fps_count = 0;

    if(gpio_get_level(P10_3) == 1)
    {
        if(camera_work_type == mt9v03x_double && mt9v03x_dma_state[1] && gpio_get_level(P33_7) == 0)
        {
            mt9v03x_dma_state[1] = 0;
            mt9v03x_dma_restart_1();

            mt9v03x_gather_flag = 1;
        }
        else if(mt9v03x_gather_flag == 0)
        {
            mt9v03x_dma_restart_2();
            mt9v03x_gather_flag = 2;
        }
    }
    else
    {
        if(mt9v03x_gather_flag == 2)
        {
            // 采集完成
            // 一副图像从采集开始到采集结束耗时3.8MS左右(50FPS、188*120分辨率)

            dma_disable(MT9V03X_2_DMA_CH);

            if(camera_work_type == mt9v03x_double)
            {
                if(gpio_get_level(P33_7))
                {
                    mt9v03x_dma_state[1] = 1;
                }
                else
                {
                    mt9v03x_dma_restart_1();

                    mt9v03x_gather_flag = 1;
                }
            }
            else
            {
                mt9v03x_dma_restart_2();
            }

            module_num = IfxStm_getAddress((IfxStm_Index)(IfxCpu_getCoreId()));

            if(mt9v03x_time[1] == 0)
            {
                mt9v03x_time[1] = IfxStm_getLower(module_num);
            }
            else
            {
                temp_time = (uint32)((uint64)(IfxStm_getLower(module_num) - mt9v03x_time[1]) * 1000 / IfxStm_getFrequency(module_num));
                if(temp_time >= 1000)
                {
                    mt9v03x_time[1] = IfxStm_getLower(module_num);
                    mt9v03x_fps[1] = mt9v03x_fps_count - 1;
                    mt9v03x_fps_count = 0;
                }
            }

            mt9v03x_fps_count ++;

            mt9v03x_finish_flag_2 = 1;
        }
    }
}
*/



//-------------------------------------------------------------------------------------------------------------------
//  函数简介      MT9V03X摄像头DMA完成中断 (图像采集完毕)
//-------------------------------------------------------------------------------------------------------------------
static void mt9v03x_dma_handler_1(void)
{
    clear_dma_flag(MT9V03X_1_DMA_CH);

    if(IfxDma_getChannelTransactionRequestLost(&MODULE_DMA, MT9V03X_1_DMA_CH))
    {
        mt9v03x_finish_flag_1 = 0;
        dma_disable(MT9V03X_1_DMA_CH);
        IfxDma_clearChannelTransactionRequestLost(&MODULE_DMA, MT9V03X_1_DMA_CH);
        mt9v03x_dma_init_flag_1 = 1;
    }
    else
    {
        mt9v03x_dma_int_num_1++;
        if(mt9v03x_dma_int_num_1 >= mt9v03x_link_list_num_1)
        {
            mt9v03x_dma_int_num_1 = 0;
            mt9v03x_lost_flag_1   = 0;
            mt9v03x_finish_flag_1 = 1;
            dma_disable(MT9V03X_1_DMA_CH);
        }
    }
}



/*
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单独设置曝光时间 (UART 版本)
// 参数说明     light           曝光时间，数越大图像越亮
// 返回参数     uint8           0：配置成功 1：配置失败
//-------------------------------------------------------------------------------------------------------------------
uint8 mt9v03x_set_exposure_time(uint16 light)
{
    uint8 return_state = 0;
    uint8  uart_buffer[4];
    uint16 temp = 0;
    uint16 timeout_count = 0;
    uint32 uart_buffer_index = 0;

    // 清空接收 FIFO 准备接收摄像头反馈
    fifo_clear(&camera_receiver_fifo);

    // 打包命令：设置曝光时间 (0xF0)
    uart_buffer[0] = 0xA5;
    uart_buffer[1] = MT9V03X_DOUBLE_SET_EXP_TIME;
    temp = light;
    uart_buffer[2] = temp >> 8;
    uart_buffer[3] = (uint8)temp;
    
    // 使用模拟串口发送
    mt9v03x_soft_uart_send_buff(uart_buffer, 4);

    // 等待摄像头回复以确认设置成功
    do
    {
        if(3 <= fifo_used(&camera_receiver_fifo))
        {
            uart_buffer_index = 3;
            fifo_read_buffer(&camera_receiver_fifo, uart_buffer, &uart_buffer_index, FIFO_READ_AND_CLEAN);
            temp = uart_buffer[1] << 8 | uart_buffer[2];
            break;
        }
        system_delay_ms(1);
    }while(MT9V03X_1_INIT_TIMEOUT > timeout_count ++);
    
    if((temp != light) || (MT9V03X_1_INIT_TIMEOUT <= timeout_count))
    {
        return_state = 1; // 配置失败或超时
    }
    
    return return_state;
}
*/




/*
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     mt9v03x单独设置曝光时间
// 参数说明     init_type       选择要设置的摄像头
// 参数说明     light           曝光时间，没有单位，数越大图像越亮
// 返回参数     unsigned char           0：配置成功 1：配置失败
// 使用示例     mt9v03x_set_exposure_time_sccb(mt9v03x_1,100);
//-------------------------------------------------------------------------------------------------------------------

uint8 mt9v03x_set_exposure_time_sccb(m9v03x_double_init_type_enum init_type,unsigned short int light)
{

    uint8 return_state = 0;
    switch(init_type)
    {
        case mt9v03x_1:
        {
            mt9v03x_set_exposure_time_sccb_1(light);
            return_state = 1;
        }break;
        case mt9v03x_2:
        {
            mt9v03x_set_exposure_time_sccb_2(light);
            return_state = 1;

        }break;
        case mt9v03x_double:
        {
            mt9v03x_set_exposure_time_sccb_1(light);
            mt9v03x_set_exposure_time_sccb_2(light);
            return_state = 1;
        }break;
        default:break;
    }

    return return_state;
    }
*/




/*
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MT9V03X 双摄像头初始化
// 参数说明     init_type       初始化方式
// 返回参数     uint8           1-失败 0-成功
// 使用示例     zf_log(mt9v03x_double_init(mt9v03x_1), "mt9v03x init error");
// 备注信息     注意:双摄版本不再兼容串口版本摄像头 修改摄像头初始化通讯类型详见摄像头资料说明
//-------------------------------------------------------------------------------------------------------------------
uint8 mt9v03x_double_init(m9v03x_double_init_type_enum init_type)
{
    //-------------------该函数仅用于TC系列 V3.0 双摄主板 摄像头初始化----------------------
    //-------------------该函数仅用于TC系列 V3.0 双摄主板 摄像头初始化----------------------
    //-------------------该函数仅用于TC系列 V3.0 双摄主板 摄像头初始化----------------------
    uint8 return_state = 0;


    uint32 interrupt_state = interrupt_global_disable();     // 关闭全局中断

    system_delay_ms(200);

    camera_work_type = init_type;

    switch(init_type)
    {
        case mt9v03x_1:
        {
            do
            {
                // 使用SCCB通讯
                set_camera_type(CAMERA_GRAYSCALE_1, mt9v03x_vsync_handler_1, NULL, NULL);

                soft_iic_init(&mt9v03x_iic_struct_1, 0, MT9V03X_1_COF_IIC_DELAY, MT9V03X_1_COF_IIC_SCL, MT9V03X_1_COF_IIC_SDA);
//                mt9v03x_sccb_check_id();
                if(mt9v03x_set_config_sccb_1(&mt9v03x_iic_struct_1,mt9v03x_set_confing_buffer_1))
                {
                    // SCCB通讯失败
                    zf_log(0, "MT9V03X 1 set sccb error.");
                    return_state = 1;
                    break;
                }

                mt9v03x_link_list_num_1 = camera_init(MT9V03X_1_DATA_ADD, mt9v03x_image_1[0], MT9V03X_1_IMAGE_SIZE);

                IfxDma_disableChannelInterrupt(&MODULE_DMA, MT9V03X_1_DMA_CH);
            }while(0);
        }break;

        case mt9v03x_2:
        {
            do
            {
                // 使用SCCB通讯
                set_camera_type(CAMERA_GRAYSCALE_2, mt9v03x_vsync_handler_2, NULL, NULL);

                soft_iic_init(&mt9v03x_iic_struct_2, 0, MT9V03X_2_COF_IIC_DELAY, MT9V03X_2_COF_IIC_SCL, MT9V03X_2_COF_IIC_SDA);
                if(mt9v03x_set_config_sccb_2(&mt9v03x_iic_struct_2,mt9v03x_set_confing_buffer_1))
                {
                    // SCCB通讯失败
                    zf_log(0, "MT9V03X 2 set sccb error.");
                    return_state = 1;
                    break;
                }

                mt9v03x_link_list_num_2 = camera_init(MT9V03X_2_DATA_ADD, mt9v03x_image_2[0], MT9V03X_2_IMAGE_SIZE);

                IfxDma_disableChannelInterrupt(&MODULE_DMA, MT9V03X_2_DMA_CH);
            }while(0);

        }break;

        case mt9v03x_double:
        {
            do
            {
                // 使用SCCB通讯
                set_camera_type(CAMERA_GRAYSCALE_1, mt9v03x_vsync_handler_1, NULL, NULL);

                soft_iic_init(&mt9v03x_iic_struct_1, 0, MT9V03X_1_COF_IIC_DELAY, MT9V03X_1_COF_IIC_SCL, MT9V03X_1_COF_IIC_SDA);
                if(mt9v03x_set_config_sccb_1(&mt9v03x_iic_struct_1,mt9v03x_set_confing_buffer_1))
                {
                    // SCCB通讯失败
                    zf_log(0, "MT9V03X 1 set sccb error.");
                    return_state = 1;
                    break;
                }

                mt9v03x_link_list_num_1 = camera_init(MT9V03X_1_DATA_ADD, mt9v03x_image_1[0], MT9V03X_1_IMAGE_SIZE);

                IfxDma_disableChannelInterrupt(&MODULE_DMA, MT9V03X_1_DMA_CH);

                system_delay_ms(MT9V03X_INIT_INTV_DELAY);                                                    //修改这个延时可以增加高帧率的运行时长

                // 使用SCCB通讯
                set_camera_type(CAMERA_GRAYSCALE_2, mt9v03x_vsync_handler_2, NULL, NULL);

                soft_iic_init(&mt9v03x_iic_struct_2, 0, MT9V03X_2_COF_IIC_DELAY, MT9V03X_2_COF_IIC_SCL, MT9V03X_2_COF_IIC_SDA);

                if(mt9v03x_set_config_sccb_2(&mt9v03x_iic_struct_2,mt9v03x_set_confing_buffer_1))
                {
                    // SCCB通讯失败
                    zf_log(0, "MT9V03X 2 set sccb error.");
                    return_state = 1;
                    break;
                }

                mt9v03x_link_list_num_2 = camera_init(MT9V03X_2_DATA_ADD, mt9v03x_image_2[0], MT9V03X_2_IMAGE_SIZE);

                IfxDma_disableChannelInterrupt(&MODULE_DMA, MT9V03X_2_DMA_CH);
            }while(0);
        }break;

        default:break;
    }

    interrupt_global_enable(interrupt_state);     // 打开全局中断

    return return_state;
}
*/



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MT9V03X 摄像头初始化 (串口定制版)
// 返回参数     uint8           1-失败 0-成功
//-------------------------------------------------------------------------------------------------------------------
uint8 mt9v03x_init(void)
{
    uint8 return_state = 0;
    uint32 interrupt_state = interrupt_global_disable();
    gpio_init(MT9V03X_SOFT_RX_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    system_delay_ms(300); // 等待摄像头上电稳定

    // 1. 设置回调函数 (传入单摄的 Vsync, DMA, UART 接收处理函数)
    set_camera_type(CAMERA_GRAYSCALE_1, mt9v03x_vsync_handler_1, mt9v03x_dma_handler_1, mt9v03x_uart_handler);
    camera_fifo_init();

    // 2. 初始化串口
    uart_init(MT9V03X_COF_UART, MT9V03X_COF_BAUR, UART1_TX_P33_12, MT9V03X_COF_UART_RX);
    uart_rx_interrupt(MT9V03X_COF_UART, 1);
    fifo_clear(&camera_receiver_fifo);    // 3. 握手与配置
    if(mt9v03x_get_version() == 0)
    {
        zf_log(0, "MT9V03X UART Handshake Failed! Check TX/RX.");
        interrupt_global_enable(interrupt_state);
        return 1;
    }

    if(mt9v03x_set_config(mt9v03x_set_confing_buffer_1))
    {
        zf_log(0, "MT9V03X set config error.");
        uart_rx_interrupt(MT9V03X_COF_UART, 0);
        set_camera_type(NO_CAMERE, NULL, NULL, NULL);
        interrupt_global_enable(interrupt_state);
        return 1;
    }

    // 4. 初始化图像采集 DMA
    mt9v03x_link_list_num_1 = camera_init(MT9V03X_1_DATA_ADD, mt9v03x_image_1[0], MT9V03X_1_IMAGE_SIZE);

    interrupt_global_enable(interrupt_state);
    return return_state;
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件模拟串口发送一个字节 (Bit-Banging)
// 参数说明     data ：待发送的数据
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_soft_uart_tx(uint8 data)
{
    uint8 i;
    // 1. 起始位: 低电平
    gpio_set_level(MT9V03X_SOFT_TX_PIN, 0);
    system_delay_us(104); // 9600波特率 1bit时间

    // 2. 数据位: 低位在前
    for(i = 0; i < 8; i++)
    {
        gpio_set_level(MT9V03X_SOFT_TX_PIN, (data >> i) & 0x01);
        system_delay_us(104);
    }

    // 3. 停止位: 高电平
    gpio_set_level(MT9V03X_SOFT_TX_PIN, 1);
    system_delay_us(104);
}

// 软件发送缓冲区
void mt9v03x_soft_uart_send_buff(uint8 *buff, uint16 len)
{
    while(len--)
    {
        mt9v03x_soft_uart_tx(*buff++);
    }
}