/*
 * Motor_encoder.h
 *
 *  Created on: 2025年3月9日
 *      Author: Monst
 */
#include "zf_common_headfile.h"
#include "Motor_encoder.h"

/*
使用示例
1.调用EncoderInit()函数初始化编码器。
2.在主循环中调用Encoder()函数获取编码器计数，并通过Mf结构体访问Right_encoder和Left_encoder的值。
注意事项
- 确保在调用Encoder()之前已经正确初始化编码器。
- 根据实际需求调整编码器的使用方式，例如是否需要清空计数等。
*/
MotorInf Mf;
int encodertest=0;
void Encoder()
{
    float temp1,temp2;
    temp1=encoder_get_count(ENCODER_2);
    Mf.Right_encoder = temp1 ;                             // 获取编码器计数
    encoder_clear_count(ENCODER_2);                                             // 清空编码器计数


    temp2=encoder_get_count(ENCODER_1);
    Mf.Left_encoder = -temp2 ;                               // 获取编码器计数
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数
    encodertest=(temp1+temp2)/2;
}

void EncoderInit()
{
    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);
}
