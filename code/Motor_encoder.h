/*
 * Motor_encoder.h
 *
 *  Created on: 2025?3?9?
 *      Author: Monst
 */

#ifndef CODE_MOTOR_encoder_H_
#define CODE_MOTOR_encoder_H_



#define ENCODER_1                   (TIM4_ENCODER)   //ÓÒ
#define ENCODER_1_A                 (TIM4_ENCODER_CH1_P02_8)
#define ENCODER_1_B                 (TIM4_ENCODER_CH2_P00_9)

#define ENCODER_2                   (TIM6_ENCODER)  //×ó
#define ENCODER_2_A                 (TIM6_ENCODER_CH1_P20_3)
#define ENCODER_2_B                 (TIM6_ENCODER_CH2_P20_0)
typedef struct{
        int Right_encoder;
        int Left_encoder;
}MotorInf;
extern MotorInf Mf;
void Encoder();
void EncoderInit();


#endif /* CODE_MOTOR_encoder_H_ */
