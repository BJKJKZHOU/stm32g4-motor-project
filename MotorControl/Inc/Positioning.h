/*============================================================================
    File Name     : Positioning.h
    Description   : 电机位置相关 
    Author        : ZHOUHENG
    Date          : 2025-11-12
    ----------------------------------------------------------------------       
    ## 主要是根据电流信号来判断电机的位置
    1、转子初始位置估计，磁极，相位。
    2、位置观测。
    3、位置传感器的上层模块，暂时没有位置传感器。

    ### 脉冲法(IPD)实现转子初始定位 
    1、在 d 轴发送特定角度的电压脉冲，12个脉冲，每个脉冲角度差为 30 度。
        电压脉冲幅值：
            
        持续时间：
    
    3、记录每个脉冲对应的电流值。
        发一次脉冲后，下一次发脉冲在对角位置。
        顺序      角度       脉冲对标记       采样值
        1          0°         P1+           I1
        2          180°       P1-           I2  
        3          30°        P2+           I3
        4          210°       P2-           I4
        5          60°        P3+           I5
        6          240°       P3-           I6
        7          90°        P4+           I7  
        8          270°       P4-           I8
        9          120°       P5+           I9
        10         300°       P5-           I10
        11         150°       P6+           I11
        12         330°       P6-           I12

    
    4、根据电流值，判断电机的磁极和相位。
        在一对测试中，电流响应峰值大的N极。
        多个方向的测试中，电流响应峰值最大的角度为转子位置
        如有编码器，此角度与编码器角度有差异，需要校准。

*=============================================================================
*/


#ifndef POSITIONING_H
#define POSITIONING_H


#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


 extern const float IPD_PULSE_ANGLES[12];
  

typedef struct {
    float angle_elec;       // 电角度 (度)
    float current_sample;   // 电流采样值
} IPD_Pulse_t;


float IPD_CalculateRotorPosition(IPD_Pulse_t pulses[12]);



#ifdef __cplusplus
}
#endif

#endif /* POSITIONING_H */