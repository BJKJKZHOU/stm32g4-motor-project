/*============================================================================
    File Name     : PID_controller.h
    Description   : PID控制器头文件
    Author        : ZHOUHENG-D
    Date          : 2025-11-08
    ----------------------------------------------------------------------       
    pid控制器，电流环和速度环的基础控制器是pi控制。
    控制器需要有抗积分保护功能
    速度环的另一方案是PDFF控制器，调整参数可变为pi控制
    位置环的控制器是PD控制，
            
    电流环参数
    name     HMI代码     单位        描述
    Kip       PXXXX       -       电流环比例系数
    Kii       PXXXX       -       电流环积分系数

    电流环解耦
    q 轴: id_feedback反馈的电流和 ws 电转数(rad/s) 
        ws*( Ld * id_feedback + Flux  )
    d 轴: iq_feedback反馈的电流和 ws 电转数(rad/s) 
        -ws*( Lq * iq_feedback)


    直接电压前馈 V_ff = Ke × ω //低速下禁用，禁用范围待测试
    V_ff：前馈电压
    ω ：  机械转数
    将 V_ff 直接加到电流环PI控制器的输出上
  
    速度环参数
    name     HMI代码     单位        描述
    Kvp       PXXXX       -       速度环比例系数
    Kvi       PXXXX       -       速度环积分系数
    Kvfr      PXXXX       -       速度环前馈
  
    位置环参数 - 待定，暂时不设置位置环
  
    需要原始的KP,KI参数和部分基值共同计算从归一化后的参数

    PID_Controller
        输入：
            float setpoint：设定值
            float feedback：反馈值
            float dt：控制周期 
        输出：
            float output：控制器输出值
        内部处理：
            计算误差：error = setpoint - feedback
            比例项：proportional = kp * error
            积分项：integral += ki * error * dt
            积分限幅：if (integral > integral_limit) { integral = integral_limit; } else if (integral < -integral_limit) { integral = -integral_limit; }
            电流环和速度环暂不使用微分项//微分项：derivative = kd * (error - prev_error) / dt
            计算总输出：output = proportional + integral + derivative
            输出限幅：if (output > output_limit) { output = output_limit; } else if (output < -output_limit) { output = -output_limit; }
            更新prev_error：prev_error = error
            
        PDFF PID模块内部包含此算法，使用相同的参数    
            Kfr_speed：前馈系数
            kp_speed：比例系数
            ki_speed：积分系数
                    output = (setpoint * Kfr_speed)+((setpoint - feedback) * ki_speed/积分算子)-(feedback * kp_speed)
            由公式可知，当 Kfr_speed = kp_speed 时，PDFF控制器退化为PI控制器



*=============================================================================
*/
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    float kp;                  // 比例系数
    float ki;                  // 积分系数
    float kd;                  // 微分系数（电流环和速度环暂不使用）
    float Kfr_speed;           // 速度环前馈系数（0=PID模式，非0=PDFF模式）
    
    float integral_limit;      // 积分限幅
    float output_limit;        // 输出限幅
} PID_Params_t;


typedef struct {
    float integral;            // 积分累积
    float prev_error;          // 上次误差（用于微分项）
} PID_State_t;


/* 设定值:setpoint 反馈值: feedback 控制周期:dt 控制器参数:params 控制器状态:state */
float PID_Controller(float setpoint, float feedback, float dt,
                    PID_Params_t *params, PID_State_t *state);


#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H