/*============================================================================
    File Name     : can_comm.h
    Description   : CAN 通信模块 - 用于电机控制系统的 CAN 总线通信
    Author        : ZHOUHENG
    Date          : 2025-11-27
    ----------------------------------------------------------------------
    功能说明：
    1. 发送电机状态数据（电流、速度、角度等）
    2. 接收控制指令（启动/停止、速度设定、参数配置等）
    3. 与 UART/VOFA+ 系统并行工作，互不干扰

    CAN ID 分配：
    - 0x100-0x1FF: 电机状态数据（发送）
    - 0x200-0x2FF: 控制指令（接收）
    - 0x300-0x3FF: 参数配置（接收）
*=============================================================================
*/

#ifndef CAN_COMM_H
#define CAN_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
   CAN ID 定义 - 数据字典
   ============================================================================ */

/* 状态数据发送 ID（0x100-0x1FF）*/
#define CAN_ID_MOTOR_CURRENT        0x101  // 三相电流（Ia, Ib）
#define CAN_ID_MOTOR_VOLTAGE_ANGLE  0x102  // 母线电压、电角度
#define CAN_ID_MOTOR_SPEED_SECTOR   0x103  // 速度、SVPWM 扇区
#define CAN_ID_PWM_DUTY             0x104  // PWM 占空比（Tcm1, Tcm2）
#define CAN_ID_MOTOR_STATUS         0x105  // 电机状态字（运行状态、故障码）
#define CAN_ID_DQ_CURRENT           0x106  // dq 轴电流（Id, Iq）

/* 控制指令接收 ID（0x200-0x2FF）*/
#define CAN_ID_CMD_MOTOR_ENABLE     0x201  // 电机使能/失能
#define CAN_ID_CMD_SPEED_SETPOINT   0x202  // 速度设定值
#define CAN_ID_CMD_CURRENT_SETPOINT 0x203  // 电流设定值（Id, Iq）
#define CAN_ID_CMD_EMERGENCY_STOP   0x2FF  // 紧急停止

/* 参数配置接收 ID（0x300-0x3FF）*/
#define CAN_ID_CFG_MOTOR_PARAMS     0x301  // 电机参数配置
#define CAN_ID_CFG_CURRENT_LIMIT    0x302  // 电流限值配置
#define CAN_ID_CFG_SPEED_LIMIT      0x303  // 速度限值配置

/* ============================================================================
   数据结构定义
   ============================================================================ */

/**
 * @brief  CAN 发送数据包结构体
 */
typedef struct {
    float Ia;           // A 相电流 (A)
    float Ib;           // B 相电流 (A)
    float Ic;           // C 相电流 (A)
    float V_bus;        // 母线电压 (V)
    float angle_elec;   // 电角度 (rad)
    float speed_rpm;    // 转速 (RPM)
    uint8_t sector;     // SVPWM 扇区 (0-5)
    uint32_t Tcm1;      // PWM 比较值 1
    uint32_t Tcm2;      // PWM 比较值 2
    uint32_t Tcm3;      // PWM 比较值 3
    float Id;           // d 轴电流 (A)
    float Iq;           // q 轴电流 (A)
    uint8_t motor_state; // 电机状态
    uint16_t fault_code; // 故障码
} CAN_TxData_t;

/**
 * @brief  CAN 接收控制指令结构体
 */
typedef struct {
    bool motor_enable;      // 电机使能标志
    float speed_setpoint;   // 速度设定值 (RPM)
    float Id_setpoint;      // d 轴电流设定值 (A)
    float Iq_setpoint;      // q 轴电流设定值 (A)
    bool emergency_stop;    // 紧急停止标志
} CAN_RxCmd_t;

/**
 * @brief  CAN 通信配置结构体
 */
typedef struct {
    bool tx_enable;         // 发送使能
    uint16_t tx_period_ms;  // 发送周期 (ms)
    bool rx_enable;         // 接收使能
    uint8_t motor_id;       // 电机 ID（用于多电机系统）
} CAN_Config_t;

/* ============================================================================
   函数声明
   ============================================================================ */

/**
 * @brief  CAN 通信模块初始化
 * @param  config: 配置参数指针
 * @retval None
 */
void CAN_Comm_Init(CAN_Config_t *config);

/**
 * @brief  CAN 发送任务（周期性调用）
 * @note   建议在低优先级线程中调用，周期 10-100ms
 * @retval None
 */
void CAN_Comm_TxTask(void);

/**
 * @brief  更新 CAN 发送数据（从全局变量采集）
 * @param  tx_data: 发送数据结构体指针
 * @retval None
 */
void CAN_Comm_UpdateTxData(CAN_TxData_t *tx_data);

/**
 * @brief  发送电机电流数据
 * @param  Ia: A 相电流 (A)
 * @param  Ib: B 相电流 (A)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Comm_SendMotorCurrent(float Ia, float Ib);

/**
 * @brief  发送电机电压和角度
 * @param  V_bus: 母线电压 (V)
 * @param  angle_elec: 电角度 (rad)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Comm_SendVoltageAngle(float V_bus, float angle_elec);

/**
 * @brief  发送电机速度和扇区
 * @param  speed_rpm: 转速 (RPM)
 * @param  sector: SVPWM 扇区 (0-5)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Comm_SendSpeedSector(float speed_rpm, uint8_t sector);

/**
 * @brief  发送 PWM 占空比
 * @param  Tcm1: PWM 比较值 1
 * @param  Tcm2: PWM 比较值 2
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Comm_SendPWMDuty(uint32_t Tcm1, uint32_t Tcm2);

/**
 * @brief  发送电机状态
 * @param  motor_state: 电机状态
 * @param  fault_code: 故障码
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Comm_SendMotorStatus(uint8_t motor_state, uint16_t fault_code);

/**
 * @brief  发送 dq 轴电流
 * @param  Id: d 轴电流 (A)
 * @param  Iq: q 轴电流 (A)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Comm_SendDQCurrent(float Id, float Iq);

/**
 * @brief  处理接收到的 CAN 控制指令
 * @param  rx_header: CAN 接收头
 * @param  rx_data: 接收数据缓冲区
 * @retval None
 * @note   此函数在 HAL_FDCAN_RxFifo0Callback 中调用
 */
void CAN_Comm_ProcessRxCommand(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

/**
 * @brief  获取接收到的控制指令
 * @param  rx_cmd: 接收指令结构体指针
 * @retval None
 */
void CAN_Comm_GetRxCommand(CAN_RxCmd_t *rx_cmd);

/**
 * @brief  使能/失能 CAN 发送
 * @param  enable: true=使能, false=失能
 * @retval None
 */
void CAN_Comm_SetTxEnable(bool enable);

/**
 * @brief  设置 CAN 发送周期
 * @param  period_ms: 发送周期 (ms)
 * @retval None
 */
void CAN_Comm_SetTxPeriod(uint16_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* CAN_COMM_H */
