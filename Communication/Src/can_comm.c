/*============================================================================
    File Name     : can_comm.c
    Description   : CAN 通信模块实现
    Author        : ZHOUHENG
    Date          : 2025-11-27
*=============================================================================
*/

#include "can_comm.h"
#include "fdcan.h"
#include "Current.h"
#include "motor_params.h"
#include <string.h>

/* ============================================================================
   外部变量声明（来自其他模块）
   ============================================================================ */

// 来自 main.c 中断
extern volatile uint32_t g_Tcm1;
extern volatile uint32_t g_Tcm2;
extern volatile uint32_t g_Tcm3;

// 来自 FOC_Loop.c
extern volatile float g_debug_angle;
extern volatile uint8_t sector;

// 来自电流采样模块
extern CurrentSample_t g_CurrentSample;

/* ============================================================================
   内部变量定义
   ============================================================================ */

static CAN_Config_t g_can_config = {
    .tx_enable = false,
    .tx_period_ms = 100,  // 默认 100ms 发送周期
    .rx_enable = true,
    .motor_id = 0
};

static CAN_TxData_t g_can_tx_data = {0};
static CAN_RxCmd_t g_can_rx_cmd = {0};

static uint32_t g_last_tx_tick = 0;

/* ============================================================================
   内部函数声明
   ============================================================================ */

static void CAN_Comm_ProcessMotorEnableCmd(uint8_t *rx_data);
static void CAN_Comm_ProcessSpeedSetpointCmd(uint8_t *rx_data);
static void CAN_Comm_ProcessCurrentSetpointCmd(uint8_t *rx_data);
static void CAN_Comm_ProcessEmergencyStopCmd(uint8_t *rx_data);
static void CAN_Comm_ProcessParamConfigCmd(uint32_t can_id, uint8_t *rx_data);

/* ============================================================================
   公共函数实现
   ============================================================================ */

/**
 * @brief  CAN 通信模块初始化
 */
void CAN_Comm_Init(CAN_Config_t *config)
{
    if (config != NULL)
    {
        memcpy(&g_can_config, config, sizeof(CAN_Config_t));
    }

    // 清空数据结构
    memset(&g_can_tx_data, 0, sizeof(CAN_TxData_t));
    memset(&g_can_rx_cmd, 0, sizeof(CAN_RxCmd_t));

    g_last_tx_tick = HAL_GetTick();
}

/**
 * @brief  CAN 发送任务（周期性调用）
 */
void CAN_Comm_TxTask(void)
{
    if (!g_can_config.tx_enable)
    {
        return;
    }

    uint32_t current_tick = HAL_GetTick();

    // 检查是否到达发送周期
    if (current_tick - g_last_tx_tick < g_can_config.tx_period_ms)
    {
        return;
    }

    g_last_tx_tick = current_tick;

    // 更新发送数据
    CAN_Comm_UpdateTxData(&g_can_tx_data);

    // 发送关键数据帧
    CAN_Comm_SendMotorCurrent(g_can_tx_data.Ia, g_can_tx_data.Ib);
    CAN_Comm_SendVoltageAngle(g_can_tx_data.V_bus, g_can_tx_data.angle_elec);
    CAN_Comm_SendSpeedSector(g_can_tx_data.speed_rpm, g_can_tx_data.sector);
    CAN_Comm_SendPWMDuty(g_can_tx_data.Tcm1, g_can_tx_data.Tcm2);
    CAN_Comm_SendMotorStatus(g_can_tx_data.motor_state, g_can_tx_data.fault_code);
}

/**
 * @brief  更新 CAN 发送数据（从全局变量采集）
 */
void CAN_Comm_UpdateTxData(CAN_TxData_t *tx_data)
{
    if (tx_data == NULL)
    {
        return;
    }

    // 采集电流数据
    tx_data->Ia = g_CurrentSample.current_abc[0];
    tx_data->Ib = g_CurrentSample.current_abc[1];
    tx_data->Ic = g_CurrentSample.current_abc[2];

    // 采集电压和角度
    tx_data->V_bus = motor_params[g_can_config.motor_id].V_DC;
    tx_data->angle_elec = g_debug_angle;

    // 采集速度和扇区
    tx_data->speed_rpm = 0.0f;  // TODO: 从速度估计模块获取
    tx_data->sector = sector;

    // 采集 PWM 占空比
    tx_data->Tcm1 = g_Tcm1;
    tx_data->Tcm2 = g_Tcm2;
    tx_data->Tcm3 = g_Tcm3;

    // 采集 dq 轴电流
    tx_data->Id = 0.0f;  // TODO: 从 FOC 模块获取
    tx_data->Iq = 0.0f;  // TODO: 从 FOC 模块获取

    // 采集电机状态
    tx_data->motor_state = MotorParams_IsMotorEnabled(g_can_config.motor_id) ? 1 : 0;
    tx_data->fault_code = 0;  // TODO: 从故障诊断模块获取
}

/**
 * @brief  发送电机电流数据
 */
HAL_StatusTypeDef CAN_Comm_SendMotorCurrent(float Ia, float Ib)
{
    uint8_t tx_data[8];

    // Ia 电流（float，4 字节）
    memcpy(&tx_data[0], &Ia, 4);

    // Ib 电流（float，4 字节）
    memcpy(&tx_data[4], &Ib, 4);

    return FDCAN_Transmit(CAN_ID_MOTOR_CURRENT, tx_data, 8);
}

/**
 * @brief  发送电机电压和角度
 */
HAL_StatusTypeDef CAN_Comm_SendVoltageAngle(float V_bus, float angle_elec)
{
    uint8_t tx_data[8];

    // 母线电压（float，4 字节）
    memcpy(&tx_data[0], &V_bus, 4);

    // 电角度（float，4 字节）
    memcpy(&tx_data[4], &angle_elec, 4);

    return FDCAN_Transmit(CAN_ID_MOTOR_VOLTAGE_ANGLE, tx_data, 8);
}

/**
 * @brief  发送电机速度和扇区
 */
HAL_StatusTypeDef CAN_Comm_SendSpeedSector(float speed_rpm, uint8_t sector)
{
    uint8_t tx_data[8];

    // 转速（float，4 字节）
    memcpy(&tx_data[0], &speed_rpm, 4);

    // SVPWM 扇区（uint8_t，1 字节）
    tx_data[4] = sector;

    // 填充剩余字节
    memset(&tx_data[5], 0, 3);

    return FDCAN_Transmit(CAN_ID_MOTOR_SPEED_SECTOR, tx_data, 8);
}

/**
 * @brief  发送 PWM 占空比
 */
HAL_StatusTypeDef CAN_Comm_SendPWMDuty(uint32_t Tcm1, uint32_t Tcm2)
{
    uint8_t tx_data[8];

    // Tcm1（uint32_t，4 字节）
    memcpy(&tx_data[0], &Tcm1, 4);

    // Tcm2（uint32_t，4 字节）
    memcpy(&tx_data[4], &Tcm2, 4);

    return FDCAN_Transmit(CAN_ID_PWM_DUTY, tx_data, 8);
}

/**
 * @brief  发送电机状态
 */
HAL_StatusTypeDef CAN_Comm_SendMotorStatus(uint8_t motor_state, uint16_t fault_code)
{
    uint8_t tx_data[8];

    // 电机状态（uint8_t，1 字节）
    tx_data[0] = motor_state;

    // 故障码（uint16_t，2 字节）
    memcpy(&tx_data[1], &fault_code, 2);

    // 填充剩余字节
    memset(&tx_data[3], 0, 5);

    return FDCAN_Transmit(CAN_ID_MOTOR_STATUS, tx_data, 8);
}

/**
 * @brief  发送 dq 轴电流
 */
HAL_StatusTypeDef CAN_Comm_SendDQCurrent(float Id, float Iq)
{
    uint8_t tx_data[8];

    // Id 电流（float，4 字节）
    memcpy(&tx_data[0], &Id, 4);

    // Iq 电流（float，4 字节）
    memcpy(&tx_data[4], &Iq, 4);

    return FDCAN_Transmit(CAN_ID_DQ_CURRENT, tx_data, 8);
}

/**
 * @brief  处理接收到的 CAN 控制指令
 */
void CAN_Comm_ProcessRxCommand(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    if (!g_can_config.rx_enable || rx_header == NULL || rx_data == NULL)
    {
        return;
    }

    uint32_t can_id = rx_header->Identifier;

    // 根据 CAN ID 分发处理
    switch (can_id)
    {
        case CAN_ID_CMD_MOTOR_ENABLE:
            CAN_Comm_ProcessMotorEnableCmd(rx_data);
            break;

        case CAN_ID_CMD_SPEED_SETPOINT:
            CAN_Comm_ProcessSpeedSetpointCmd(rx_data);
            break;

        case CAN_ID_CMD_CURRENT_SETPOINT:
            CAN_Comm_ProcessCurrentSetpointCmd(rx_data);
            break;

        case CAN_ID_CMD_EMERGENCY_STOP:
            CAN_Comm_ProcessEmergencyStopCmd(rx_data);
            break;

        case CAN_ID_CFG_MOTOR_PARAMS:
        case CAN_ID_CFG_CURRENT_LIMIT:
        case CAN_ID_CFG_SPEED_LIMIT:
            CAN_Comm_ProcessParamConfigCmd(can_id, rx_data);
            break;

        default:
            // 未知 CAN ID，忽略
            break;
    }
}

/**
 * @brief  获取接收到的控制指令
 */
void CAN_Comm_GetRxCommand(CAN_RxCmd_t *rx_cmd)
{
    if (rx_cmd != NULL)
    {
        memcpy(rx_cmd, &g_can_rx_cmd, sizeof(CAN_RxCmd_t));
    }
}

/**
 * @brief  使能/失能 CAN 发送
 */
void CAN_Comm_SetTxEnable(bool enable)
{
    g_can_config.tx_enable = enable;
}

/**
 * @brief  设置 CAN 发送周期
 */
void CAN_Comm_SetTxPeriod(uint16_t period_ms)
{
    g_can_config.tx_period_ms = period_ms;
}

/* ============================================================================
   内部函数实现（控制指令处理）
   ============================================================================ */

/**
 * @brief  处理电机使能/失能指令
 * @note   数据格式：[motor_id(1B)][enable(1B)][reserved(6B)]
 */
static void CAN_Comm_ProcessMotorEnableCmd(uint8_t *rx_data)
{
    uint8_t motor_id = rx_data[0];
    uint8_t enable = rx_data[1];

    // 检查电机 ID 是否有效
    if (motor_id >= motors_number)
    {
        return;
    }

    // 更新接收指令缓存
    g_can_rx_cmd.motor_enable = (enable != 0);

    // 执行电机使能/失能
    if (enable)
    {
        MotorParams_SetActiveMotor(motor_id);
    }
    else
    {
        MotorParams_DisableMotor(motor_id);
    }
}

/**
 * @brief  处理速度设定值指令
 * @note   数据格式：[speed_setpoint(4B float)][reserved(4B)]
 */
static void CAN_Comm_ProcessSpeedSetpointCmd(uint8_t *rx_data)
{
    float speed_setpoint;
    memcpy(&speed_setpoint, rx_data, 4);

    // 更新接收指令缓存
    g_can_rx_cmd.speed_setpoint = speed_setpoint;

    // TODO: 将速度设定值传递给速度环控制器
    // SpeedLoop_SetSetpoint(speed_setpoint);
}

/**
 * @brief  处理电流设定值指令
 * @note   数据格式：[Id_setpoint(4B float)][Iq_setpoint(4B float)]
 */
static void CAN_Comm_ProcessCurrentSetpointCmd(uint8_t *rx_data)
{
    float Id_setpoint, Iq_setpoint;
    memcpy(&Id_setpoint, &rx_data[0], 4);
    memcpy(&Iq_setpoint, &rx_data[4], 4);

    // 更新接收指令缓存
    g_can_rx_cmd.Id_setpoint = Id_setpoint;
    g_can_rx_cmd.Iq_setpoint = Iq_setpoint;

    // TODO: 将电流设定值传递给电流环控制器
    // CurrentLoop_SetSetpoint(Id_setpoint, Iq_setpoint);
}

/**
 * @brief  处理紧急停止指令
 * @note   数据格式：[stop_flag(1B)][reserved(7B)]
 */
static void CAN_Comm_ProcessEmergencyStopCmd(uint8_t *rx_data)
{
    uint8_t stop_flag = rx_data[0];

    // 更新接收指令缓存
    g_can_rx_cmd.emergency_stop = (stop_flag != 0);

    // 执行紧急停止
    if (stop_flag)
    {
        // 失能所有电机
        for (uint8_t i = 0; i < motors_number; i++)
        {
            MotorParams_DisableMotor(i);
        }

        // TODO: 关闭 PWM 输出
        // HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        // HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        // HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    }
}

/**
 * @brief  处理参数配置指令
 * @note   数据格式：[motor_id(1B)][param_id(1B)][value(4B float)][reserved(2B)]
 */
static void CAN_Comm_ProcessParamConfigCmd(uint32_t can_id, uint8_t *rx_data)
{
    uint8_t motor_id = rx_data[0];
    uint8_t param_id = rx_data[1];
    float value;
    memcpy(&value, &rx_data[2], 4);

    // 检查电机 ID 是否有效
    if (motor_id >= motors_number)
    {
        return;
    }

    // 根据 CAN ID 和参数 ID 配置参数
    switch (can_id)
    {
        case CAN_ID_CFG_MOTOR_PARAMS:
            // TODO: 根据 param_id 设置电机参数
            // 示例：MotorParams_SetParam(motor_id, "V_DC", value);
            break;

        case CAN_ID_CFG_CURRENT_LIMIT:
            if (param_id == 0)
            {
                MotorParams_SetLimitParam(motor_id, "I_limit_user", value);
            }
            break;

        case CAN_ID_CFG_SPEED_LIMIT:
            if (param_id == 0)
            {
                MotorParams_SetLimitParam(motor_id, "speed_limit_user", value);
            }
            break;

        default:
            break;
    }
}
