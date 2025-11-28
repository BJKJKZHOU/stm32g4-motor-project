# MotorControl 模块设计文档

## 1. 概述

MotorControl 是为 STM32G4 平台设计的永磁同步电机（PMSM）磁场定向控制（FOC）系统，提供从参数管理、电流采样、位置估计到实时控制的完整解决方案。

### 1.1 核心特性

- **多电机支持**：支持多个独立电机参数套，动态激活/停用
- **归一化系统**：基于硬件能力的标幺值计算，确保数值稳定性
- **电流采样**：双ADC同步采样，动态触发点调整，支持高占空比场景
- **位置估计**：IPD脉冲法转子初始定位 + 非线性磁链观测器无传感器位置估计
- **FOC算法**：完整的Clarke/Park变换、SVPWM调制，集成CORDIC硬件加速
- **统一控制器**：PID/PDFF统一算法，支持电流环和速度环
- **实时命令**：串口命令交互，支持参数查看和修改
- **电流环控制**：完整的闭环电流控制，集成非线性磁链观测器

### 1.2 模块架构

```
┌─────────────────────────────────────────────────────────────┐
│                    MotorControl 系统                        │
├─────────────────────────────────────────────────────────────┤
│  命令层 (command)                                           │
│  串口命令解析 | VOFA数据绘图 | 参数配置                      │
├─────────────────────────────────────────────────────────────┤
│  管理层 (motor_params + normalization)                     │
│  电机参数管理 | 归一化基值计算 | 标幺值转换                  │
├─────────────────────────────────────────────────────────────┤
│  控制层 (FOC_Loop + Current)                               │
│  FOC循环 | 电流采样 | 电流环控制 | 位置估计                  │
├─────────────────────────────────────────────────────────────┤
│  算法层 (FOC_math + PID_controller + Positioning)          │
│  Clarke/Park变换 | SVPWM调制 | PID/PDFF控制器 | 位置观测器   │
└─────────────────────────────────────────────────────────────┘
```

## 2. 模块详细说明

### 2.1 电机参数管理 (motor_params)

**功能**：管理多套电机参数，支持动态激活/停用，提供参数查看和修改接口。

**核心API**：
- `MotorParams_Init()` - 初始化电机参数
- `MotorParams_SetParam()` - 设置参数
- `MotorParams_SetActiveMotor()` / `DisableMotor()` - 激活/停用电机
- `MotorParams_GetActiveMotor()` - 获取当前激活电机ID

**参数包括**：V_DC（母线电压）、I_rated（额定电流）、Rs（定子电阻）、Lq/Ld（电感）、RPM_rated（额定转速）、Pn（极对数）、Ke（反电动势常数）、Flux（磁链）、J（转动惯量）、B（摩擦系数）等。

### 2.2 归一化模块 (normalization)

**功能**：基于硬件能力计算归一化基值，提供物理量与标幺值的双向转换，支持浮点和Q31格式。

**归一化方案**（基于硬件实际能力）：
```
V_base = V_DC/√3,  I_base = I_limit_actual,  ω_base = (2π×RPM/60)×Pn
Flux_base = V_base/ω_base,  T_base = 1.5×Pn×Flux×I_base
```

**核心API**：
- `Normalization_Init()` / `UpdateMotor()` - 初始化和更新基值
- `Normalization_ToPerUnit()` / `FromPerUnit()` - 物理量↔标幺值转换
- `Normalization_FromQ31WithBase()` - Q31格式转物理量（用于CORDIC硬件加速）

### 2.3 命令解析 (command)

**功能**：解析串口命令，支持参数查看/修改、VOFA绘图控制、电机激活管理。

**支持命令**：
- `plot` / `plot stop` - 启动/停止VOFA数据绘图
- `motor <id>` 或 `m<id>` - 显示电机参数
- `set motor<id> <param> = <value>` - 设置参数
- `set motor<id> enable/disable` - 激活/停用电机

**特性**：大小写不敏感、灵活格式、自动空格处理、详细错误提示。

**核心API**：
- `Command_Init()` - 初始化命令解析器
- `Command_Parse()` - 解析命令字符串
- `Command_GetLastError()` - 获取错误信息

### 2.4 FOC数学计算 (FOC_math)

**功能**：实现FOC控制所需的数学变换，基于标幺值计算，集成STM32G4 CORDIC硬件加速。

**数据流**：
```
物理电流 → 标幺转换 → Clarke变换 → αβ → Park变换 → dq
PWM输出 ← SVPWM调制 ← 逆Park变换 ← PID控制器 ← 标幺值
```

**核心API**：
- `Clarke_Transform()` - Clarke变换 (abc→αβ)，返回bool
- `Park_Transform()` - Park变换 (αβ→dq)
- `Inverse_Park_Transform()` - 逆Park变换 (dq→αβ)
- `SVPWM_minmax()` / `SVPWM_SectorBased()` - 空间矢量调制，直接输出定时器计数值
- `Sine_Cosine()` - 角度转正弦余弦（CORDIC硬件加速，性能提升3-5倍）
- `PWM_To_Voltage_ABC()` - 根据PWM占空比计算逆变器输出电压

**CORDIC优化**：使用STM32G4 CORDIC硬件加速三角函数计算，配置为6周期精度，Q1.31格式，失败时自动回退到ARM DSP库。Q31格式主要用于CORDIC硬件加速接口。

### 2.5 PID控制器 (PID_controller)

**功能**：提供统一的PID/PDFF控制算法，支持抗积分饱和，独立状态管理。

**控制算法**：
- **PID模式**（`Kfr_speed = 0`）：`output = kp*error + ki*∫error*dt + kd*d(error)/dt`
- **PDFF模式**（`Kfr_speed ≠ 0`）：`output = setpoint*Kfr_speed + ki*∫(setpoint-feedback)*dt - feedback*kp`
- 当 `Kfr_speed = kp` 时，PDFF退化为PI控制器

**核心API**：
- `PID_Controller()` - 统一PID/PDFF控制器，参数：setpoint, feedback, dt, params*, state*

**使用要点**：
- 每个控制回路必须有独立的状态变量（`PID_State_t`）
- 初始化时必须清零状态变量
- 支持多电机多回路控制

### 2.6 电流采样 (Current)

**功能**：双ADC同步采样三相电流，动态调整ADC触发点，支持高占空比场景。

**采样配置**：
- 双ADC同步模式：ADC1采样Ia/Ib，ADC2采样Ic/Vbus
- TIM1 CH4双边沿触发，每个PWM周期采样2次（中心对齐模式）
- 采样时间窗口：PWM周期50μs（20kHz），CH4脉宽1.47μs，ADC转换0.45μs

**动态触发策略**：
- 占空比<90%：固定在ARR-300触发（高侧采样）
- 占空比90-98%：动态调整到低侧采样
- 占空比>98%：降级采样或跳过

**核心API**：
- `Current_Init()` - 初始化电流采样模块
- `Current_GetABC()` / `Current_GetPU()` - 获取物理值/标幺值
- `Update_ADC_Trigger_Point()` - 动态调整ADC触发点
- `HAL_ADCEx_InjectedConvCpltCallback()` - ADC中断回调

### 2.7 FOC循环 (FOC_Loop)

**功能**：实现FOC开环测试和电流环控制。

**开环测试API**：
- `FOC_OpenLoopTest()` - 开环测试函数
  - 输入：电频率参考值（rad/s）
  - 输出：三相PWM占空比计数值
  - 功能：根据电频率计算角度增量，生成三相正弦波，完整FOC变换流程

**电流环控制**：
- `CurrentLoop_t` - 电流环控制器数据结构
- `CurrentLoop_Init()` - 初始化电流环
- `CurrentLoop_Run()` - 执行电流环控制
- `CurrentLoop_Reset()` - 复位电流环状态

**电流环控制流程**：
1. ADC采集三相电流
2. Clarke变换：abc → αβ
3. 计算逆变器输出电压（αβ坐标系）
4. 非线性磁链观测器估计电角度
5. Park变换：αβ → dq（电流反馈）
6. PID控制器计算dq轴电压（含解耦补偿）
7. 逆Park变换：dq → αβ（电压输出）
8. SVPWM生成PWM占空比

**调试变量**：
- `g_debug_angle` - 当前电角度（rad）
- `g_debug_sin` / `g_debug_cos` - sin(θ) / cos(θ)
- `g_debug_frequency` - 当前电频率（rad/s）
- `g_tim1_interrupt_count` / `g_tim1_interrupt_freq_hz` - TIM1中断计数和频率

### 2.8 位置估计 (Positioning)

**功能**：IPD脉冲法转子初始定位 + 非线性磁链观测器无传感器位置估计。

#### IPD脉冲法转子初始定位

**IPD脉冲法原理**：
- 在d轴发送12个特定角度的电压脉冲（每个脉冲角度差30°）
- 记录每个脉冲对应的电流值
- 根据电流响应峰值判断磁极和相位
- 电流响应峰值大的为N极，最大峰值对应的角度为转子位置

**核心API**：
- `IPD_DetectRotorPosition()` - 检测转子初始位置（简化接口）
- `IPD_ExecutePulseSequence()` - 执行完整的12脉冲定位序列
- `IPD_CalculateRotorPosition()` - 计算转子位置

#### 非线性磁链观测器无传感器位置估计

**基于论文**：Sensorless Control of Surface-Mount Permanent-Magnet Synchronous Motors Based on a Nonlinear Observer

**核心API**：
- `NonlinearObs_Position_Init()` - 初始化非线性磁链观测器
- `NonlinearObs_Position_Update()` - 更新观测器状态
- `NonlinearObs_Position_GetThetaRad()` - 获取位置估计（弧度）
- `NonlinearObs_Position_GetThetaDeg()` - 获取位置估计（度）

**观测器特性**：
- 基于磁链观测的位置估计
- 支持收敛判定和稳定性检测
- 集成归一化系统，支持标幺值运算

## 3. 系统集成

### 3.1 初始化流程

```c
int main(void) {
    // 1. 硬件初始化
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM_Init();
    MX_ADC_Init();
    MX_USART_Init();

    // 2. MotorControl模块初始化
    MotorParams_Init();
    Normalization_Init();
    Command_Init();
    Current_Init();

    // 3. 激活电机
    MotorParams_SetActiveMotor(MOTOR_0);
    Normalization_UpdateMotor(MOTOR_0);

    // 4. 启动控制任务
    while (1) {
        // 主循环处理命令等低频任务
    }
}
```

### 3.2 电流环控制示例

```c
// 全局电流环实例
CurrentLoop_t g_CurrentLoop;

// 初始化电流环
void CurrentLoop_Init_Example(void) {
    CurrentLoop_Init(&g_CurrentLoop, MOTOR_0, 0.00005f,  // 50us控制周期
                     0.5f, 0.1f,  // d轴PID参数
                     0.5f, 0.1f,  // q轴PID参数
                     100.0f);     // 观测器增益
}

// 在ADC中断中执行电流环控制
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        // 执行电流环控制
        uint32_t Tcm1, Tcm2, Tcm3;
        CurrentLoop_Run(&g_CurrentLoop, 0.0f, 0.5f,  // id_ref=0, iq_ref=0.5标幺值
                       &Tcm1, &Tcm2, &Tcm3);
        
        // 更新PWM输出
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Tcm1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Tcm2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Tcm3);
    }
}
```

### 3.3 开环测试示例

```c
// 在TIM1中断中调用
void TIM1_UP_IRQHandler(void) {
    uint32_t Tcm1, Tcm2, Tcm3;
    float frequency = 10.0f;  // 10 rad/s
    FOC_OpenLoopTest(frequency, &Tcm1, &Tcm2, &Tcm3);
    TIM1->CCR1 = Tcm1;
    TIM1->CCR2 = Tcm2;
    TIM1->CCR3 = Tcm3;
}
```

## 4. 快速开始

### 4.1 系统使用流程

```
┌─────────────────────────────────────────────────────────────┐
│                    系统使用流程图                           │
└─────────────────────────────────────────────────────────────┘

    ┌──────────────┐
    │  1. 硬件连接  │
    │  - 电机三相   │
    │  - 电流采样   │
    │  - 编码器     │
    └──────┬───────┘
           │
           ▼
    ┌──────────────┐
    │  2. 上电初始化│
    │  - HAL初始化  │
    │  - 模块初始化 │
    └──────┬───────┘
           │
           ▼
    ┌──────────────┐
    │  3. 参数配置  │────────┐
    │  串口命令设置 │        │
    │  电机参数     │        │
    └──────┬───────┘        │
           │                │
           ▼                │
    ┌──────────────┐        │
    │  4. 激活电机  │        │
    │  set motor0   │        │
    │  enable       │        │
    └──────┬───────┘        │
           │                │
           ▼                │
    ┌──────────────┐        │
    │  5. 位置估计  │        │
    │  (可选)       │        │
    │  IPD脉冲定位  │        │
    └──────┬───────┘        │
           │                │
           ▼                │
    ┌──────────────┐        │
    │  6. 开环测试  │        │
    │  验证PWM输出  │        │
    │  检查电流采样 │        │
    └──────┬───────┘        │
           │                │
           ▼                │
    ┌──────────────┐        │
    │  7. 闭环控制  │        │
    │  - 电流环     │        │
    │  - 速度环     │        │
    └──────┬───────┘        │
           │                │
           ▼                │
    ┌──────────────┐        │
    │  8. 监控调试  │        │
    │  VOFA绘图     │◄───────┘
    │  参数调整     │
    └──────────────┘
```

### 4.2 参数配置

通过串口命令配置电机参数：
```
set motor0 V_DC = 24.0
set motor0 I_rated = 5.0
set motor0 Rs = 0.3
set motor0 Pn = 4
set motor0 enable
plot
```

### 4.3 电流环控制配置

```c
// 初始化电流环
CurrentLoop_Init(&g_CurrentLoop, MOTOR_0, 0.00005f,  // 50us控制周期
                 0.5f, 0.1f,  // d轴PID参数 (kp, ki)
                 0.5f, 0.1f,  // q轴PID参数 (kp, ki)
                 100.0f);     // 观测器增益

// 启动电流环
g_CurrentLoop.is_running = true;
```

### 4.4 开环测试

```c
// 在TIM1中断中调用
void TIM1_UP_IRQHandler(void) {
    uint32_t Tcm1, Tcm2, Tcm3;
    float frequency = 10.0f;  // 10 rad/s
    FOC_OpenLoopTest(frequency, &Tcm1, &Tcm2, &Tcm3);
    TIM1->CCR1 = Tcm1;
    TIM1->CCR2 = Tcm2;
    TIM1->CCR3 = Tcm3;
}
```



## 8. 参考资料

[电机控制，PI控制器，PID控制器和现场定向控制简介 TI 德州仪器](https://www.ti.com.cn/zh-cn/video/series/motor-control.html)  

[Motor Control Blockset™ 快速入门指南](https://ww2.mathworks.cn/help/releases/R2025b/pdf_doc/mcb/mcb_gs_zh_CN.pdf)  

[Motor Control Blockset™ 参考](https://ww2.mathworks.cn/help/releases/R2025b/pdf_doc/mcb/mcb_ref_zh_CN.pdf)  

[How to use the CORDIC to perform mathematical functions on STM32 MCUs](https://www.st.com/resource/en/application_note/an5325-how-to-use-the-cordic-to-perform-mathematical-functions-on-stm32-mcus-stmicroelectronics.pdf)  

[FOC基础与实战培训教程 Microchip微芯](https://www.bilibili.com/video/BV1bs42137t4/?spm_id_from=333.337.search-card.all.click&vd_source=f454dcc6f008ec7697db6318909b0b78)  

[PID and PDFF Compensators for Motion Control](https://ieeexplore.ieee.org/document/377694/authors#authors)  

[永磁无刷电机及其驱动技术](https://ebooks.cmpbook.com/detail?id=13097)  

电机控制学习笔记 (QQ:986461300@qq.com) _捍卫你的梦想_ _那些年让你迷途的，不过是你自己不切实际的期盼罢了_  


### 8.1 技术文档

[Eclipse ThreadX documentation](https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/threadx/index.md)  



### 8.2 相关标准


### 8.3 开发工具

- **VScode**：集成开发环境
- **CMake**：自动化建构系统
- **STM32CubeCLT**：STM32开发工具集
- **STM32CubeMX**：代码生成工具
- **Cortex Debug (OpenOCD)**：调试工具
- **VOFA+**：串口调试助手
- **Git**：版本控制系统

## 9. 联系和支持

### 9.1 技术支持

- **代码仓库**：项目 Git 仓库
- **问题反馈**：通过 Issue 系统提交
- **文档更新**：不定期定期更新设计文档

---

**文档版本**：v2.2
**最后更新**：2025-11-20
**文档作者**：AI、ZHOUHENG  
**特别感谢**：  
    任何在网上分享电机控制相关的知识和经验的人  
    Linux do社区


**注意**：本文档描述的是 MotorControl 模块的当前实现。随着软件版本的更新，某些细节可能会发生变化。请定期查看最新版本的文档。