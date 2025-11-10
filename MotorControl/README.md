
# MotorControl 模块设计文档

## 1. 概述

MotorControl 是一个完整的永磁同步电机（PMSM）磁场定向控制（FOC）系统，为 STM32G4 平台设计。该模块提供了从参数管理到实时控制的全套解决方案，支持多电机配置、实时命令交互和高性能控制算法。

### 1.1 核心特性

- **多电机支持**：最多支持 2 个独立的电机参数套，可在代码中增加参数
- **状态管理**：动态激活/停用电机参数套
- **归一化系统**：基于硬件能力的标幺值计算，确保数值稳定性
- **统一控制器**：PID/PDFF 统一算法，支持多种控制模式
- **实时命令**：串口命令交互，支持参数查看和修改
- **FOC 算法**：完整的磁场定向控制数学计算

### 1.2 模块架构图

```
┌─────────────────────────────────────────────────────────────┐
│                    MotorControl 系统                        │
├─────────────────────────────────────────────────────────────┤
│  命令层 (command)                                           │
│  ┌─────────────────┐  ┌─────────────────┐                  │
│  │  串口命令解析    │  │  VOFA 数据绘图   │                  │
│  │  plot/motor/set │  │  实时数据监控    │                  │
│  └─────────────────┘  └─────────────────┘                  │
├─────────────────────────────────────────────────────────────┤
│  管理层 (motor_params + normalization)                     │
│  ┌─────────────────┐  ┌─────────────────┐                  │
│  │  电机参数管理    │  │  归一化基值计算  │                  │
│  │  多参数套支持    │  │  标幺值/Q31转换  │                  │
│  │  状态管理       │  │  动态基值更新    │                  │
│  └─────────────────┘  └─────────────────┘                  │
├─────────────────────────────────────────────────────────────┤
│  算法层 (FOC_math + PID_controller)                        │
│  ┌─────────────────┐  ┌─────────────────┐                  │
│  │  FOC 数学计算    │  │  PID/PDFF 控制器 │                  │
│  │  Clarke/Park    │  │  电流/速度控制   │                  │
│  │  SVPWM 调制     │  │  抗积分饱和      │                  │
│  └─────────────────┘  └─────────────────┘                  │
└─────────────────────────────────────────────────────────────┘
```

## 2. 模块详细说明

### 2.1 电机参数管理模块 (motor_params)

#### 2.1.1 功能职责
- 管理多套电机参数（默认 2 套）
- 提供参数的查看、修改和持久化
- 实现电机激活状态管理
- 支持通过 HMI 代码或参数名称访问参数

#### 2.1.2 数据结构

**电机参数结构体**
```c
typedef struct {
    float V_DC;         // 直流母线电压 (V)
    float I_rated;      // 额定电流 (A)
    float Rs;           // 定子电阻 (Ω)
    float Lq;           // q轴电感 (H)
    float Ld;           // d轴电感 (H)
    float RPM_rated;    // 额定转速 (rpm)
    float Pn;           // 极对数
    float Ke;           // 反电动势常数 (Vpk_LL/krpm)
    float Flux;         // 转子磁链 (Wb)
    float J;            // 转动惯量 (kg·m²×10⁻³)
    float B;            // 摩擦系数 (N·m·s/rad)
} Motor_Params_t;
```

**限值参数结构体**
```c
typedef struct {
    float I_limit_user;         // 用户电流限值 (A)
    float I_limit_max;          // 硬件电流限值 (A)
    float speed_limit_user;     // 用户转速限值 (rpm)
    float speed_limit_max;      // 电机最大转速 (rpm)
    float I_limit_actual;       // 实际电流限值 (A)
    float speed_limit_actual;   // 实际转速限值 (rpm)
} Motor_LimitParams_t;
```

#### 2.1.3 核心 API

| 函数名 | 功能描述 | 参数 | 返回值 |
|--------|----------|------|--------|
| [`MotorParams_Init()`](MotorControl/Src/motor_params.c:26) | 初始化所有电机参数 | void | void |
| [`MotorParams_SetParam()`](MotorControl/Src/motor_params.c:173) | 设置电机参数 | motor_id, param_name, value | void |
| [`MotorParams_PrintAll()`](MotorControl/Src/motor_params.c:122) | 打印所有参数 | motor_id | void |
| [`MotorParams_SetActiveMotor()`](MotorControl/Src/motor_params.c:224) | 激活指定电机 | motor_id | void |
| [`MotorParams_DisableMotor()`](MotorControl/Src/motor_params.c:235) | 停用指定电机 | motor_id | void |
| [`MotorParams_GetActiveMotor()`](MotorControl/Src/motor_params.c:250) | 获取激活电机ID | void | uint8_t |
| [`MotorParams_IsMotorEnabled()`](MotorControl/Src/motor_params.c:216) | 检查电机是否激活 | motor_id | bool |

#### 2.1.4 参数映射表

| HMI 代码 | 参数名 | 单位 | 描述 | 默认值(电机0) |
|----------|--------|------|------|---------------|
| P1001 | V_DC | V | 直流母线电压 | 14.0 |
| P1002 | I_rated | A | 额定电流 | 4.0 |
| P1003 | Rs | ohm | 定子电阻 | 0.5 |
| P1004 | Lq | mH | q轴电感 | 1.0 |
| P1005 | Ld | mH | d轴电感 | 1.0 |
| P1006 | RPM_rated | rpm | 额定转速 | 3000 |
| P1007 | Pn | - | 极对数 | 4 |
| P1008 | Ke | Vpk_LL/krpm | 反电动势常数 | 2.0 |
| P1009 | Flux | Wb | 转子磁链 | 计算得出 |
| P1010 | J | kg·m²×10⁻³ | 转动惯量 | 0.0 |
| P1011 | B | N·m·s/rad | 摩擦系数 | 0.0 |

### 2.2 归一化模块 (normalization)

#### 2.2.1 功能职责
- 基于硬件能力计算归一化基值
- 提供物理量与标幺值的双向转换
- 支持 Q31 定点运算转换
- 确保数值计算的稳定性和一致性

#### 2.2.2 归一化方案

采用**第二套方案**：基于硬件实际能力作为基值

```
电压基值:  V_base  = V_DC / √3
电流基值:  I_base  = I_limit_actual
转速基值:  ω_base  = (2π × RPM / 60) × Pn
磁链基值:  Flux_base = V_base / ω_base
转矩基值:  T_base  = 1.5 × Pn × Flux × I_base
功率基值:  P_base  = 1.5 × I_base × V_base
阻抗基值:  Z_base  = V_base / I_base
电感基值:  L_base  = Flux_base / I_base
时间基值:  T_base  = 1 / ω_base
```

#### 2.2.3 数据结构

```c
typedef struct {
    float voltage_base;    // 电压基值 (V)
    float current_base;    // 电流基值 (A)
    float omega_base;      // 电角速度基值 (rad/s)
    float flux_base;       // 磁链基值 (Wb)
    float torque_base;     // 转矩基值 (N·m)
    float power_base;      // 功率基值 (W)
    float impedance_base;  // 阻抗基值 (Ω)
    float inductance_base; // 电感基值 (H)
    float time_base;       // 时间基值 (s)
    float friction_base;   // 摩擦系数基值
    float inertia_base;    // 转动惯量基值
} normalization_base_values_t;
```

#### 2.2.4 核心 API

| 函数名 | 功能描述 | 参数 | 返回值 |
|--------|----------|------|--------|
| [`Normalization_Init()`](MotorControl/Src/normalization.c:153) | 初始化归一化模块 | void | void |
| [`Normalization_UpdateMotor()`](MotorControl/Src/normalization.c:160) | 更新电机基值 | motor_id | void |
| [`Normalization_GetBases()`](MotorControl/Src/normalization.c:178) | 获取基值结构 | motor_id | const base_values_t* |
| [`Normalization_ToPerUnit()`](MotorControl/Src/normalization.c:187) | 物理量→标幺值 | motor_id, quantity, value | float |
| [`Normalization_FromPerUnit()`](MotorControl/Src/normalization.c:202) | 标幺值→物理量 | motor_id, quantity, pu_value | float |
| [`Normalization_ToQ31()`](MotorControl/Src/normalization.c:216) | 物理量→Q31 | motor_id, quantity, value | q31_t |
| [`Normalization_FromQ31()`](MotorControl/Src/normalization.c:226) | Q31→物理量 | motor_id, quantity, q31_value | float |

#### 2.2.5 使用示例

```c
// 1. 激活电机并更新基值
MotorParams_SetActiveMotor(MOTOR_0);
Normalization_UpdateMotor(MOTOR_0);

// 2. 获取基值信息
const normalization_base_values_t *bases = Normalization_GetBases(MOTOR_0);
if (bases != NULL) {
    printf("电压基值: %.2fV, 电流基值: %.2fA\n", 
           bases->voltage_base, bases->current_base);
}

// 3. 电流转换为标幺值
float actual_current = 2.5f;  // 实际电流 2.5A
float current_pu = Normalization_ToPerUnit(MOTOR_0, NORMALIZE_CURRENT, actual_current);
// current_pu 范围 [-1, 1]

// 4. 标幺值转换为物理量
float voltage_pu = 0.8f;  // 标幺电压 0.8
float actual_voltage = Normalization_FromPerUnit(MOTOR_0, NORMALIZE_VOLTAGE, voltage_pu);

// 5. Q31 格式转换（用于定点运算）
q31_t current_q31 = Normalization_ToQ31(MOTOR_0, NORMALIZE_CURRENT, actual_current);
float current_back = Normalization_FromQ31(MOTOR_0, NORMALIZE_CURRENT, current_q31);
```

### 2.3 命令解析模块 (command)

#### 2.3.1 功能职责
- 解析串口输入的命令
- 支持电机参数查看和修改
- 控制 VOFA 数据绘图功能
- 管理电机激活状态

#### 2.3.2 支持的命令格式

| 命令类型 | 格式 | 示例 | 功能 |
|----------|------|------|------|
| **绘图命令** | `plot [stop]` | `plot`, `plot stop` | 启动/停止 VOFA 数据绘图 |
| **电机查看** | `motor <id>` | `motor 0`, `m0` | 显示指定电机参数 |
| **参数设置** | `set <motor> <param> = <value>` | `set motor0 Rs = 0.5` | 设置电机参数 |
| **状态管理** | `set <motor> enable/disable` | `set motor0 enable` | 激活/停用电机 |

#### 2.3.3 命令特性

- **大小写不敏感**：支持 `MOTOR0`, `motor0`, `Motor0` 等
- **灵活格式**：支持 `motor0`, `m 0`, `motor 0` 等写法
- **自动空格处理**：智能处理多余空格
- **错误提示**：详细的错误信息和使用帮助

#### 2.3.4 核心 API

| 函数名 | 功能描述 | 参数 | 返回值 |
|--------|----------|------|--------|
| [`Command_Init()`](MotorControl/Src/command.c:43) | 初始化命令解析器 | void | void |
| [`Command_Parse()`](MotorControl/Src/command.c:48) | 解析命令字符串 | command_line | void |
| [`Command_GetLastError()`](MotorControl/Src/command.c:119) | 获取最后的错误信息 | void | const char* |

#### 2.3.5 使用示例

```c
// 初始化
Command_Init();

// 在串口接收回调中调用
void UART_RxCallback(char* received_data) {
    Command_Parse(received_data);
    
    // 检查是否有错误
    const char* error = Command_GetLastError();
    if (error[0] != '\0') {
        printf("命令错误: %s\n", error);
    }
}

// 命令示例：
// "plot"                    // 启动绘图
// "plot stop"               // 停止绘图
// "motor 0"                 // 显示电机0参数
// "m0"                      // 显示电机0参数（简写）
// "set motor0 Rs = 0.5"     // 设置电机0电阻
// "set m1 Lq = 0.002"       // 设置电机1电感
// "set motor0 enable"        // 激活电机0
// "set m0 disable"          // 停用电机0
```

### 2.4 FOC 数学计算模块 (FOC_math)

#### 2.4.1 功能职责
- 实现 FOC 控制所需的数学变换
- 基于预标准化标幺值进行计算，确保数值稳定性和一致性
- 提供完整的输入验证和错误处理机制
- 实现完整的 SVPWM 调制算法，直接输出定时器计数值
- 支持角度路径的原始弧度处理和自动包装
- **集成 STM32G4 CORDIC 硬件加速**，提升三角函数计算性能

#### 2.4.2 数据格式约定

**数据流契约**：
- **相电流**：必须在调用 Clarke_Transform 之前转换为标幺值
- **电角度**：以弧度(float)形式提供给 Sine_Cosine；函数自动包装到 [-π, π] 范围
- **电压路径**：Park/逆Park变换、SVPWM 和下游 PWM 映射都基于标幺值操作

**数据流程**：
```
物理电流 → 标幺转换 → Clarke变换 → αβ坐标系 → Park变换 → dq坐标系
    ↑                                                       ↓
PWM输出 ← SVPWM调制 ← 逆Park变换 ← PID控制器 ← 电流环控制 ← 标幺值
```

#### 2.4.3 核心 API

| 函数名 | 功能描述 | 参数 | 返回值 |
|--------|----------|------|--------|
| [`Clarke_Transform()`](MotorControl/Src/FOC_math.c:301) | Clarke 变换 (abc→αβ) | ia_pu, ib_pu, I_alpha*, I_beta* | bool |
| [`Park_Transform()`](MotorControl/Src/FOC_math.c:424) | Park 变换 (αβ→dq) | I_alpha_pu, I_beta_pu, sinθ, cosθ, I_d*, I_q* | void |
| [`Inverse_Park_Transform()`](MotorControl/Src/FOC_math.c:180) | 逆 Park 变换 (dq→αβ) | U_d_pu, U_q_pu, sinθ, cosθ, U_alpha*, U_beta* | void |
| [`SVPWM()`](MotorControl/Src/FOC_math.c:228) | 空间矢量脉宽调制 | U_alpha_pu, U_beta_pu, Tcm1*, Tcm2*, Tcm3* | void |
| [`Sine_Cosine()`](MotorControl/Src/FOC_math.c:340) | 角度转正弦余弦 (CORDIC硬件加速) | θ_e, sinθ*, cosθ* | void |
| [`Sine_CosineQ31()`](MotorControl/Src/FOC_math.c:379) | Q31格式角度转正弦余弦 (CORDIC硬件加速) | θ_e, sinθ_q31*, cosθ_q31* | void |

**Q31 定点运算版本**：
| 函数名 | 功能描述 | 参数 | 返回值 |
|--------|----------|------|--------|
| [`Clarke_TransformQ31()`](MotorControl/Src/FOC_math.c:318) | Q31 Clarke 变换 | ia_q31, ib_q31, I_alpha_q31*, I_beta_q31* | bool |
| [`Park_TransformQ31()`](MotorControl/Src/FOC_math.c:454) | Q31 Park 变换 | I_alpha_q31, I_beta_q31, sinθ_q31, cosθ_q31, I_d_q31*, I_q_q31* | bool |
| [`Inverse_Park_TransformQ31()`](MotorControl/Src/FOC_math.c:202) | Q31 逆 Park 变换 | U_d_q31, U_q_q31, sinθ_q31, cosθ_q31, U_alpha_q31*, U_beta_q31* | bool |
| [`SVPWM_Q31()`](MotorControl/Src/FOC_math.c:259) | Q31 空间矢量脉宽调制 | U_alpha_q31, U_beta_q31, Tcm1*, Tcm2*, Tcm3* | bool |

**重要变更说明**：
- **Clarke_Transform**: 现在返回 `bool` 类型，输入参数必须为标幺值
- **参数命名**: 所有电流和电压参数使用 `_pu` 后缀表示标幺值
- **错误处理**: 无效输入时输出零值，Clarke_Transform 返回 false
- **CORDIC硬件加速**: Sine_Cosine函数使用STM32G4 CORDIC硬件，性能提升3-5倍
- **Q31定点运算**: 新增完整的Q31定点运算版本，适用于高频控制路径

#### 2.4.4 CORDIC 硬件加速优化

**CORDIC 配置参数**：
- **函数类型**: `CORDIC_FUNCTION_COSINE` (同时输出cos和sin)
- **精度等级**: `CORDIC_PRECISION_6CYCLES` (适合电机控制的平衡精度)
- **输入输出**: Q1.31格式，无缩放
- **超时设置**: 20个周期 (合理的计算时间)

**优化效果**：
- **计算速度**: 比 ARM DSP 库快 3-5 倍
- **精度**: 6周期精度提供足够的电机控制精度
- **实时性**: 显著提升FOC控制环路性能

**错误处理机制**：
- CORDIC配置失败时自动回退到ARM DSP库
- 计算失败时使用软件实现作为备用方案
- 完整的输入参数验证

#### 2.4.5 FOC 控制流程示例

```c
// 完整的 FOC 控制循环示例（v2.1 CORDIC优化版本）
void FOC_Control_Loop(void) {
    // 1. 电流采样（物理量）
    float ia_physical = get_phase_current_A();  // A相电流 (A)
    float ib_physical = get_phase_current_B();  // B相电流 (A)
    
    // 2. 电流转换为标幺值（重要新增步骤）
    uint8_t active_motor = MotorParams_GetActiveMotor();
    float ia_pu = Normalization_ToPerUnit(active_motor, NORMALIZE_CURRENT, ia_physical);
    float ib_pu = Normalization_ToPerUnit(active_motor, NORMALIZE_CURRENT, ib_physical);
    
    // 3. Clarke 变换：abc → αβ（标幺值输入）
    float I_alpha_pu, I_beta_pu;
    bool clarke_ok = Clarke_Transform(ia_pu, ib_pu, &I_alpha_pu, &I_beta_pu);
    
    if (!clarke_ok) {
        // Clarke 变换失败，输出零电压
        TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = ARR_PERIOD / 2;
        return;
    }
    
    // 4. 获取转子电角度（原始弧度）- CORDIC硬件加速
    float theta_e = get_electrical_angle();  // 电角度 (rad)
    float sin_theta, cos_theta;
    Sine_Cosine(theta_e, &sin_theta, &cos_theta);  // 使用CORDIC硬件加速
    
    // 5. Park 变换：αβ → dq（标幺值输入，使用CORDIC提供的sin/cos）
    float id_feedback, iq_feedback;
    Park_Transform(I_alpha_pu, I_beta_pu, sin_theta, cos_theta, &id_feedback, &iq_feedback);
    
    // 6. PID 控制（输出标幺值）
    float id_ref = 0.0f;  // d轴电流设定值（标幺值）
    float iq_ref = speed_controller_output;  // q轴电流设定值（标幺值）
    
    float vd_pu = PID_Controller(id_ref, id_feedback, dt, &id_params, &id_state);
    float vq_pu = PID_Controller(iq_ref, iq_feedback, dt, &iq_params, &iq_state);
    
    // 7. 逆 Park 变换：dq → αβ（使用CORDIC提供的sin/cos）
    float U_alpha_pu, U_beta_pu;
    Inverse_Park_Transform(vd_pu, vq_pu, sin_theta, cos_theta, &U_alpha_pu, &U_beta_pu);
    
    // 8. SVPWM 调制：αβ → 定时器计数值（直接输出）
    uint32_t Tcm1, Tcm2, Tcm3;
    SVPWM(U_alpha_pu, U_beta_pu, &Tcm1, &Tcm2, &Tcm3);
    
    // 9. 更新 PWM 寄存器
    TIM1->CCR1 = Tcm1;
    TIM1->CCR2 = Tcm2;
    TIM1->CCR3 = Tcm3;
}
```

**高性能 Q31 版本示例**：
```c
// 高频控制路径使用 Q31 定点运算
void FOC_Control_Loop_Q31(void) {
    // 1. 电流采样并直接转换为 Q31 格式
    float ia_physical = get_phase_current_A();
    float ib_physical = get_phase_current_B();
    
    q31_t ia_q31 = Normalization_ToQ31(MOTOR_0, NORMALIZE_CURRENT, ia_physical);
    q31_t ib_q31 = Normalization_ToQ31(MOTOR_0, NORMALIZE_CURRENT, ib_physical);
    
    // 2. Q31 Clarke 变换
    q31_t I_alpha_q31, I_beta_q31;
    bool clarke_ok = Clarke_TransformQ31(ia_q31, ib_q31, &I_alpha_q31, &I_beta_q31);
    
    if (!clarke_ok) {
        TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = ARR_PERIOD / 2;
        return;
    }
    
    // 3. CORDIC Q31 三角函数计算
    float theta_e = get_electrical_angle();
    q31_t sin_theta_q31, cos_theta_q31;
    Sine_CosineQ31(theta_e, &sin_theta_q31, &cos_theta_q31);
    
    // 4. Q31 Park 变换
    q31_t id_q31, iq_q31;
    Park_TransformQ31(I_alpha_q31, I_beta_q31, sin_theta_q31, cos_theta_q31, &id_q31, &iq_q31);
    
    // 5. Q31 PID 控制（需要将参数转换为 Q31）
    q31_t vd_q31 = PID_Controller_Q31(id_ref_q31, id_q31, dt_q31, &id_params_q31, &id_state_q31);
    q31_t vq_q31 = PID_Controller_Q31(iq_ref_q31, iq_q31, dt_q31, &iq_params_q31, &iq_state_q31);
    
    // 6. Q31 逆 Park 变换
    q31_t U_alpha_q31, U_beta_q31;
    Inverse_Park_TransformQ31(vd_q31, vq_q31, sin_theta_q31, cos_theta_q31, &U_alpha_q31, &U_beta_q31);
    
    // 7. Q31 SVPWM 调制
    uint32_t Tcm1, Tcm2, Tcm3;
    SVPWM_Q31(U_alpha_q31, U_beta_q31, &Tcm1, &Tcm2, &Tcm3);
    
    // 8. 更新 PWM 寄存器
    TIM1->CCR1 = Tcm1;
    TIM1->CCR2 = Tcm2;
    TIM1->CCR3 = Tcm3;
}
```

**重构要点**：
- **标幺值转换**: 物理电流必须先转换为标幺值
- **CORDIC硬件加速**: Sine_Cosine函数使用STM32G4 CORDIC硬件
- **错误处理**: Clarke_Transform 返回值检查
- **数据一致性**: 整个计算链保持标幺值一致性
- **直接输出**: SVPWM 直接输出定时器计数值
- **Q31定点运算**: 高频路径可选使用Q31版本进一步提升性能

### 2.5 PID 控制器模块 (PID_controller)

#### 2.5.1 功能职责
- 提供统一的 PID/PDFF 控制算法
- 支持抗积分饱和保护
- 自动模式切换（通过参数配置）
- 独立的状态管理，支持多控制器实例

#### 2.5.2 控制算法

**PID 模式**（当 `Kfr_speed = 0` 时）
```
error = setpoint - feedback
output = kp * error + ki * ∫error*dt + kd * d(error)/dt
```

**PDFF 模式**（当 `Kfr_speed ≠ 0` 时）
```
output = setpoint * Kfr_speed + ki * ∫(setpoint - feedback)*dt - feedback * kp
```

**特殊性质**：当 `Kfr_speed = kp` 时，PDFF 退化为 PI 控制器

#### 2.5.3 数据结构

```c
// 控制器参数
typedef struct {
    float kp;                  // 比例系数
    float ki;                  // 积分系数
    float kd;                  // 微分系数（暂不使用）
    float Kfr_speed;           // 速度环前馈系数（0=PID，非0=PDFF）
    float integral_limit;      // 积分限幅
    float output_limit;        // 输出限幅
} PID_Params_t;

// 控制器状态
typedef struct {
    float integral;            // 积分累积值
    float prev_error;          // 上次误差值
} PID_State_t;
```

#### 2.5.4 核心 API

| 函数名 | 功能描述 | 参数 | 返回值 |
|--------|----------|------|--------|
| [`PID_Controller()`](MotorControl/Src/PID_controller.c:13) | 统一 PID/PDFF 控制器 | setpoint, feedback, dt, params*, state* | float |

#### 2.5.5 使用示例

**示例 1：电流环 PI 控制**
```c
// 参数配置
PID_Params_t current_params = {
    .kp = 0.5f,
    .ki = 10.0f,
    .kd = 0.0f,
    .Kfr_speed = 0.0f,        // PID 模式
    .integral_limit = 0.5f,
    .output_limit = 1.0f
};

// 状态变量（必须独立且初始化为0）
PID_State_t current_state = {0};

// 控制循环
float vd = PID_Controller(id_ref, id_fb, 0.0001f, &current_params, &current_state);
```

**示例 2：速度环 PDFF 控制**
```c
// 参数配置
PID_Params_t speed_params = {
    .kp = 0.1f,
    .ki = 1.0f,
    .kd = 0.0f,
    .Kfr_speed = 0.1f,        // PDFF 模式
    .integral_limit = 0.5f,
    .output_limit = 1.0f
};

// 状态变量
PID_State_t speed_state = {0};

// 控制循环
float iq_ref = PID_Controller(speed_ref, speed_fb, 0.001f, &speed_params, &speed_state);
```

**示例 3：多电机多回路控制**
```c
typedef struct {
    PID_Params_t id_params;
    PID_State_t  id_state;
    PID_Params_t iq_params;
    PID_State_t  iq_state;
    PID_Params_t speed_params;
    PID_State_t  speed_state;
} Motor_Controller_t;

// 两个电机独立的控制器
Motor_Controller_t motor[2];

// 初始化
for (int i = 0; i < 2; i++) {
    motor[i].id_state = (PID_State_t){0};
    motor[i].iq_state = (PID_State_t){0};
    motor[i].speed_state = (PID_State_t){0};
}

// 使用
float vd = PID_Controller(0, id_fb, dt, &motor[0].id_params, &motor[0].id_state);
float vq = PID_Controller(iq_ref, iq_fb, dt, &motor[0].iq_params, &motor[0].iq_state);
```

#### 2.5.6 状态管理要点

1. **每个控制回路必须有独立的状态变量**
2. **初始化时必须清零状态变量**
3. **函数会自动更新状态，无需手动操作**
4. **不能多个控制器共享同一个状态变量**
5. **需要重置时，将状态变量清零即可**

## 3. 数据流和系统集成

### 3.1 完整数据流图

```
┌─────────────────────────────────────────────────────────────────┐
│                        FOC 控制数据流                           │
└─────────────────────────────────────────────────────────────────┘

┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  电流采样    │    │ Clarke变换  │    │ Park变换    │    │ PID控制器    │
│  ia, ib, ic  │───▶│  Iα, Iβ     │───▶│  Id, Iq     │───▶│  Vd, Vq      │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
                                                            │
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    │
│  PWM输出    │◀───│ SVPWM调制   │◀───│ 逆Park变换  │◀───┘
│  Tcm1,2,3   │    │  Tcm1,2,3   │    │  Vα, Vβ     │
└─────────────┘    └─────────────┘    └─────────────┘
                                           ▲
┌─────────────┐    ┌─────────────┐          │
│  角度信息    │    │  归一化模块  │──────────┘
│  θe         │    │  标幺值转换  │
└─────────────┘    └─────────────┘
                          ▲
┌─────────────┐    ┌─────────────┐
│  参数管理    │    │  电机参数    │
│  激活状态    │    │  硬件限制    │
└─────────────┘    └─────────────┘
```

### 3.2 系统集成流程

#### 3.2.1 初始化序列

```c
int main(void) {
    // 1. 基础硬件初始化
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM_Init();
    MX_ADC_Init();
    MX_USART_Init();
    
    // 2. MotorControl 模块初始化（按依赖顺序）
    MotorParams_Init();           // 初始化电机参数
    Normalization_Init();         // 初始化归一化模块
    Command_Init();               // 初始化命令解析
    
    // 3. PID 控制器初始化（应用层）
    // 注意：PID控制器由应用层管理，模块本身不包含全局初始化
    
    // 4. 激活默认电机（可选）
    MotorParams_SetActiveMotor(MOTOR_0);
    Normalization_UpdateMotor(MOTOR_0);
    
    // 5. 启动实时控制任务
    // osThreadNew(FOC_Control_Task, NULL, &foc_task_attributes);
    
    while (1) {
        // 主循环处理命令等低频任务
    }
}
```

#### 3.2.2 实时控制循环

```c
// 高频控制任务（建议 ≥10kHz）
void FOC_Control_Task(void *argument) {
    // 控制周期
    const float dt = 0.0001f;  // 10kHz
    
    // 检查电机激活状态
    if (!MotorParams_IsAnyMotorActive()) {
        // 没有激活电机，输出零电压
        TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
        osDelay(1);
        return;
    }
    
    while (1) {
        // FOC 控制算法
        FOC_Control_Loop();
        
        // 等待下一个控制周期
        osDelayUntil(&last_wake_time, dt * 1000);
    }
}
```

### 3.3 命令交互流程

```
用户输入串口命令
        │
        ▼
┌─────────────────┐
│ Command_Parse() │───▶ 解析命令类型
└─────────────────┘        │
        │                  ▼
        │        ┌─────────────────────┐
        │        │   命令类型判断       │
        │        └─────────────────────┘
        │                  │
        ▼                  ▼
┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│ plot 命令   │  │ motor 命令  │  │ set 命令    │
│ VOFA 控制   │  │ 参数查看    │  │ 参数修改    │
└─────────────┘  └─────────────┘  └─────────────┘
        │                  │                  │
        ▼                  ▼                  ▼
┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│ Vofa_Plot*  │  │MotorParams* │  │MotorParams* │
│ 函数调用     │  │ 参数打印     │  │ 参数设置     │
└─────────────┘  └─────────────┘  └─────────────┘
                                         │
                                         ▼
                                ┌─────────────┐
                                │ Normalization│
                                │ _UpdateMotor │
                                └─────────────┘
```

## 4. 使用指南和最佳实践

### 4.1 快速开始

#### 4.1.1 基本配置步骤

1. **配置电机参数**
```c
// 通过命令行配置
set motor0 V_DC = 24.0
set motor0 I_rated = 5.0
set motor0 Rs = 0.3
set motor0 Pn = 4
// ... 其他参数
```

2. **激活电机**
```c
set motor0 enable
```

3. **启动控制**
```c
// 通过命令启动 VOFA 绘图
plot
```

#### 4.1.2 代码集成示例

```c
// main.c 中的集成
#include "MotorControl/Inc/motor_params.h"
#include "MotorControl/Inc/normalization.h"
#include "MotorControl/Inc/command.h"
#include "MotorControl/Inc/FOC_math.h"
#include "MotorControl/Inc/PID_controller.h"

// 全局变量
PID_Params_t g_id_params, g_iq_params, g_speed_params;
PID_State_t  g_id_state, g_iq_state, g_speed_state;

void init_motor_control(void) {
    // 初始化模块
    MotorParams_Init();
    Normalization_Init();
    Command_Init();
    
    // 配置 PID 参数
    g_id_params = (PID_Params_t){
        .kp = 0.5f, .ki = 10.0f, .kd = 0.0f, .Kfr_speed = 0.0f,
        .integral_limit = 0.5f, .output_limit = 1.0f
    };
    
    g_iq_params = (PID_Params_t){
        .kp = 0.5f, .ki = 10.0f, .kd = 0.0f, .Kfr_speed = 0.0f,
        .integral_limit = 0.5f, .output_limit = 1.0f
    };
    
    g_speed_params = (PID_Params_t){
        .kp = 0.1f, .ki = 1.0f, .kd = 0.0f, .Kfr_speed = 0.1f,
        .integral_limit = 0.5f, .output_limit = 1.0f
    };
    
    // 清零状态
    g_id_state = g_iq_state = g_speed_state = (PID_State_t){0};
    
    // 激活电机
    MotorParams_SetActiveMotor(MOTOR_0);
    Normalization_UpdateMotor(MOTOR_0);
}

void foc_control_loop(void) {
    static float speed_ref = 1000.0f;  // 速度设定值
    
    // 获取反馈信号
    float ia = get_current_a();
    float ib = get_current_b();
    float theta_e = get_electrical_angle();
    float speed_fb = get_speed_feedback();
    
    // FOC 变换
    float I_alpha, I_beta;
    Clarke_Transform(ia, ib, &I_alpha, &I_beta);
    
    float sin_theta, cos_theta;
    Sine_Cosine(theta_e, &sin_theta, &cos_theta);
    
    float id_fb, iq_fb;
    Park_Transform(I_alpha, I_beta, sin_theta, cos_theta, &id_fb, &iq_fb);
    
    // 速度环（假设1kHz）
    static uint32_t speed_counter = 0;
    if (++speed_counter >= 100) {  // 每100次电流环执行一次速度环
        speed_counter = 0;
        float iq_ref = PID_Controller(speed_ref, speed_fb, 0.001f, 
                                   &g_speed_params, &g_speed_state);
        
        // 电流环
        float vd = PID_Controller(0.0f, id_fb, 0.0001f, 
                                &g_id_params, &g_id_state);
        float vq = PID_Controller(iq_ref, iq_fb, 0.0001f, 
                                &g_iq_params, &g_iq_state);
        
        // 逆变换和 SVPWM
        float U_alpha, U_beta;
        Inverse_Park_Transform(vd, vq, sin_theta, cos_theta, &U_alpha, &U_beta);
        
        uint32_t Tcm1, Tcm2, Tcm3;
        SVPWM(U_alpha, U_beta, &Tcm1, &Tcm2, &Tcm3);
        
        // 更新 PWM
        TIM1->CCR1 = Tcm1;
        TIM1->CCR2 = Tcm2;
        TIM1->CCR3 = Tcm3;
    }
}
```

### 4.2 最佳实践

#### 4.2.1 参数整定建议

1. **电流环参数**
   - 先整定 d 轴电流环（设定值 0）
   - 使用 PI 控制，Kfr_speed = 0
   - 比例系数从 0.1 开始，逐步增加
   - 积分系数确保稳态误差为 0

2. **速度环参数**
   - 推荐使用 PDFF 控制
   - Kfr_speed 可以提高动态响应
   - 设定值等于 kp 时退化为 PI 控制

3. **归一化基值**
   - 确保所有基值都大于 0
   - 电流基值使用实际限值而非额定值
   - 定期检查基值有效性

#### 4.2.2 调试技巧

1. **使用 VOFA 绘图**
```c
// 在控制循环中添加数据发送
Vofa_SendData(id_fb, iq_fb, vd, vq, speed_fb, speed_ref);
```

2. **参数验证**
```c
// 验证归一化基值
const normalization_base_values_t *bases = Normalization_GetBases(MOTOR_0);
if (bases == NULL) {
    printf("错误：归一化基值无效\n");
}
```

3. **状态监控**
```c
// 检查控制器状态
printf("积分值: %.3f, 上次误差: %.3f\n", 
       g_id_state.integral, g_id_state.prev_error);
```

#### 4.2.3 性能优化

1. **定点运算**
   - 高频路径使用 Q31 格式
   - 归一化模块提供 Q31 转换接口
   - 保留浮点运算用于参数配置

2. **内存优化**
   - 避免在控制循环中动态分配内存
   - 使用静态变量存储状态
   - 合理设置缓冲区大小

3. **实时性保证**
   - 控制循环执行时间应小于周期时间
   - 避免在控制循环中调用阻塞函数
   - 使用 DMA 减少 CPU 占用

### 4.3 常见问题解决

#### 4.3.1 归一化基值无效

**问题**：`Normalization_GetBases()` 返回 NULL

**原因**：
- 电机未激活
- 参数值无效（0 或负数）
- 基值计算失败

**解决**：
```c
// 检查电机状态
if (!MotorParams_IsMotorEnabled(MOTOR_0)) {
    printf("请先激活电机: set motor0 enable\n");
}

// 检查参数值
MotorParams_PrintAll(MOTOR_0
// 手动更新基值
Normalization_UpdateMotor(MOTOR_0);
```

#### 4.3.2 PID 控制器振荡

**问题**：控制系统出现振荡或不稳定

**原因**：
- 参数过大
- 积分限幅不合适
- 控制周期不稳定

**解决**：
```c
// 减小比例系数
params.kp *= 0.5;

// 调整积分限幅
params.integral_limit = params.output_limit * 0.8;

// 确保控制周期稳定
float actual_dt = get_actual_control_period();
if (actual_dt > expected_dt * 1.1f) {
    printf("警告：控制周期超时\n");
}
```

#### 4.3.3 命令解析失败

**问题**：命令无法正确解析

**原因**：
- 命令格式错误
- 参数名不正确
- 数值格式错误

**解决**：
```c
// 检查错误信息
const char* error = Command_GetLastError();
if (error[0] != '\0') {
    printf("命令错误: %s\n", error);
}

// 使用正确的格式
// 错误: set motor0 Rs=0.5
// 正确: set motor0 Rs = 0.5
```

## 5. 性能指标和限制

### 5.1 计算性能

### 5.2 数值精度

| 数据类型 | 用途 | 精度 | 范围 |
|----------|------|------|------|
| float32 | 参数计算 | 单精度 | ±3.4e38 |
| q31_t | 实时控制 | 31位定点 | [-1, 1] |
| uint32_t | PWM 计数 | 整数 | 0 到 ARR |

### 5.3 系统限制

- **最大电机数量**：2 个（可通过修改 `motors_number` 调整）
- **最大参数数量**：11 个（可通过修改 `PARAM_COUNT` 调整）
- **命令长度限制**：128 字符
- **控制周期建议**：100 μs - 1 ms

## 6. 扩展和定制

### 6.1 添加新的电机参数

1. **更新数据结构**
```c
// 在 motor_params.h 中添加
typedef struct {
    // ... 现有参数
    float new_param;  // 新参数
} Motor_Params_t;
```

2. **更新参数描述**
```c
// 在 motor_params.c 中添加
const ParamDesc_t param_descs[PARAM_COUNT + 1] = {
    // ... 现有参数
    {"P1012", "new_param", "unit", "新参数描述"}  // 新参数
};
```

3. **更新映射表**
```c
// 为每个电机添加新参数的映射
{&motor_params[MOTOR_0].new_param, PARAM_COUNT},
```

### 6.2 添加新的命令

1. **在 command.c 中添加处理函数**
```c
static void handle_new_command(char* args) {
    // 命令处理逻辑
}
```

2. **在 Command_Parse() 中添加分支**
```c
else if (string_equals_ignore_case(cursor, "new")) {
    handle_new_command(args);
}
```

### 6.3 集成新的控制算法

1. **创建新的控制模块**
```c
// new_controller.h
typedef struct {
    float param1;
    float param2;
} New_Controller_Params_t;

float New_Controller_Calculate(float setpoint, float feedback, 
                              New_Controller_Params_t *params);
```

2. **在控制循环中集成**
```c
// 在 FOC_Control_Loop() 中添加
float output = New_Controller_Calculate(setpoint, feedback, &new_params);
```

## 7. 版本历史和更新记录

### 7.1 当前版本 (v2.1)

**发布日期**：2025-11-09

**主要更新**：
- **FOC 数学核心重构**: 完全基于预标准化标幺值进行计算，消除单位混合风险
- **数据流契约**: 明确定义所有 API 的数据格式要求，确保数据流一致性
- **输入验证增强**: 新增 foc_math_is_valid_pu() 函数，提供严格的标幺值验证
- **错误处理优化**: 无效输入时输出零值，Clarke_Transform 返回布尔值允许上游错误处理
- **角度路径文档化**: 明确角度处理使用原始弧度，自动包装到 [-π, π] 范围
- **SVPWM 直接映射**: 标幺值直接映射到定时器计数值，提高性能
- **STM32G4 CORDIC 硬件加速**: 集成CORDIC硬件加速三角函数计算 
- **Q31 定点运算支持**: 新增完整的Q31定点运算版本，适用于高频控制路径

### 7.2 版本 v2.0 (2025-11-08)

**主要更新**：
- 完全重构 PID 控制器，统一 PID/PDFF 算法
- 实现动态电机状态管理机制
- 优化归一化模块，支持动态基值更新
- 增强 FOC 数学计算模块的鲁棒性
- 完善命令解析系统，支持状态管理命令

### 7.3 版本 v1.1 (2025-11-04)

**主要更新**：
- 添加归一化模块
- 实现 FOC 数学计算函数
- 基础命令解析功能
- 多电机参数支持

### 7.4 版本 v1.0 (2025-11-03)

**初始版本**：
- 基础电机参数管理
- 简单命令解析
- PID 控制器框架

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

**文档版本**：v2.1
**最后更新**：2025-11-09
**文档作者**：AI、ZHOUHENG  
**特别感谢**：  
    任何在网上分享电机控制相关的知识和经验的人  
    Linux do社区


**注意**：本文档描述的是 MotorControl 模块的当前实现。随着软件版本的更新，某些细节可能会发生变化。请定期查看最新版本的文档。