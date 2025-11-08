## MotorControl 设计文档

### 总体架构
- MotorControl 子系统负责电机参数管理、归一化（标幺化）计算、命令行交互以及后续的控制器实现。
- 主要模块：命令解析 (`command`)、参数存储 (`motor_params`)、归一化 (`normalization`)、PID 控制器 (`PID_controller`)。
- 所有模块均采用 C 风格全局初始化函数，由 `main.c` 在外设初始化完成后调用。

### 模块概览

| 模块 | 路径 | 主要职责 | 关键依赖 | 主要 API |
| --- | --- | --- | --- | --- |
| command | `MotorControl/Inc/command.h`<br>`MotorControl/Src/command.c` | 解析串口/HMI 命令、打印/修改参数、联动归一化刷新、状态管理命令 | `motor_params`, `normalization`, `Vofa_STM32G474` | `Command_Init()` `Command_Parse()` `Command_GetLastError()` |
| motor_params | `MotorControl/Inc/motor_params.h`<br>`MotorControl/Src/motor_params.c` | 保存电机与限幅参数，提供描述符和映射、状态管理 | `stdio.h` `math.h` `usart.h` | `MotorParams_Init()` `MotorParams_SetParam()` `MotorParams_PrintAll()` `MotorParams_SetActiveMotor()` `MotorParams_DisableMotor()` 等 |
| normalization | `MotorControl/Inc/normalization.h`<br>`MotorControl/Src/normalization.c` | 基于第二套方案生成基值，提供标幺/Q31 转换、只处理激活电机 | `arm_math.h` `motor_params.h` | `Normalization_Init()` `Normalization_UpdateMotor()` `Normalization_GetBases()` `Normalization_ToPerUnit()` 等 |
| FOC_math | `MotorControl/Inc/FOC_math.h`<br>`MotorControl/Src/FOC_math.c` | FOC数学计算函数、动态电机ID支持、标幺化集成 | `main.h` `arm_math.h` `normalization.h` `motor_params.h` | `Clarke_Transform()` `Park_Transform()` `Inverse_Park_Transform()` `SVPWM()` `Sine_Cosine()` |
| PID_controller | `MotorControl/Inc/PID_controller.h`<br>`MotorControl/Src/PID_controller.c` | FOC电机PID控制器、抗积分饱和、电流环解耦、速度前馈 | `motor_params`, `normalization`, `arm_math.h` | `PID_Init()` `PID_Calculate()` `PID_CurrentD_Calculate()` `PID_CurrentQ_Calculate()` `PID_Speed_Calculate()` `PID_Position_Calculate()` 等 |

---

### command 模块

**职责**
- 接收字符命令（如 `plot`, `motor`, `set`），提供参数查看/修改和 VOFA 曲线控制。
- 自动忽略大小写、处理前后空白、支持 `motor0`/`m0` 等多种索引写法。
- 通过 `MotorParams_SetParam` 更新参数后调用 `Normalization_UpdateMotor` 确保基值同步。
- **状态管理**：支持电机参数套的激活/停用命令，与归一化模块联动更新基值。

**关键实现**
- `Command_Parse`：主解析入口，负责分派到 `handle_plot_command`、`handle_motor_command`、`handle_set_command`。
- 解析过程中维护 `last_error` 缓冲，通过 `Command_GetLastError` 对外报告错误。
- `parse_motor_id_token` 将输入 token 转换为有效电机 ID，并在出错时给出帮助提示。
- **状态管理命令**：
  - `handle_enable_command(uint8_t motor_id, bool enable)`：处理电机激活/停用逻辑
  - `ends_with_ignore_case()`：辅助函数，支持大小写不敏感的 enable/disable 命令识别
  - 支持 `set motor0 enable`、`set m0 disable` 等格式

---

### motor_params 模块

**数据结构**
- `Motor_Params_t`：母线电压、额定电流、Rs、Lq/Ld、额定转速、极对数、Ke、Flux、J、B。
- `Motor_LimitParams_t`：用户/硬件两级限制以及最终生效值（以 `fmin` 合成）。
- `ParamDesc_t` & `ParamMap_t`：参数描述符和指针映射数组，便于通过 HMI 编号或名称访问。
- **状态管理**：`g_active_motor_id` 全局变量跟踪当前激活的电机参数套（初始值 `0xFF` 表示无激活电机）。

**逻辑**
- `MotorParams_Init`：为 `MOTOR_0` 与 `MOTOR_1` 写入默认参数和限幅，计算 `I_limit_actual`、`speed_limit_actual`。
- `MotorParams_SetParam`：支持 HMI 码（PXXXX）或名称设置参数，变更后输出确认信息。
- `MotorParams_PrintAll`：遍历 `param_maps` 表输出参数、单位与描述。

**状态管理 API**
- `MotorParams_SetActiveMotor(uint8_t motor_id)`：激活指定电机参数套
- `MotorParams_DisableMotor(uint8_t motor_id)`：停用指定电机参数套
- `MotorParams_GetActiveMotor(void)`：获取当前激活电机ID
- `MotorParams_IsMotorEnabled(uint8_t motor_id)`：检查指定电机是否激活
- `MotorParams_IsAnyMotorActive(void)`：检查是否有激活的电机

---

### normalization 模块

**设计思路**
- 采纳 `normalization.h` 注释中的“第二套方案”：使用硬件实际能力作为基值。
- 参考变量：
  - `V_base = V_DC / √3`
  - `I_base = I_limit_actual (无则回退最大或额定值)`
  - `w_base = (2πRPM/60) * Pn`
  - 由此推导 `Fluxb`, `T_base`, `P_base`, `Z_base`, `L_base`, `time_base`。

**实现**
- `normalization_base_values_t`：封装九项基值；内部 `normalization_motor_ctx_t` 维护基值、倒数和有效标记。
- `Normalization_Init`：遍历全部电机调用 `Normalization_UpdateMotor`。
- `Normalization_UpdateMotor`：**只对激活的电机参数套计算基值**，失败时清零上下文。
- `Normalization_GetBases`：**只返回激活电机的归一化基值**，确保状态一致性。
- `get_base_pair`：内部函数，集成激活状态检查，确保只有激活电机能获取基值。
- `Normalization_ToPerUnit()` / `Normalization_FromPerUnit()`：双向标幺转换并限定在 [-1, 1]。
- `Normalization_ToQ31()` / `Normalization_FromQ31()`：基于 `arm_float_to_q31`/`arm_q31_to_float` 提供定点接口。

**使用示例**
```c
// 1. 获取激活电机的归一化基值
const normalization_base_values_t *bases = Normalization_GetBases(MOTOR_0);
if (bases != NULL) {
    printf("电压基值: %.2fV, 电流基值: %.2fA\n",
           bases->voltage_base, bases->current_base);
}

// 2. 物理量转换为标幺值
float actual_current = 2.5f;  // 实际电流 2.5A
float current_pu = Normalization_ToPerUnit(MOTOR_0, NORMALIZE_CURRENT, actual_current);
// current_pu 范围 [-1, 1]

// 3. 标幺值转换为物理量
float voltage_pu = 0.8f;  // 标幺电压 0.8
float actual_voltage = Normalization_FromPerUnit(MOTOR_0, NORMALIZE_VOLTAGE, voltage_pu);

// 4. Q31格式转换（用于定点运算）
q31_t current_q31 = Normalization_ToQ31(MOTOR_0, NORMALIZE_CURRENT, actual_current);
float current_back = Normalization_FromQ31(MOTOR_0, NORMALIZE_CURRENT, current_q31);

// 5. 通用基值转换（不依赖电机状态）
float custom_value = 100.0f;
float custom_base = 200.0f;
float pu_value = Normalization_ToPerUnitWithBase(custom_value, custom_base);
```

---

### FOC_math 模块

**职责**
- 提供磁场定向控制（FOC）相关的数学计算函数，支持Clarke、Park、逆Park变换和SVPWM调制。
- **动态电机ID支持**：自动使用当前激活的电机参数套进行归一化计算。
- **标幺化集成**：所有物理量自动转换为标幺值进行计算，确保数值稳定性和一致性。

**关键实现**
- `foc_math_try_get_motor_id()`：验证并获取当前有效的电机ID，确保有激活电机且归一化基值可用
- `foc_math_to_per_unit()`：将物理量转换为标幺值，使用动态电机ID
- `Clarke_Transform()`：三相电流到α-β坐标系变换
- `Park_Transform()`：α-β到d-q坐标系变换
- `Inverse_Park_Transform()`：d-q到α-β坐标系逆变换
- `SVPWM()`：空间矢量脉宽调制，输出三相PWM占空比
- `Sine_Cosine()`：角度转换为正弦余弦值

**使用示例**
```c
// FOC控制流程示例
// 1. 电流反馈处理
float ia, ib, ic;  // 三相电流反馈
float I_alpha, I_beta;
Clarke_Transform(ia, ib, &I_alpha, &I_beta);  // Clarke变换

// 2. 获取角度信息
float theta_e = get_electrical_angle();  // 电角度
float sin_theta, cos_theta;
Sine_Cosine(theta_e, &sin_theta, &cos_theta);  // 计算正弦余弦

// 3. Park变换
float id_feedback, iq_feedback;
Park_Transform(I_alpha, I_beta, sin_theta, cos_theta, &id_feedback, &iq_feedback);

// 4. PID控制计算
float id_ref = 0.0f;  // d轴电流设定值（通常为0）
float iq_ref = speed_controller_output;  // q轴电流设定值（来自速度环）
float ud = PID_CurrentD_Calculate(MOTOR_0, id_ref, id_feedback, iq_feedback);
float uq = PID_CurrentQ_Calculate(MOTOR_0, iq_ref, iq_feedback, id_feedback);

// 5. 逆Park变换
float U_alpha, U_beta;
Inverse_Park_Transform(ud, uq, sin_theta, cos_theta, &U_alpha, &U_beta);

// 6. SVPWM调制
uint32_t Tcm1, Tcm2, Tcm3;
SVPWM(U_alpha, U_beta, &Tcm1, &Tcm2, &Tcm3);

// 7. 更新PWM寄存器
TIM1->CCR1 = Tcm1;
TIM1->CCR2 = Tcm2;
TIM1->CCR3 = Tcm3;
```

**状态管理集成**
- 移除了固定的 `FOC_MATH_ACTIVE_MOTOR_ID` 宏定义
- 所有FOC计算函数通过 `MotorParams_GetActiveMotor()` 获取当前激活电机ID
- 自动检查激活状态和归一化基值有效性，失败时返回安全默认值

---

### PID_controller 模块

**职责**
- 实现FOC电机控制的PID控制器模块，支持电流环、速度环和位置环控制。
- 提供抗积分饱和保护、电流环解耦、速度前馈等高级功能。
- **多电机支持**：每个电机独立的PID控制器组，支持动态电机ID。
- **统一接口**：所有PID控制器使用相同的计算函数，通过参数配置实现不同控制类型。

**数据结构**
- `pid_controller_t`：通用PID控制器结构体，包含控制参数、状态变量、限幅参数等
- `pid_controller_group_t`：PID控制器组，每个电机包含d轴电流环、q轴电流环、速度环、位置环
- `current_decoupling_t`：电流环解耦参数（ws, Ld, Lq, Flux）
- `speed_feed_forward_t`：速度前馈参数（增益、阈值）

**关键实现**
- **核心PID算法**：`PID_Calculate()` - 通用PID计算函数，支持PI、PD、PID控制
- **电流环控制**：
  - `PID_CurrentD_Calculate()` - d轴电流环PI控制器 + 解耦
  - `PID_CurrentQ_Calculate()` - q轴电流环PI控制器 + 解耦
- **速度环控制**：`PID_Speed_Calculate()` - 速度环PI控制器 + 前馈
- **位置环控制**：`PID_Position_Calculate()` - 位置环PD控制器

**高级功能**
- **抗积分饱和保护**：
  - 积分限幅模式：防止积分项过大
  - 反向计算模式：智能抗积分饱和
  - 可配置抗饱和参数
- **电流环解耦**：
  - d轴解耦：`-ws × Lq × iq_feedback`
  - q轴解耦：`ws × (Ld × id_feedback + Flux)`
  - 支持动态参数更新和使能控制
- **速度前馈控制**：
  - 直接电压前馈：`V_ff = Ke × ω`
  - 低速禁用功能：可设置启用阈值
  - 可调前馈增益

**参数配置接口**
- `PID_SetParameters(motor_id, type, kp, ki, kd)` - 设置Kp, Ki, Kd参数
- `PID_SetOutputLimit(motor_id, type, limit)` - 设置输出限幅
- `PID_SetIntegralLimit(motor_id, type, limit)` - 设置积分限幅
- `PID_SetAntiWindup(motor_id, type, mode, back_calc_gain)` - 配置抗积分饱和模式
- `PID_SetDecoupling(motor_id, ws, Ld, Lq, Flux)` - 设置电流环解耦参数
- `PID_SetSpeedFeedForward(motor_id, feed_forward_gain, omega_threshold)` - 设置速度前馈参数

**控制器管理接口**
- `PID_ResetController(motor_id, type)` - 重置控制器状态
- `PID_EnableController(motor_id, type, enable)` - 启用/禁用控制器
- `PID_GetOutput(motor_id, type)` - 获取控制器输出值
- `PID_GetParameters(motor_id, type, kp, ki, kd)` - 获取控制器参数
- `PID_SetSampleTime(motor_id, dt)` - 设置采样时间

**高级功能使能接口**
- `PID_EnableDecoupling(motor_id, enable)` - 启用/禁用电流环解耦
- `PID_EnableSpeedFeedForward(motor_id, enable)` - 启用/禁用速度前馈

**使用方式**
```c
// 初始化
PID_Init(MOTOR_0);

// 设置控制参数
PID_SetParameters(MOTOR_0, PID_TYPE_CURRENT_D, 0.5f, 10.0f, 0.0f);
PID_SetParameters(MOTOR_0, PID_TYPE_SPEED, 0.1f, 1.0f, 0.0f);

// 控制计算
float ud = PID_CurrentD_Calculate(MOTOR_0, id_ref, id_feedback, iq_feedback);
float uq = PID_CurrentQ_Calculate(MOTOR_0, iq_ref, iq_feedback, id_feedback);
float iq_ref = PID_Speed_Calculate(MOTOR_0, speed_ref, speed_feedback);

// 启用高级功能
PID_EnableDecoupling(MOTOR_0, true);
PID_EnableSpeedFeedForward(MOTOR_0, true);
```

**设计特点**
- **统一接口设计**：通过设置kd=0实现PI控制，kp=0实现I控制
- **归一化兼容**：支持标幺值参数系统
- **实时参数调整**：支持运行时修改控制参数
- **安全保护机制**：内置限幅和抗饱和保护
- **模块化架构**：便于扩展和维护

---

### 初始化与运行流程

1. `main.c` 中完成外设 (`MX_...`) 初始化。
2. 调用 `MotorParams_Init()` 初始化参数和限幅。
3. 随后调用 `Normalization_Init()` 初始化归一化上下文。
4. 调用 `PID_Init(motor_id)` 初始化各电机的PID控制器组。
5. `Command_Init()` 在命令线程/任务启动前调用，之后 `Command_Parse()` 在串口接收回调或任务循环中使用。
6. **状态管理**：通过命令（如 `set motor0 enable`）激活指定电机参数套，自动触发归一化基值更新。
7. **运行时控制**：在FOC控制循环中调用相应的PID计算函数和FOC变换函数。

**完整初始化示例**
```c
int main(void)
{
    // 1. HAL库和外设初始化
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM_Init();
    MX_ADC_Init();
    // ... 其他外设初始化
    
    // 2. MotorControl模块初始化
    MotorParams_Init();           // 初始化电机参数
    Normalization_Init();         // 初始化归一化模块
    
    // 3. PID控制器初始化
    for (uint8_t i = 0; i < motors_number; i++) {
        PID_Init(i);              // 初始化各电机的PID控制器
    }
    
    // 4. 命令解析初始化
    Command_Init();               // 初始化命令解析模块
    
    // 5. 激活默认电机（可选）
    // MotorParams_SetActiveMotor(MOTOR_0);
    // Normalization_UpdateMotor(MOTOR_0);
    
    // 6. 启动实时控制任务
    // osThreadNew(FOC_Control_Task, NULL, &foc_task_attributes);
    
    while (1) {
        // 主循环
    }
}
```

---

### 后续建议与改进方向

**功能扩展**
1. **命令扩展**：
   - 补充查看/配置PID参数的命令（如 `set motor0 Kip=0.5`）
   - 添加实时查看控制器状态的命令（如 `pid status motor0`）
   - 支持保存/加载参数配置到Flash存储
   
2. **HMI参数映射**：
   - 将PID参数（Kip, Kii, Kvp, Kvi, Kvfr等）集成到HMI参数系统中
   - 分配HMI代码（P2XXX系列）用于PID参数访问
   - 支持通过VOFA实时调整控制参数并观察效果

**性能优化**
3. **Q31定点实现**：
   - 针对实时控制需求优化PID计算性能
   - 将高频控制路径（电流环≥10kHz、FOC变换）迁移到Q31定点运算
   - 保留低频路径（速度环、位置环、参数配置）的浮点实现
   
4. **CORDIC硬件加速**：
   - 利用STM32G4的CORDIC协处理器加速三角函数计算
   - 优化`Sine_Cosine()`函数，使用硬件CORDIC替代软件查表
   - 减少sin/cos计算的CPU占用率

**调试与测试**
5. **调试接口增强**：
   - 添加PID控制器状态监控功能（积分项、输出饱和状态等）
   - 实现控制变量的实时波形输出（与VOFA深度集成）
   - 提供性能分析工具（控制周期、CPU占用率统计）
   
6. **参数自动整定**：
   - 考虑实现基于继电反馈法的PID参数自整定
   - 添加系统频率响应分析功能
   - 提供增益裕度和相位裕度检测
   
7. **错误处理增强**：
   - 在归一化基值计算失败时提供详细错误信息和恢复建议
   - 添加电机状态异常检测（失步、过流、过压等）
   - 实现多级保护机制（软件过流保护、电压限幅、温度监控）

**架构优化**
8. **多电机协同控制**：
   - 支持多电机同步控制和相位锁定
   - 实现电机间的负载均衡和转矩分配
   - 添加主从控制模式
   
9. **模块化测试框架**：
   - 为每个模块添加单元测试（使用Unity测试框架）
   - 实现模拟环境下的功能验证
   - 建立HIL（硬件在环）测试平台




### 修改记录

#### 2025-11-08
**PID控制器模块完整实现**

**头文件设计** (`MotorControl/Inc/PID_controller.h`)
- 定义了完整的PID控制器数据结构：`pid_controller_t`、`pid_controller_group_t`
- 支持多控制器类型：电流环(d/q轴)、速度环、位置环
- 定义了抗积分饱和模式：无、积分限幅、反向计算
- 添加了电流环解耦和速度前馈数据结构
- 提供了完整的函数接口声明和默认参数定义

**源文件实现** (`MotorControl/Src/PID_controller.c`)
- 实现了通用PID计算函数 `PID_Calculate()`，支持PI、PD、PID控制
- 实现了电流环控制：`PID_CurrentD_Calculate()`、`PID_CurrentQ_Calculate()`
- 实现了速度环控制：`PID_Speed_Calculate()`，包含前馈功能
- 实现了位置环控制：`PID_Position_Calculate()`
- 实现了抗积分饱和保护机制
- 实现了电流环解耦计算
- 实现了速度前馈控制
- 提供了完整的参数配置和管理接口

**设计特点**
- 统一接口设计：所有控制器使用相同计算函数
- 多电机支持：每个电机独立的PID控制器组
- 实时参数调整：支持运行时修改控制参数
- 安全保护：内置限幅和抗饱和保护
- 归一化兼容：支持标幺值参数系统

#### 2024-11-08
**状态管理机制实现**
- 在 motor_params.h 中添加了全局激活状态变量 g_active_motor_id
- 实现了激活状态管理函数：MotorParams_SetActiveMotor()、MotorParams_DisableMotor()、MotorParams_GetActiveMotor()

**命令模块扩展**
- 在 command.c 中扩展了 handle_set_command() 函数
- 支持大小写不敏感的命令格式：set motor0 enable、set m0 enable、set motor0 disable 等
- 实现了 handle_enable_command() 处理激活/停用逻辑

**归一化模块优化**
- 修改了 Normalization_UpdateMotor() 只对激活参数套计算基值
- 更新了 Normalization_GetBases() 只返回激活电机的归一化基值

**FOC计算模块适配**
- 移除了固定的 FOC_MATH_ACTIVE_MOTOR_ID 宏定义
- 修改了 foc_math_try_get_motor_id() 使用动态激活电机ID

**使用方式**
- 激活参数套：set motor0 enable 或 set m0 enable
- 停用参数套：set motor0 disable 或 set m0 disable
- 支持大小写不敏感和电机ID缩写（m0, m1）



### 技术笔记

#### Q31定点化迁移方案（2024-11-07）

**迁移目标**
将整套FOC流程从浮点运算迁移到Q31定点运算，重点优化高速路径：
- `MotorControl/Src/FOC_math.c` - FOC数学计算
- `MotorControl/Src/normalization.c` - 标幺化转换
- 控制环路（PI/PID、电压调制）

**核心依赖**
- ARM CMSIS-DSP库已提供Q31类型支持
- 向量运算、`arm_circularWrite_f32`等辅助函数可直接复用

**Q31化实施步骤**

1. **物理量标幺化转换**
   - 将循环内频繁计算的物理量转换为标幺值，再转Q31
   - 包括：相电流（Clarke/Park输入输出）、dq轴电压指令、SVPWM零序注入/占空比
   - 接口：`Normalization_ToQ31()` / `Normalization_FromQ31()`

2. **三角函数处理**
   - **CORDIC硬件方案**：使用Q1.31格式（±π）作为输入，Sin/Cos输出为Q1.31
   - **软件方案**：保留`arm_sin_cos_f32()`，在角度环结束后转换sin/cos为Q31参与矩阵运算

3. **保留浮点的模块**
   - 低速、低频或配置数据应保持浮点运算：
     * `motor_params.c` - 参数表
     * `command.c` - 命令解析
     * HMI交互模块
     * 归一化基值计算（涉及浮点除法/平方根，执行频率低）
   - 策略：更新后一次性生成Q31快照供实时控制使用

4. **PID控制器定点化**
   - **完全定点方案**：预先转换增益、积分限幅、前馈系数为Q31或Q15常数，需检查溢出
   - **折中方案**：若PID执行频率仅几百Hz，可保留浮点输入，输出端转换为Q31

5. **验证与测试**
   - ⚠️ 所有定点路径必须重新校验量程与误差
   - 验证步骤：
     1. 识别实时循环内的浮点变量，替换为标幺-Q31
     2. 更新FOC路径（Clarke→Park→调制）为Q31
     3. 若使用CORDIC，确保角度缩放符合Q31格式
     4. 对比仿真/实测误差，必要时加入饱和、移位和舍入策略
     5. 低频模块保留浮点，通过接口给Q31控制器提供初始值

**关键注意事项**
- Q31定点运算需要特别注意数值范围和精度损失
- 建议先在仿真环境验证，再部署到实际硬件
- 保留浮点版本作为参考对照