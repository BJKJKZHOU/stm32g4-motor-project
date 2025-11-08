## MotorControl 设计文档

### 总体架构
- MotorControl 子系统负责电机参数管理、归一化（标幺化）计算、命令行交互以及后续的控制器实现。
- 主要模块：命令解析 (`command`)、参数存储 (`motor_params`)、归一化 (`normalization`)、PID 控制器 (`PID_controller`)。
- 所有模块均采用 C 风格全局初始化函数，由 `main.c` 在外设初始化完成后调用。

### 模块概览

| 模块 | 路径 | 主要职责 | 关键依赖 | 主要 API |
| --- | --- | --- | --- | --- |
| command | `MotorControl/Inc/command.h`<br>`MotorControl/Src/command.c` | 解析串口/HMI 命令、打印/修改参数、联动归一化刷新 | `motor_params`, `normalization`, `Vofa_STM32G474` | `Command_Init` `Command_Parse` `Command_GetLastError` |
| motor_params | `MotorControl/Inc/motor_params.h`<br>`MotorControl/Src/motor_params.c` | 保存电机与限幅参数，提供描述符和映射 | `stdio.h` `math.h` | `MotorParams_Init` `MotorParams_SetParam` `MotorParams_PrintAll` |
| normalization | `MotorControl/Inc/normalization.h`<br>`MotorControl/Src/normalization.c` | 基于第二套方案生成基值，提供标幺/Q31 转换 | `arm_math.h` `motor_params.h` | `Normalization_Init` `Normalization_UpdateMotor` `Normalization_ToPerUnit` 等 |
| PID_controller | `MotorControl/Inc/PID_controller.h`<br>`MotorControl/Src/PID_controller.c` | 预留的 PID/PI 控制器实现与参数说明 | 暂无 | 计划提供 PID 相关接口（未实现） |

---

### command 模块

**职责**
- 接收字符命令（如 `plot`, `motor`, `set`），提供参数查看/修改和 VOFA 曲线控制。
- 自动忽略大小写、处理前后空白、支持 `motor0`/`m0` 等多种索引写法。
- 通过 `MotorParams_SetParam` 更新参数后调用 `Normalization_UpdateMotor` 确保基值同步。

**关键实现**
- `Command_Parse`：主解析入口，负责分派到 `handle_plot_command`、`handle_motor_command`、`handle_set_command`。
- 解析过程中维护 `last_error` 缓冲，通过 `Command_GetLastError` 对外报告错误。
- `parse_motor_id_token` 将输入 token 转换为有效电机 ID，并在出错时给出帮助提示。

---

### motor_params 模块


**数据结构**
- `Motor_Params_t`：母线电压、额定电流、Rs、Lq/Ld、额定转速、极对数、Ke、Flux、J、B。
- `Motor_LimitParams_t`：用户/硬件两级限制以及最终生效值（以 `fmin` 合成）。
- `ParamDesc_t` & `ParamMap_t`：参数描述符和指针映射数组，便于通过 HMI 编号或名称访问。

**逻辑**
- `MotorParams_Init`：为 `MOTOR_0` 与 `MOTOR_1` 写入默认参数和限幅，计算 `I_limit_actual`、`speed_limit_actual`。
- `MotorParams_SetParam`：支持 HMI 码（PXXXX）或名称设置参数，变更后输出确认信息。
- `MotorParams_PrintAll`：遍历 `param_maps` 表输出参数、单位与描述。

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
- `Normalization_UpdateMotor`：从 `motor_params`/`motor_limit_params` 读取数据计算基值，失败时清零上下文。
- `Normalization_ToPerUnit` / `Normalization_FromPerUnit`：双向标幺转换并限定在 [-1, 1]。
- `Normalization_ToQ31` / `Normalization_FromQ31`：基于 `arm_float_to_q31`/`arm_q31_to_float` 提供定点接口。

---

### PID_controller 模块

**现状**
- 头文件描述了电流环（PI + 前馈）、速度环（PI/PDFF）等设计思路与参数命名要求。
- `.c` 文件只保留了文件头注释，功能尚未实现。
- 后续需结合归一化结果，将 PID 参数（含 HMI 映射）纳入 `motor_params` 或独立结构。

---

### 初始化与运行流程

1. `main.c` 中完成外设 (`MX_...`) 初始化。
2. 调用 `MotorParams_Init()` 初始化参数和限幅。
3. 随后调用 `Normalization_Init()` 计算全部基值；后续参数变更通过命令模块触发 `Normalization_UpdateMotor()`。
4. `Command_Init()` 在命令线程/任务启动前调用，之后 `Command_Parse()` 在串口接收回调或任务循环中使用。
5. PID 控制器模块待后续补充，对应的初始化应在归一化后进行，确保使用标幺值参数。

---

### 后续建议

1. **完成 PID 控制器实现**：根据头文件计划定义参数结构、HMI 映射、控制算法及与标幺体系的接口。
2. **命令扩展**：补充查看/配置归一化基值、PID 参数的命令，便于调试。
3. **错误处理**：在 `Normalization_UpdateMotor` 失败时，通过日志/命令提示告知参数配置异常。
4. **文档维护**：随着 PID 与更多模块落地，持续更新此说明，以保持与代码同步。






### 笔记
若要把整套FOC流程迁移到Q31，重心在MotorControl/Src/FOC_math.c、MotorControl/Src/  normalization.c和
后续控制环（PI/PID、电压调制）等高速路径。arm_math本身已提供Q31类型、  
向量运算以及arm_circularWrite_f32等辅助，可直接复用。
Q31化的第一步是把所有“循环内频繁计算”的物理量转换成标幺，
再转Q31：相电流（Clarke/Park输入输出）、dq电压指令、  
SVPWM零序注入/占空比等。normalization.c已有Normalization_ToQ31(ToPerUnit)/FromQ31，  
这些接口就是Q31化的桥梁。
角度和三角函数若走CORDIC，推荐Q1.31（±π）输入，Sine/Cosine输出仍是Q1.31，  
再由FOC运算使用；若暂时仍调用arm_sin_cos_f32，可在角度环结束后把sin/cos转换为Q31再参与矩阵运算。
需要保持浮点的主要是低速、低频或配置数据：  
MotorControl/Src/motor_params.c里的参数表、命令解析、  
HMI交互、归一化基值计算（因为涉及浮点除法/平方根，且执行频率低）。  
这些可在更新后一次性生成Q31快照供实时控制使用。
控制器（PI/PID）若要求完全定点，需要把增益、积分限幅、  
前馈系数等都预先转换为Q31或Q15常数，并检查溢出。  
若PID仅在几百Hz级执行，可以保留浮点输入、输出再在末端转换，也是一种折中。
注意：所有定点路径都要重新校验量程与误差。步骤建议：1)   
识别实时循环内的浮点变量并替换为标幺-Q31；2)   
更新FOC路径（Clarke→Park→调制）为Q31；3) 若使用CORDIC，  
则保证角度缩放符合Q31格式；4)   
对比仿真/实测误差，必要时加入饱和、移位和舍入策略；5)   
低频模块保留浮点并通过接口给Q31控制器提供初始值。