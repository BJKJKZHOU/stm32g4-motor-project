# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于 STM32G474RE 的永磁同步电机（PMSM）磁场定向控制（FOC）系统，采用 Eclipse ThreadX RTOS 实现实时控制。项目实现了完整的 FOC 算法栈，包括 Clarke/Park 变换、SVPWM 调制、电流环控制、位置估计等核心功能。

## 构建和测试命令

### 嵌入式固件构建

```bash
# 配置 CMake（首次或清理后）
cmake --preset Debug

# 编译固件
cmake --build build

# 烧录到 STM32（使用 STM32CubeProgrammer）
# cmake --build build --target flash

# 烧录到 STM32（使用 J-Link）
cmake --build build --target flash_jlink
```

### 单元测试（在 PC 上运行）

```powershell
# 进入测试目录
cd test_cpputest

# 运行所有单元测试
.\run_cpputest.ps1

# 运行代码覆盖率分析
.\run_coverage.ps1

# 运行特定测试组（在 build 目录下）
cd build
.\Debug\FOC_math_Tests_CPPUTest.exe -sg FOC_OpenLoop_Integration -sg ADC_Integration -sg Load_Disturbance -v
```

**注意**：单元测试使用 CppUTest 框架，需要在 Windows 上使用 MSVC（Visual Studio 2022）编译器，架构为 Win32（x86）。

## 核心架构

### 1. 标幺化系统（Per-Unit System）

项目采用基于硬件能力的标幺化系统，确保数值稳定性和跨平台移植性：

- **电压标幺化**：基准值为直流母线电压 `V_DC`
- **电流标幺化**：基准值为额定电流 `I_rated`
- **速度标幺化**：基准值为额定转速对应的电角速度
- **所有 FOC 计算在标幺值域进行**，避免浮点数溢出和精度问题

关键模块：`MotorControl/Src/normalization.c`

### 2. FOC 控制流程

完整的 FOC 控制循环在 TIM1 中断中执行（约 10-20 kHz）：

```
ADC采样 → Clarke变换 → Park变换 → PID控制器 → 逆Park变换 → SVPWM → PWM输出
   ↑                        ↑                                    ↓
   └────────────────── 位置/速度估计 ←──────────────────────────┘
```

**关键文件**：
- `MotorControl/Src/FOC_Loop.c` - FOC 主循环
- `MotorControl/Src/FOC_math.c` - Clarke/Park 变换、SVPWM
- `MotorControl/Src/Current.c` - 电流采样和 ADC 触发管理
- `MotorControl/Src/PID_controller.c` - PID/PDFF 控制器

### 3. 硬件加速

项目集成了 STM32G4 的 CORDIC 硬件加速器用于三角函数计算：

- `arm_sin_cos_f32()` 使用 CORDIC 加速（在 `FOC_math.c` 中）
- 显著降低 CPU 负载，提高控制频率

### 4. 多线程架构（ThreadX RTOS）

项目使用 Eclipse ThreadX RTOS 管理多个并发任务：

- **电机控制线程** (`motor_control_threadx.c`) - 高优先级，管理电流环设定值
- **VOFA 通信线程** (`vofa_com_threadx.c`) - 串口数据发送，用于实时波形显示
- **测试线程** (`test_threadx.c`) - 开发调试用途

**中断驱动的实时控制**：FOC 核心循环在 TIM1 中断中执行，不依赖线程调度。

### 5. 电流采样策略

项目实现了动态 ADC 触发点调整，支持高占空比场景：

- **双 ADC 同步采样**：ADC1（Ia、Ib）、ADC2（Ic、Vbus）
- **动态触发点**：根据占空比自动调整采样时刻，避免死区干扰
- **过采样和滤波**：提高电流测量精度

关键函数：`Current_UpdateADCTrigger()` in `Current.c`

### 6. 位置估计

- **IPD（Initial Position Detection）**：脉冲法转子初始定位
- **PLL（Phase-Locked Loop）**：基于反电动势的速度和位置估计（开发中）

关键文件：`MotorControl/Src/Positioning.c`

## 代码风格和约定

### 命名约定

- **函数名**：`PascalCase`（如 `Clarke_Transform`、`FOC_OpenLoopTest`）
- **变量名**：`snake_case`（如 `theta_e`、`id_setpoint`）
- **宏定义**：`UPPER_CASE`（如 `ARR_PERIOD`、`TPWM_PERIOD`）
- **结构体类型**：`PascalCase_t`（如 `CurrentLoop_t`、`PID_Controller_t`）

### 文件组织

- **头文件**：`MotorControl/Inc/` - 所有模块的公共接口
- **实现文件**：`MotorControl/Src/` - 模块实现
- **测试文件**：`test_cpputest/test_MotorControl/` - 单元测试
- **测试桩**：`test_cpputest/test_stubs/` - HAL 和硬件依赖的模拟实现

### 注释规范

- **使用简体中文**注释，详细说明设计意图和约束
- **文件头**：包含文件名、描述、作者、日期
- **函数注释**：说明输入、输出、功能、注意事项
- **关键算法**：提供数学公式和参考文献

### 编码标准

- **C 标准**：C11
- **文件编码**：UTF-8 无 BOM
- **缩进**：2 空格（根据现有代码风格）
- **浮点数后缀**：使用 `f` 后缀（如 `0.5f`、`3.14159f`）

## 关键设计决策

### 为什么使用标幺化？

1. **数值稳定性**：避免不同量纲的数值混合运算导致的精度问题
2. **硬件无关**：标幺值在 [-1, 1] 范围内，便于跨平台移植
3. **饱和保护**：自动限制在物理可行范围内，防止过压/过流

### 为什么在中断中执行 FOC？

1. **实时性保证**：中断优先级高于线程，确保控制周期精确
2. **低延迟**：从 ADC 采样到 PWM 更新的延迟最小化
3. **确定性**：不受 RTOS 调度抖动影响

### 为什么使用 ThreadX？

1. **实时性能**：ThreadX 是经过认证的 RTOS，适合安全关键应用
2. **STM32 官方支持**：STM32CubeMX 原生集成
3. **丰富的中间件**：FileX（文件系统）、USBX（USB 协议栈）

## 添加新功能的步骤

1. **在对应模块目录下添加源文件**（如 `MotorControl/Src/new_feature.c`）
2. **更新 `CMakeLists.txt`**：在 `target_sources` 中添加新文件
3. **编写单元测试**：在 `test_cpputest/test_MotorControl/` 下创建测试文件
4. **更新测试构建配置**：在 `test_cpputest/CMakeLists.txt` 中添加测试源文件
5. **运行测试验证**：`.\run_cpputest.ps1` 确保所有测试通过
6. **更新文档**：在模块 README 或代码注释中说明新功能

## 调试技巧

### 实时数据监控

- **VOFA+ 串口调试**：使用 `plot` 命令启动实时波形显示（波特率 115200）
- **串口命令**：通过 `command.c` 模块查看和修改电机参数
  ```
  motor0          # 查看电机参数
  set motor0 V_DC = 24.0  # 设置直流母线电压
  plot            # 启动数据绘图
  plot stop       # 停止绘图
  ```

### 单元测试驱动开发

- **先在 PC 上验证算法**：使用 CppUTest 框架快速迭代
- **电机仿真器**：`test_stubs/motor_simulator.c` 提供完整的电机物理模型
- **集成测试**：`test_foc_integration.cpp` 验证完整 FOC 控制流程

### 断点调试

- **使用 VSCode + Cortex-Debug 扩展**
- **OpenOCD 配置**：`st_nucleo_g4.cfg`
- **关键断点位置**：
  - `TIM1_UP_IRQHandler` - FOC 中断入口
  - `CurrentLoop_Execute` - 电流环执行
  - `SVPWM` - PWM 调制输出

## 常见问题

### 编译错误：找不到 CORDIC 相关定义

确保在 `stm32g4xx_hal_conf.h` 中启用了 CORDIC 模块：
```c
#define HAL_CORDIC_MODULE_ENABLED
```

### 测试失败：CppUTest 库未找到

1. 检查 `test_cpputest/CMakeLists.txt` 中的 `CPPUTEST_HOME` 路径
2. 确保使用 MSVC（Visual Studio 2022）编译器
3. 确保架构为 Win32（x86），而非 x64

### 电机不转或抖动

1. **检查电机参数**：通过串口命令 `motor0` 查看参数是否正确
2. **验证电流采样**：使用 VOFA+ 观察 `Ia`、`Ib`、`Ic` 波形
3. **检查 PWM 输出**：使用示波器测量三相 PWM 信号
4. **运行开环测试**：`FOC_OpenLoopTest()` 验证基础功能

### 覆盖率报告生成失败

确保安装了 OpenCppCoverage：
```powershell
choco install opencppcoverage
```

## 参考资料

- **STM32G474 参考手册**：详细的外设寄存器说明
- **FOC 算法**：《永磁同步电机矢量控制技术》
- **ThreadX 文档**：https://github.com/eclipse-threadx/rtos-docs
- **CppUTest 文档**：https://cpputest.github.io/

## 项目状态

- ✅ FOC 基础算法实现
- ✅ 标幺化系统
- ✅ CORDIC 硬件加速
- ✅ 电流采样和动态触发
- ✅ IPD 位置估计
- ✅ 串口命令交互
- ✅ 单元测试框架
- ✅ 闭环电流控制
- 🚧 速度环控制（开发中）
- 🚧 位置环控制（规划中）
- 🚧 无感 FOC（规划中）

---

## AI 工作指导

本章节为 Claude Code 提供工作方法和交互模式指导，确保高质量的技术支持。

### 思维模式

在处理技术问题时，应遵循以下思维模式：

1. **系统性分析**：从整体到局部，全面分析项目结构、技术栈和业务逻辑
   - 理解问题在整个系统中的位置
   - 识别相关模块和依赖关系
   - 评估改动的影响范围

2. **前瞻性思维**：考虑技术选型的长远影响
   - 评估可扩展性和可维护性
   - 考虑未来可能的需求变化
   - 避免过度设计，保持适度前瞻

3. **风险评估**：识别潜在的技术风险和性能瓶颈
   - 实时性风险（中断延迟、调度抖动）
   - 数值稳定性（浮点精度、溢出）
   - 硬件限制（CPU 负载、内存占用）
   - 安全性问题（过流、过压、失控）

### 多方案对比分析

对于复杂问题，应提供多种解决方案并进行对比分析：

**标准格式**：

```
方案 A: [方案名称]
优点:
- [优点1]
- [优点2]
缺点:
- [缺点1]
- [缺点2]
适用场景: [具体场景]
实现复杂度: [低/中/高]

方案 B: [方案名称]
优点:
- [优点1]
缺点:
- [缺点1]
适用场景: [具体场景]
实现复杂度: [低/中/高]

推荐: 基于当前项目情况（实时性要求、资源限制等），推荐方案 X，因为...
```

**何时使用多方案对比**：
- 架构设计决策（如控制器结构、数据流设计）
- 算法选择（如位置估计方法、滤波器类型）
- 性能优化方案（如中断优先级调整、DMA 配置）
- 重构方案（如模块拆分、接口设计）

### 实用主义原则

1. **问题导向**：针对实际问题提供解决方案，避免过度设计
   - 优先解决当前最紧迫的问题
   - 避免引入不必要的复杂性
   - 保持代码简洁和可读性

2. **渐进式改进**：在现有基础上逐步优化，避免推倒重来
   - 小步迭代，每次改动可验证
   - 保持系统始终处于可工作状态
   - 通过测试验证每次改进

3. **成本效益平衡**：考虑实现成本和维护成本
   - 评估开发时间和测试工作量
   - 考虑长期维护的复杂度
   - 权衡性能提升与代码复杂度

### 技术解答结构

提供技术解答时，应遵循以下结构：

1. **问题理解**：复述和确认问题，确保理解正确
2. **背景分析**：简要说明相关背景和技术原理
3. **解决方案**：提供详细方案（复杂问题提供多方案对比）
4. **实现步骤**：说明具体实现步骤和注意事项
5. **测试验证**：说明如何验证方案的正确性
6. **潜在风险**：指出可能的风险和应对措施

### 代码实现要求

1. **完整性**：提供完整、可编译的代码示例
2. **注释**：添加必要的中文注释解释关键逻辑和设计意图
3. **测试**：提供对应的单元测试用例
4. **文档**：更新相关文档和注释

### 重要原则

1. **诚实透明**：对不确定的内容明确说明，不进行臆测
   - 明确指出假设和前提条件
   - 承认知识盲区，建议查阅资料
   - 提供可验证的推理过程

2. **安全第一**：优先考虑系统安全性
   - 防止电机失控（过流、过压、过速）
   - 确保实时性要求（中断响应、控制周期）
   - 验证边界条件和异常情况

3. **价值导向**：关注技术方案的实际价值
   - 解决实际问题，而非炫技
   - 提高代码质量和可维护性
   - 提升系统性能和可靠性

4. **测试驱动**：重视测试和验证
   - 先在 PC 上通过单元测试验证算法
   - 使用电机仿真器进行集成测试
   - 在实际硬件上进行最终验证
