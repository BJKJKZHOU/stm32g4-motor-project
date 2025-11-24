# STM32G4 电机控制项目

基于 STM32G474RE 的永磁同步电机（PMSM）磁场定向控制（FOC）系统，采用 ThreadX RTOS 实现实时控制。

## 项目概述

本项目是一个完整的电机控制解决方案，实现了从底层硬件驱动到高层控制算法的全栈开发，支持多电机管理、实时数据监控和串口命令交互。

### 核心特性

- **FOC 算法**：完整的磁场定向控制实现（Clarke/Park 变换、SVPWM 调制）
- **硬件加速**：集成 STM32G4 CORDIC 硬件加速三角函数计算
- **标幺化系统**：基于硬件能力的归一化计算，确保数值稳定性
- **多电机支持**：支持多个独立电机参数套，动态激活/停用
- **实时操作系统**：基于 Eclipse ThreadX RTOS 的多线程架构
- **电流采样**：双 ADC 同步采样，动态触发点调整，支持高占空比场景
- **位置估计**：IPD 脉冲法转子初始定位
- **串口交互**：命令行接口，支持参数查看和修改
- **数据可视化**：集成 VOFA+ 串口调试助手，实时波形显示
- **单元测试**：基于 CppUTest 的完整测试框架，支持代码覆盖率分析

## 技术栈

### 硬件平台
- **MCU**：STM32G474RE（ARM Cortex-M4，170MHz）
- **开发板**：NUCLEO-G474RE
- **外设**：TIM1（PWM 生成）、ADC1/ADC2（电流采样）、CORDIC（硬件加速）、USART（串口通信）

### 软件架构
- **RTOS**：Eclipse ThreadX（Azure RTOS）
- **中间件**：FileX（文件系统）、USBX（USB 协议栈）
- **构建系统**：CMake + ARM GCC
- **调试工具**：OpenOCD / J-Link
- **测试框架**：CppUTest + OpenCppCoverage

### 开发工具
- **IDE**：Visual Studio Code
- **编译器**：ARM GCC（STM32CubeCLT）
- **代码生成**：STM32CubeMX
- **版本控制**：Git
- **串口调试**：VOFA+

## 项目结构

```
stm32g4-motor-project/
├── Core/                       # STM32 HAL 核心代码
│   ├── Src/
│   │   ├── main.c             # 主程序入口
│   │   ├── motor_control_threadx.c  # 电机控制线程
│   │   ├── vofa_com_threadx.c       # VOFA 通信线程
│   │   └── test_threadx.c           # 测试线程
│   └── Inc/                   # 头文件
│
├── MotorControl/              # 电机控制模块（核心）
│   ├── Inc/
│   │   ├── FOC_math.h        # FOC 数学计算
│   │   ├── PID_controller.h  # PID/PDFF 控制器
│   │   ├── motor_params.h    # 电机参数管理
│   │   ├── normalization.h   # 标幺化模块
│   │   ├── Current.h         # 电流采样
│   │   ├── Positioning.h     # 位置估计（IPD）
│   │   ├── FOC_Loop.h        # FOC 控制循环
│   │   └── command.h         # 串口命令解析
│   ├── Src/                  # 实现文件
│   └── README.md             # 模块详细文档
│
├── Communication/             # 串口通信模块
│   ├── Inc/
│   │   ├── Vofa.h            # VOFA 协议
│   │   └── Vofa_STM32G474.h  # STM32 适配层
│   └── Src/
│
├── Drivers/                   # STM32 HAL 驱动库
│   ├── STM32G4xx_HAL_Driver/ # HAL 驱动
│   └── CMSIS/                # CMSIS 库（含 DSP）
│
├── AZURE_RTOS/               # ThreadX RTOS 源码
├── Middlewares/              # 中间件（FileX、USBX）
│
├── test_cpputest/            # 单元测试
│   ├── test_MotorControl/    # 电机控制模块测试
│   ├── test_stubs/           # 测试桩
│   ├── CMakeLists.txt        # 测试构建配置
│   ├── run_cpputest.ps1      # 测试运行脚本
│   └── run_coverage.ps1      # 覆盖率分析脚本
│
├── build/                    # 构建输出目录
├── CMakeLists.txt            # 主构建配置
├── CMakePresets.json         # CMake 预设
├── stm32g4-motor-project-ex.ioc  # STM32CubeMX 配置文件
└── README.md                 # 本文件
```

## 快速开始

### 环境准备

1. **安装开发工具**
   - [Visual Studio Code](https://code.visualstudio.com/)
   - [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html)（包含 ARM GCC 和 CMake）
   - [Git](https://git-scm.com/)
   - [OpenOCD](https://openocd.org/) 或 [J-Link](https://www.segger.com/downloads/jlink/)

2. **克隆仓库**
   ```bash
   git clone <repository-url>
   cd stm32g4-motor-project
   ```

### 编译项目

```bash
# 配置 CMake（首次）
cmake --preset Debug

# 编译
cmake --build build

# 或使用 PowerShell 脚本
cd build
cmake ..
cmake --build .
```

### 烧录程序

**方法 1：使用 STM32CubeProgrammer**
```bash
cmake --build build --target flash
```

**方法 2：使用 J-Link**
```bash
cmake --build build --target flash_jlink
```

**方法 3：使用 OpenOCD**
```bash
openocd -f st_nucleo_g4.cfg -c "program build/stm32g4-motor-project-ex.elf verify reset exit"
```

### 运行测试

```powershell
# 运行所有单元测试
cd test_cpputest
.\run_cpputest.ps1

# 运行代码覆盖率分析
.\run_coverage.ps1
```

## 使用指南

### 1. 硬件连接

- **电机三相**：连接到逆变器输出（U、V、W）
- **电流采样**：ADC1（Ia、Ib）、ADC2（Ic、Vbus）
- **编码器**：（可选）用于位置反馈
- **串口**：USART2（PA2/PA3）连接到 PC

### 2. 参数配置

通过串口命令配置电机参数（波特率：115200）：

```
# 查看电机参数
motor0
m0

# 设置参数
set motor0 V_DC = 24.0
set motor0 I_rated = 5.0
set motor0 Rs = 0.3
set motor0 Lq = 0.001
set motor0 Ld = 0.001
set motor0 Pn = 4
set motor0 RPM_rated = 3000

# 激活电机
set motor0 enable

# 启动 VOFA 数据绘图
plot

# 停止绘图
plot stop
```

### 3. 开环测试

```c
// 在 TIM1 中断中调用
void TIM1_UP_IRQHandler(void) {
    uint32_t Tcm1, Tcm2, Tcm3;
    float frequency = 10.0f;  // 10 rad/s 电频率
    FOC_OpenLoopTest(frequency, &Tcm1, &Tcm2, &Tcm3);

    // 更新 PWM 占空比
    TIM1->CCR1 = Tcm1;
    TIM1->CCR2 = Tcm2;
    TIM1->CCR3 = Tcm3;
}
```

### 4. 闭环控制

详细的 FOC 控制循环示例请参考 [MotorControl/README.md](MotorControl/README.md)。

## 测试覆盖

项目包含完整的单元测试，覆盖核心算法模块：

- **FOC 数学计算**：Clarke/Park 变换、SVPWM 调制
- **电流环测试**：PID 控制器、电流采样
- **电机仿真器**：完整的电机物理模型仿真
- **位置估计**：IPD 脉冲定位算法
- **集成测试**：完整 FOC 控制流程

查看测试覆盖率报告：
```powershell
cd test_cpputest
.\run_coverage.ps1
# 打开 coverage_report/html/index.html
```

## 文档

- **[MotorControl 模块文档](MotorControl/README.md)**：详细的模块设计和 API 说明
- **[设计笔记](MotorControl/部分设计笔记.md)**：开发过程中的技术笔记
- **[测试文档](test_cpputest/README.md)**：单元测试说明

## 开发指南

### 代码风格

- **C 标准**：C11
- **命名约定**：
  - 函数：`PascalCase`（如 `Clarke_Transform`）
  - 变量：`snake_case`（如 `theta_e`）
  - 宏定义：`UPPER_CASE`（如 `ARR_PERIOD`）
- **注释**：使用简体中文，详细说明设计意图和约束
- **文件编码**：UTF-8 无 BOM

### 添加新功能

1. 在对应模块目录下添加源文件
2. 更新 `CMakeLists.txt` 中的 `target_sources`
3. 编写单元测试（`test_cpputest/test_MotorControl/`）
4. 更新模块文档

### 调试技巧

- **VOFA+ 实时波形**：使用 `plot` 命令启动数据流
- **FreeMASTER**：使用 FreeMASTER 
- **断点调试**：使用 VSCode + Cortex-Debug 扩展
- **日志输出**：通过串口输出调试信息
- **单元测试**：先在 PC 上验证算法正确性

## 路线图

- [x] FOC 基础算法实现
- [x] 标幺化系统
- [x] CORDIC 硬件加速
- [x] 电流采样和动态触发
- [x] IPD 位置估计
- [x] 串口命令交互
- [x] 单元测试框架
- [x] 闭环电流控制
- [ ] 速度环控制
- [ ] 位置环控制
- [ ] 位置和速度轨迹规划T/S型
- [ ] 编码器接口
- [ ] 无感 FOC (磁链观测器)
- [ ] 弱磁控制


## 许可证

本项目采用 [MIT License](LICENSE)。

---

**最后更新**：2025-11-20
**项目状态**：开发中（Development）
