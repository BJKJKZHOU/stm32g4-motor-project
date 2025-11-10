# FOC_math Unit Tests / FOC_math 单元测试

FOC_math模块的单元测试，用于STM32G4电机控制项目。

Unit tests for the FOC_math module of the STM32G4 motor control project.

---

## 快速开始 / Quick Start

### 运行测试 / Run Tests

在test目录下打开PowerShell终端，运行：

Open PowerShell in the test directory and run:

```powershell
cd test
.\run_tests.ps1
```

### 测试流程 / Test Process

脚本会自动：

The script will automatically:

- ✅ 配置CMake（首次运行） / Configure CMake (first run only)
- ✅ 使用 **Clang 19.1.0 (LLVM)** 编译器编译 / Compile with **Clang 19.1.0 (LLVM)**
- ✅ 运行19个单元测试 / Run 19 unit tests
- ✅ 显示彩色测试结果 / Display colored test results

**测试结果 / Test Results: 19/19 通过 (100.0%)**

**修改代码后 / After Modifying Code**: 只需再次运行脚本 / Simply run the script again
```powershell
.\run_tests.ps1
```

---

## 编译器配置 / Compiler Configuration

### 当前使用 / Currently Using: Clang 19.1.0

**配置详情 / Configuration Details:**
- 路径 / Path: `D:\Program Files\LLVM\bin\clang.exe`
- 构建系统 / Build System: Ninja（如已安装）或 MinGW Makefiles

**Clang 优势 / Clang Advantages:**
1. ✅ **更友好的错误提示** / Better error messages
2. ✅ **更快的编译速度** / Faster compilation
3. ✅ **更严格的代码检查** / Stricter code analysis
4. ✅ **跨平台兼容性** / Cross-platform compatibility

**编译器状态 / Compiler Status:** ✅ 无警告 / No warnings

### 编译器对比 / Compiler Comparison

| 特性 / Feature | Clang (当前/Current) | MSVC (之前/Previous) |
|----------------|---------------------|---------------------|
| 编译速度 / Speed | ⚡ 快 / Fast | 中等 / Medium |
| 错误提示 / Diagnostics | ✅ 非常友好 / Excellent | 一般 / Average |
| 代码检查 / Analysis | ✅ 严格 / Strict | 一般 / Average |
| 跨平台 / Cross-platform | ✅ 是 / Yes | ❌ 仅Windows / Windows only |
| 安装体积 / Size | 小 / Small | 大（需VS）/ Large (needs VS) |

### 切换编译器 / Switch Compiler

**切换回MSVC / Switch Back to MSVC:**

编辑 `CMakeLists.txt`，注释掉Clang设置：

Edit `CMakeLists.txt` and comment out Clang settings:

```cmake
# 注释掉这几行 / Comment out these lines
# if(EXISTS "D:/Program Files/LLVM/bin/clang.exe")
#     set(CMAKE_C_COMPILER "D:/Program Files/LLVM/bin/clang.exe")
#     message(STATUS "Using Clang compiler: ${CMAKE_C_COMPILER}")
# endif()
```

然后删除build目录重新运行 / Then delete the build directory and rerun:
```powershell
Remove-Item test\build -Recurse -Force
.\run_tests.ps1
```

---

## 目录结构 / Directory Structure

```
test/
├── run_tests.ps1              # 测试脚本 / Test script
├── CMakeLists.txt             # CMake配置（已配置Clang）/ CMake config (Clang enabled)
├── README.md                  # 本文档 / This document
├── test_MotorControl/
│   └── test_FOC_math.c       # 测试用例 / Test cases
└── test_stubs/
    ├── stm32_hal_stubs.h     # HAL桩代码头文件 / HAL stub headers
    ├── stm32_hal_stubs.c     # HAL桩代码实现 / HAL stub implementation
    ├── stm32g4xx_hal.h       # STM32G4 HAL桩 / STM32G4 HAL stub
    └── cordic.h              # CORDIC外设桩 / CORDIC peripheral stub
```

---

## 测试覆盖 / Test Coverage

### 已测试函数 / Tested Functions

1. ✅ **逆Park变换** / Inverse Park Transform - DQ → αβ 坐标变换 / coordinate transformation
2. ✅ **Clarke变换** / Clarke Transform - 三相 → αβ 坐标变换 / Three-phase → αβ transformation
3. ✅ **Park变换** / Park Transform - αβ → DQ 坐标变换 / coordinate transformation
4. ✅ **SVPWM** - 空间矢量脉宽调制 / Space Vector Pulse Width Modulation
5. ✅ **正弦/余弦计算** / Sine/Cosine - CORDIC硬件加速器（所有测试通过 / All tests pass）
6. ✅ **低通滤波器** / Low Pass Filter - 一阶数字滤波器 / First-order digital filter

### 测试统计 / Test Statistics

- **总测试数 / Total Tests**: 19
- **通过 / Passed**: 19
- **失败 / Failed**: 0
- **成功率 / Success Rate**: 100.0%

---

## 测试输出示例 / Sample Test Output

```
========================================
FOC_math Module Unit Tests
========================================

--- Test Inverse Park Transform (Basic) ---
PASS: U_alpha = 1.0 when U_d=1.0, U_q=0.0, theta=0
PASS: U_beta = 0.0 when U_d=1.0, U_q=0.0, theta=0

--- Test Inverse Park Transform (90 degrees) ---
PASS: U_alpha = 0.0 when U_d=1.0, U_q=0.0, theta=90deg
PASS: U_beta = 1.0 when U_d=1.0, U_q=0.0, theta=90deg

...

========================================
Test Results Summary:
Total Tests: 19
Passed: 19
Failed: 0
Success Rate: 100.0%
========================================
```

---

## 硬件抽象 / Hardware Abstraction

测试在PC上运行，使用软件模拟STM32硬件功能：

Tests run on PC with software simulation of STM32 hardware:

- **STM32 HAL函数** / HAL Functions → 桩代码实现 / Stub implementation
- **CORDIC硬件** / CORDIC Hardware → 标准数学库模拟 / Standard math library simulation
- **ARM DSP函数** / ARM DSP → 软件实现 / Software implementation

无需实际STM32硬件！ / No actual STM32 hardware required!

---

## 添加新测试 / Adding New Tests

### 1. 声明测试函数 / Declare Test Function

```c
void test_my_new_function(void);
```

### 2. 实现测试 / Implement Test

```c
void test_my_new_function(void)
{
    printf("\n--- Test My New Function ---\n");
    
    // 测试代码 / Test code
    float result = My_Function(1.0f);
    
    TEST_ASSERT_FLOAT_EQUAL(1.0f, result, 0.001f, "My function returns correct value");
}
```

### 3. 在main函数中调用 / Call from main

```c
int main(void)
{
    // ... 现有测试 / existing tests ...
    test_my_new_function();
    // ... main的其余部分 / rest of main ...
}
```

### 4. 重新运行 / Rerun

```powershell
.\run_tests.ps1
```

---

## 系统要求 / System Requirements

### 必需 / Required

- **CMake** 3.10+ 
  - 下载 / Download: https://cmake.org/download/
- **Clang (LLVM)** 19.1.0
  - 路径 / Path: `D:\Program Files\LLVM\bin\clang.exe`
- **Windows** 10/11

### 可选 / Optional

- **Ninja** - 更快的构建系统 / Faster build system
  ```powershell
  winget install Ninja-build.Ninja
  ```
  或 / Or: https://ninja-build.org/

---

## 常见问题 / FAQ

### Q: PowerShell脚本无法运行 / Script won't run?

**A:** 设置执行策略 / Set execution policy:
```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

### Q: 提示CMake未找到 / CMake not found?

**A:** 安装CMake并添加到PATH / Install CMake and add to PATH:
- https://cmake.org/download/

### Q: 提示Clang未找到 / Clang not found?

**A:** 检查Clang路径 / Check Clang path:
- 应为 / Should be: `D:\Program Files\LLVM\bin\clang.exe`
- 如不同，修改 `CMakeLists.txt` / If different, edit `CMakeLists.txt`

### Q: 所有测试都通过了吗 / Are all tests passing?

**A:** 是的！所有19个测试都通过了，包括之前失败的CORDIC余弦函数测试。

Yes! All 19 tests are now passing, including the previously failing CORDIC cosine test.

### Q: 如何手动构建 / How to build manually?

**A:** 
```bash
cd test
mkdir build && cd build
cmake -G "Ninja" ..    # 或 "MinGW Makefiles" / or "MinGW Makefiles"
cmake --build .
.\FOC_math_Tests.exe
```

---

## 持续集成 / CI/CD Integration

测试脚本设计用于CI/CD集成：

The test script is designed for CI/CD integration:

- 退出代码0 / Exit code 0 = 所有测试通过 / All tests passed
- 退出代码1 / Exit code 1 = 部分测试失败 / Some tests failed
- 每次运行自动清理重建 / Automatic clean rebuild
- 无需手动干预 / No manual intervention required

---

## 已知问题 / Known Issues

1. **字符编码警告** / Character encoding warnings
   - 中文注释文件的C4819警告（无害）
   - C4819 warnings for files with Chinese comments (harmless)

2. **类型转换警告** / Type conversion warnings
   - ARM DSP库的C4244警告（预期的）
   - C4244 warnings from ARM DSP library (expected)

   ✅ **已修复问题 / Fixed Issues:**
   - CORDIC余弦函数精度问题已解决
   - Q31缩放常量溢出问题已修复
   - 编译警告已消除
   - CORDIC cosine precision issue resolved
   - Q31 scale constant overflow fixed
   - Compilation warnings eliminated

---

## 修复历史 / Fix History

### 2025-11-10 - Q31缩放和CORDIC问题修复

**问题 / Issues:**
1. 余弦函数测试失败：`cos(0)` 返回 `-1.0` 而不是 `1.0`
2. 编译警告：隐式类型转换从 `int` 到 `float`

**根本原因 / Root Cause:**
- Q31缩放常量 `2147483648.0f` 超出 `int32_t` 范围导致溢出
- CORDIC存根实现中缺少边界检查

**修复方案 / Fixes Applied:**
1. **FOC_math.c**: 将 `Q31_SCALE_F` 从 `2147483648.0f` 改为 `2147483647.0f`
2. **FOC_math.c**: 添加显式类型转换消除编译警告
3. **stm32_hal_stubs.c**: 添加边界检查防止Q31格式溢出

**结果 / Results:**
- ✅ 测试成功率：94.7% → 100.0%
- ✅ 编译警告：已消除
- ✅ 所有19个测试全部通过

---

## 许可证 / License

本测试套件是STM32G4电机控制项目的一部分。

This test suite is part of the STM32G4 motor control project.

## 作者 / Author

ZHOUHENG - 2025-11-10

---

## 技术支持 / Technical Support

如有问题，请检查：

For issues, please check:

1. Clang是否正确安装 / Clang installed correctly
2. CMake版本是否满足要求 / CMake version meets requirements
3. PowerShell执行策略是否设置 / PowerShell execution policy set
4. 构建目录是否干净 / Build directory is clean

**清理并重新构建 / Clean and rebuild:**
```powershell
Remove-Item test\build -Recurse -Force
cd test
.\run_tests.ps1