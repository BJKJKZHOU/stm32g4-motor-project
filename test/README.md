# FOC_math 单元测试

FOC_math模块的单元测试，用于STM32G4电机控制项目。

---

## 快速开始

### 运行测试

```powershell
cd test
.\run_tests.ps1
```

### 测试流程

- ✅ 配置CMake（首次运行）
- ✅ 使用 Clang 19.1.0 编译器编译
- ✅ 运行单元测试

**测试结果: 19/19 通过 (100.0%)**

---

## 编译器配置

### 当前使用: Clang 19.1.0

- 路径: `\LLVM\bin\clang.exe`
- 构建系统: Ninja（如已安装）或 MinGW Makefiles

---

## 测试其他模块

### 项目模块结构

```
MotorControl/
├── FOC_math.h           # FOC数学变换（已有测试）
├── PID_controller.h     # PID控制器
├── normalization.h      # 归一化模块
├── motor_params.h       # 电机参数
└── command.h           # 命令解析
```

### 推荐测试文件组织

```
test/test_MotorControl/
├── test_FOC_math.c       # FOC数学变换测试
├── test_PID_controller.c # PID控制器测试（新建）
├── test_normalization.c  # 归一化测试（新建）
└── test_motor_params.c   # 电机参数测试（新建）
```

### 创建新模块测试步骤

#### 1. 创建测试文件（伪代码）

```c
// test_PID_controller.c
#include "PID_controller.h"
#include "stm32_hal_stubs.h"

// 测试统计变量
static int tests_run = 0, tests_passed = 0, tests_failed = 0;

// 测试宏
#define TEST_ASSERT(condition, message) // 断言宏定义
#define TEST_ASSERT_FLOAT_EQUAL(expected, actual, tolerance, message) // 浮点比较宏

// 测试函数
void test_pid_basic(void);
void test_pid_integral_limit(void);

int main(void) {
    // 运行所有测试
    test_pid_basic();
    test_pid_integral_limit();
    
    // 输出结果
    return (tests_failed == 0) ? 0 : 1;
}

void test_pid_basic(void) {
    PID_Params_t params = {1.0f, 0.1f, 0.0f, 0.0f, 10.0f, 5.0f};
    PID_State_t state = {0};
    
    float output = PID_Controller(2.0f, 1.0f, 0.001f, &params, &state);
    TEST_ASSERT_FLOAT_EQUAL(1.0f, output, 0.001f, "基本比例控制");
}
```

#### 2. 更新 CMakeLists.txt

```cmake
# 添加PID测试
add_executable(PID_Tests
    test_MotorControl/test_PID_controller.c
    test_stubs/stm32_hal_stubs.c
)

target_include_directories(PID_Tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../MotorControl/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/test_stubs
)
```

#### 3. 运行测试

```powershell
cd test\build
cmake --build . --target PID_Tests
.\PID_Tests.exe
```

---

## 添加新测试到现有模块

### 在 test_FOC_math.c 中添加

```c
// 1. 声明测试函数
void test_my_function(void);

// 2. 实现测试
void test_my_function(void) {
    printf("\n--- Test My Function ---\n");
    
    float result = My_Function(1.0f);
    TEST_ASSERT_FLOAT_EQUAL(expected, result, tolerance, "描述");
}

// 3. 在main()中调用
int main(void) {
    // 现有测试...
    test_my_function(); // 添加新测试
    // 结果统计...
}
```

### 可用的测试宏

```c
TEST_ASSERT(condition, "描述");                    // 布尔测试
TEST_ASSERT_FLOAT_EQUAL(expected, actual, tolerance, "描述"); // 浮点数测试
TEST_ASSERT_Q31_EQUAL(expected, actual, tolerance, "描述");   // Q31定点数测试
```

---

## 系统要求

- **CMake** 3.10+
- **Clang (LLVM)** 19.1.0
- **Windows** 10/11
- **Ninja**（可选，更快的构建系统）

---

---

## 测试最佳实践

- 使用 `test_` 前缀命名测试函数
- 测试边界值和异常情况
- 合理设置容差值
- 断言消息要清晰
- 每个模块一个测试文件

---

