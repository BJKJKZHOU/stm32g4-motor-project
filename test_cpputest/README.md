# FOC_math CppUTest 单元测试

使用CppUTest框架的FOC_math模块单元测试，提供更强大的测试功能。

---

## 🚀 快速开始

### 前提条件

1. **安装CppUTest**
```powershell
# 使用vcpkg安装CppUTest
vcpkg install cpputest
vcpkg integrate install
```

2. **确保已安装的工具**
- CMake 3.22+
- Clang (LLVM) 19.1.0
- Ninja（可选，推荐）

### 运行测试

```powershell
cd test_cpputest
.\run_cpputest.ps1
```

---

## 📁 项目结构

```
test_cpputest/
├── CMakeLists.txt                          # CMake配置文件
├── run_cpputest.ps1                        # 测试运行脚本
├── README.md                               # 本文档
├── test_MotorControl/
│   └── test_FOC_math_cpputest.cpp         # CppUTest测试用例
├── test_stubs/
│   ├── cordic.h                           # CORDIC硬件模拟
│   ├── stm32g4xx_hal.h                    # HAL头文件桩
│   ├── stm32_hal_stubs.h                  # HAL桩声明
│   └── stm32_hal_stubs.c                  # HAL桩实现
└── build/                                 # 构建输出目录（自动生成）
```

---


## 🧪 CppUTest 测试编写

### 基本测试结构

```cpp
// 定义测试组
TEST_GROUP(InverseParkTransform)
{
    void setup() {
        // 每个测试前执行
        // 初始化测试数据
    }
    
    void teardown() {
        // 每个测试后执行
        // 清理资源
        mock().clear();
    }
};

// 编写测试用例
TEST(InverseParkTransform, BasicCase)
{
    float U_alpha, U_beta;
    
    Inverse_Park_Transform(1.0f, 0.0f, 0.0f, 1.0f, &U_alpha, &U_beta);
    
    DOUBLES_EQUAL(1.0f, U_alpha, 0.001f);
    DOUBLES_EQUAL(0.0f, U_beta, 0.001f);
}
```

### 常用断言宏

```cpp
// 布尔断言
CHECK(condition);
CHECK_TRUE(condition);
CHECK_FALSE(condition);

// 数值比较
LONGS_EQUAL(expected, actual);
DOUBLES_EQUAL(expected, actual, tolerance);
UNSIGNED_LONGS_EQUAL(expected, actual);

// 字符串比较
STRCMP_EQUAL(expected, actual);
STRNCMP_EQUAL(expected, actual, length);

// 指针检查
POINTERS_EQUAL(expected, actual);
CHECK_NULL(pointer);

// 失败断言
FAIL("Test failed message");
```

---

## ⚙️ 高级功能

### 1. 运行特定测试组

```powershell
# 进入构建目录
cd test_cpputest/build

# 运行特定测试组
.\FOC_math_Tests_CPPUTest.exe -sg InverseParkTransform

# 运行特定测试
.\FOC_math_Tests_CPPUTest.exe -sn BasicCase
```

### 2. 使用CMake自定义目标

```powershell
# 运行所有测试（详细输出）
cmake --build build --target run_cpputest

# 只运行逆帕克变换测试
cmake --build build --target run_inverse_park_tests

# 只运行克拉克变换测试
cmake --build build --target run_clarke_tests
```

### 3. 测试输出控制

```powershell
# 详细输出
.\FOC_math_Tests_CPPUTest.exe -v

# 彩色输出
.\FOC_math_Tests_CPPUTest.exe -c

# 只显示失败的测试
.\FOC_math_Tests_CPPUTest.exe -v -ojunit

# 生成JUnit格式报告
.\FOC_math_Tests_CPPUTest.exe -ojunit > test_results.xml
```

---

## 📝 添加新测试用例

### 步骤1：在测试文件中添加测试组

```cpp
// 在 test_FOC_math_cpputest.cpp 中添加

TEST_GROUP(NewFeature)
{
    void setup() {
        // 测试前初始化
    }
    
    void teardown() {
        // 测试后清理
    }
};

TEST(NewFeature, TestCase1)
{
    // 测试代码
    CHECK(condition);
}

TEST(NewFeature, TestCase2)
{
    // 另一个测试
    DOUBLES_EQUAL(expected, actual, 0.001f);
}
```

### 步骤2：重新编译运行

```powershell
cd test_cpputest
.\run_cpputest.ps1
```

---


---

## 📊 测试覆盖的功能

### ✅ 已实现测试

- **逆帕克变换 (InverseParkTransform)**
  - 基本测试（0度）
  - 90度测试
  - 45度测试
  - 负电压测试
  - 双分量测试

- **克拉克变换 (ClarkeTransform)**
  - 基本测试
  - 零输入测试

- **帕克变换 (ParkTransform)**
  - 基本测试

- **SVPWM**
  - 零输入测试

- **正弦余弦计算 (SineCosine)**
  - 零角度测试

- **低通滤波器 (LowPassFilter)**
  - 基本功能测试

---

## 🔄 与原测试框架的关系

本CppUTest测试目录与原始的 `test/` 目录是**互补关系**：

- **test/**：轻量级，无依赖，适合快速验证
- **test_cpputest/**：功能丰富，适合完整测试套件

两个测试框架可以**并存**，根据需要选择使用。



**最后更新**: 2025-11-10