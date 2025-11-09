# FOC_math.c 与 ARM CMSIS DSP 库函数对比分析

## 概述
本文档分析了 `MotorControl/Src/FOC_math.c` 中的内部计算函数与 `arm_math.h` (ARM CMSIS DSP 库) 中对应函数的存在情况。

## 分析结果

### 1. 基础数学运算函数 - ✅ 大部分存在对应函数

#### Q31 数学运算 (FOC_math.c 第35-117行)
| FOC_math.c 函数 | ARM DSP 对应函数 | 状态 |
|----------------|------------------|------|
| `foc_math_q31_add()` | `arm_add_q31()` | ✅ 存在 |
| `foc_math_q31_sub()` | `arm_sub_q31()` | ✅ 存在 |
| `foc_math_q31_mul()` | `arm_mult_q31()` | ✅ 存在 |
| `foc_math_q31_scale()` | `arm_scale_q31()` | ✅ 存在 |
| `foc_math_q31_clamp()` | 无直接对应 | ⚠️ 需要手动实现 |
| `foc_math_q31_saturate_int64()` | 无直接对应 | ⚠️ 需要手动实现 |

#### 浮点数学运算 (FOC_math.c 第119-176行)
| FOC_math.c 函数 | ARM DSP 对应函数 | 状态 |
|----------------|------------------|------|
| `foc_math_saturate()` | 无直接对应 | ⚠️ 需要手动实现 |
| `foc_math_abs()` | `arm_abs_f32()` | ✅ 存在 |
| `foc_math_min()` | `arm_min_f32()` | ✅ 存在 |
| `foc_math_max()` | `arm_max_f32()` | ✅ 存在 |

### 2. 三角函数 - ✅ 存在对应函数

| FOC_math.c 函数 | ARM DSP 对应函数 | 状态 |
|----------------|------------------|------|
| `Sine_Cosine()` (浮点版) | `arm_sin_cos_f32()` | ✅ 存在 |
| `Sine_Cosine_Q31()` (Q31版) | `arm_sin_cos_q31()` | ✅ 存在 |

### 3. FOC 变换函数 - ✅ 完全存在对应函数

#### Clarke 变换
| FOC_math.c 函数 | ARM DSP 对应函数 | 状态 |
|----------------|------------------|------|
| `Clarke_Transform()` (浮点版) | `arm_clarke_f32()` | ✅ 存在 |
| `Clarke_Transform_Q31()` (Q31版) | `arm_clarke_q31()` | ✅ 存在 |

#### Park 变换
| FOC_math.c 函数 | ARM DSP 对应函数 | 状态 |
|----------------|------------------|------|
| `Park_Transform()` (浮点版) | `arm_park_f32()` | ✅ 存在 |
| `Park_Transform_Q31()` (Q31版) | `arm_park_q31()` | ✅ 存在 |

#### 逆 Park 变换
| FOC_math.c 函数 | ARM DSP 对应函数 | 状态 |
|----------------|------------------|------|
| `Inverse_Park_Transform()` (浮点版) | `arm_inv_park_f32()` | ✅ 存在 |
| `Inverse_Park_Transform_Q31()` (Q31版) | `arm_inv_park_q31()` | ✅ 存在 |

### 4. 其他函数 - ⚠️ 部分需要手动实现

| FOC_math.c 函数 | ARM DSP 对应函数 | 状态 |
|----------------|------------------|------|
| `foc_math_wrap_angle()` | 无直接对应 | ⚠️ 需要手动实现 |
| `foc_math_zero_sequence()` | 无直接对应 | ⚠️ 需要手动实现 |
| `foc_math_pu_to_ticks()` | 无直接对应 | ⚠️ 需要手动实现 |
| `SVPWM()` | 无直接对应 | ⚠️ 需要手动实现 |
| `foc_math_atan2()` | `arm_atan2_f32()` (在快速数学中) | ✅ 可能存在 |

## 优化建议

### 1. 可以替换的函数 (推荐)
以下函数可以直接使用 ARM DSP 库函数替换，以获得更好的性能和硬件优化：

- **基础运算**: `foc_math_q31_add/sub/mul/scale()` → `arm_add/sub/mult/scale_q31()`
- **三角函数**: `Sine_Cosine*()` → `arm_sin_cos_*()`
- **FOC变换**: 所有 Clarke/Park 变换函数

### 2. 需要保留的函数
以下函数由于特定的业务逻辑或缺乏直接对应，需要保留：

- `foc_math_q31_clamp()` - 限幅函数，有特定的业务逻辑
- `foc_math_wrap_angle()` - 角度归一化，电机控制特定需求
- `SVPWM()` - 空间矢量PWM，算法复杂且特定
- `foc_math_pu_to_ticks()` - 标幺值到定时器计数的转换

### 3. 性能优势
使用 ARM DSP 库函数的优势：
- 硬件优化：针对 ARM Cortex-M 处理器优化
- SIMD 指令：利用 DSP 扩展指令集
- 标准化：符合 ARM CMSIS 标准
- 维护性：由 ARM 官方维护和优化

## 结论

`FOC_math.c` 中约有 **70%** 的基础数学运算和 FOC 变换函数可以在 ARM CMSIS DSP 库中找到对应实现。建议替换这些函数以获得更好的性能，同时保留具有特定业务逻辑的函数。

这种替换可以：
1. 提高代码执行效率
2. 减少维护负担
3. 利用硬件加速功能
4. 提高代码标准化程度

## 关于 CORDIC 硬件加速

### 重要发现：ARM CMSIS DSP 库 **不会自动** 使用 STM32G4 的 CORDIC 硬件加速器

根据对源代码的分析：

1. **ARM CMSIS DSP 库的实现方式**：
   - [`arm_sin_f32()`](Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c:71) 使用 **查表法 + 线性插值**
   - [`arm_sin_cos_f32()`](Drivers/CMSIS/DSP/Source/ControllerFunctions/arm_sin_cos_f32.c:74) 使用 **查表法 + 三次插值**
   - 没有任何代码调用 STM32 HAL 的 CORDIC 外设

2. **STM32G4 的 CORDIC 硬件**：
   - 项目中已经初始化了 CORDIC 外设（[`cordic.c`](Core/Src/cordic.c:30)）
   - 但 **目前未被使用**

3. **如何使用 CORDIC 硬件加速**：

   需要手动调用 STM32 HAL 函数，例如：

   ```c
   #include "cordic.h"
   
   // 使用 CORDIC 计算正弦和余弦
   int32_t sin_val, cos_val;
   int32_t angle_q31 = ...; // Q1.31 格式的角度
   
   HAL_CORDIC_Calculate(&hcordic, &angle_q31, &sin_val, 1, 1000);
   HAL_CORDIC_Calculate(&hcordic, &angle_q31, &cos_val, 1, 1000);
   ```

### 性能对比建议

| 方案 | 优点 | 缺点 | 适用场景 |
|-----|------|------|---------|
| **ARM DSP 查表法** | 快速、稳定、经过优化 | 占用ROM空间存储表格 | 对精度要求不高的场景 |
| **CORDIC 硬件** | 不占用ROM、精度高、CPU负载低 | 需要等待硬件计算 | 精度要求高、ROM受限 |
| **自定义实现** | 灵活可控 | 性能可能不如优化库 | 特殊需求场景 |

### 推荐策略

1. **保持 ARM CMSIS DSP 库函数**用于基础数学运算和FOC变换
2. **可选：使用 CORDIC 硬件**替换三角函数计算，特别是在需要高精度的场景
3. **混合使用**：根据实际性能测试结果选择最优方案