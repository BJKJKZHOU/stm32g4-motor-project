cmake_minimum_required(VERSION 3.22)

# 指定使用Clang编译器（如果已安装）
if(EXISTS "D:/Program Files/LLVM/bin/clang.exe")
    set(CMAKE_C_COMPILER "D:/Program Files/LLVM/bin/clang.exe")
    message(STATUS "Using Clang compiler: ${CMAKE_C_COMPILER}")
endif()

# 设置C标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 项目名称
project(FOC_math_Tests C)

# 编译器特定设置
if(MSVC)
    # MSVC编译器设置
    add_compile_options(/W3)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
    set(CMAKE_C_STANDARD 99)
elseif(CMAKE_C_COMPILER_ID MATCHES "Clang")
    # Clang编译器设置
    add_compile_options(
        -Wall                    # 启用所有警告
        -Wno-unknown-pragmas    # 忽略未知的pragma指令
        -Wno-unused-function    # 忽略未使用的函数警告
    )
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
elseif(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    # GCC编译器设置
    add_compile_options(
        -Wall
        -Wno-unknown-pragmas
        -Wno-unused-function
    )
endif()

# 创建测试可执行文件
add_executable(${PROJECT_NAME})

# 添加测试源文件
target_sources(${PROJECT_NAME} PRIVATE
    ../test_MotorControl/test_FOC_math.c
    ../test_stubs/stm32_hal_stubs.c
)

# 添加被测试的源文件
target_sources(${PROJECT_NAME} PRIVATE
    ../../MotorControl/Src/FOC_math.c
)

# 添加包含目录
target_include_directories(${PROJECT_NAME} PRIVATE
    ../test_MotorControl
    ../test_stubs
    ../../MotorControl/Inc
    ../../Core/Inc
    ../../Drivers/CMSIS/DSP/Include
)

# 添加编译定义
target_compile_definitions(${PROJECT_NAME} PRIVATE
    # 为测试环境添加必要的定义
    UNIT_TESTING=1
)

# 为Windows平台添加必要的库
if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        kernel32
        user32
    )
endif()

# 添加数学库
if(UNIX)
    target_link_libraries(${PROJECT_NAME} PRIVATE m)
endif()