cmake_minimum_required(VERSION 3.22)

# 设置项目名称
project(FOC_math_CppUTest_Tests C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置C标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# CppUTest库是用MSVC编译的，所以我们也必须使用MSVC
# 注释掉Clang编译器设置，让CMake自动检测MSVC
# if(EXISTS "D:/Program Files/LLVM/bin/clang.exe")
#     set(CMAKE_C_COMPILER "D:/Program Files/LLVM/bin/clang.exe")
#     set(CMAKE_CXX_COMPILER "D:/Program Files/LLVM/bin/clang++.exe")
#     message(STATUS "Using Clang compiler: ${CMAKE_C_COMPILER}")
# endif()
message(STATUS "Using default MSVC compiler for CppUTest compatibility")

# 编译器特定设置
if(MSVC)
    # MSVC编译器设置
    add_compile_options(/W3 /utf-8)
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

# 手动配置CppUTest路径
set(CPPUTEST_HOME "E:/cpputest-latest" CACHE PATH "CppUTest installation directory")
set(CppUTest_FOUND TRUE)
set(CppUTest_INCLUDE_DIRS "${CPPUTEST_HOME}/include")
# 使用Debug版本的库（CppUTestd.lib）
set(CppUTest_LIBRARIES "${CPPUTEST_HOME}/lib/CppUTestd.lib")
set(CppUTestEXT_LIBRARIES "")  # 扩展库未编译，设为空

# 检查CppUTest是否真的存在
if(EXISTS "${CppUTest_INCLUDE_DIRS}/CppUTest/TestHarness.h" AND EXISTS "${CppUTest_LIBRARIES}")
    message(STATUS "CppUTest found: ${CppUTest_INCLUDE_DIRS}")
    message(STATUS "CppUTest libraries: ${CppUTest_LIBRARIES}")
    
    # 创建CppUTest测试可执行文件
    add_executable(FOC_math_Tests_CPPUTest)
    
    # 添加测试源文件
    target_sources(FOC_math_Tests_CPPUTest PRIVATE
        test_MotorControl/test_FOC_math_cpputest.cpp
        # test_MotorControl/test_Positioning_cpputest.cpp  # 临时注释：PLL 函数未实现
        test_MotorControl/test_motor_simulator.cpp
        test_MotorControl/test_foc_integration.cpp
        test_MotorControl/test_current_loop.cpp
        test_stubs/stm32_hal_stubs.c
        test_stubs/motor_params_stub.c
        test_stubs/motor_simulator.c
        test_stubs/current_stub.c
        test_stubs/pid_controller_stub.c
        test_stubs/positioning_stub.c
    )

    # 添加被测试的源文件
    target_sources(FOC_math_Tests_CPPUTest PRIVATE
        ../MotorControl/Src/FOC_math.c
        ../MotorControl/Src/FOC_Loop.c
        ../MotorControl/Src/normalization.c
        ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.c
        # ../MotorControl/Src/Positioning.c  # 使用 positioning_stub.c 代替（包含PLL实现）
    )
    
    # 添加包含目录
    target_include_directories(FOC_math_Tests_CPPUTest PRIVATE
        test_MotorControl
        test_stubs
        ../MotorControl/Inc
        ../Core/Inc
        ../Drivers/CMSIS/DSP/Include
        ${CppUTest_INCLUDE_DIRS}
    )
    
    # 链接CppUTest库
    target_link_libraries(FOC_math_Tests_CPPUTest PRIVATE
        ${CppUTest_LIBRARIES}
    )
    
    # 为Windows平台添加必要的库
    if(WIN32)
        target_link_libraries(FOC_math_Tests_CPPUTest PRIVATE
            kernel32
            user32
            winmm  # CppUTest需要timeGetTime函数
        )
    endif()
    
    # 添加编译定义
    target_compile_definitions(FOC_math_Tests_CPPUTest PRIVATE
        UNIT_TESTING=1
    )
    
    # 启用测试
    enable_testing()
    add_test(NAME FOC_math_CPPUTest COMMAND FOC_math_Tests_CPPUTest)
    
else()
    message(WARNING "CppUTest not found. Please install CppUTest first.")
    message(STATUS "You can install CppUTest using:")
    message(STATUS "  - Or build from source: https://github.com/cpputest/cpputest")
    
    # 创建一个占位符可执行文件
    add_executable(FOC_math_Tests_CPPUTest placeholder.cpp)
    
    # 创建占位符源文件
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/placeholder.cpp
        "#include <iostream>\n"
        "int main() {\n"
        "    std::cout << \"CppUTest not found. Please install CppUTest first.\" << std::endl;\n"
        "    std::cout << \"Install from: https://github.com/cpputest/cpputest\" << std::endl;\n"
        "    return 1;\n"
        "}\n")
endif()

# 添加数学库
if(UNIX)
    if(CppUTest_FOUND)
        target_link_libraries(FOC_math_Tests_CPPUTest PRIVATE m)
    endif()
endif()

# 自定义目标：运行CppUTest
if(CppUTest_FOUND)
    add_custom_target(run_cpputest
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/FOC_math_Tests_CPPUTest -v
        DEPENDS FOC_math_Tests_CPPUTest
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Running CppUTest with verbose output"
    )
endif()

# 自定义目标：只运行特定测试组
if(CppUTest_FOUND)
    add_custom_target(run_inverse_park_tests
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/FOC_math_Tests_CPPUTest -sg InverseParkTransform
        DEPENDS FOC_math_Tests_CPPUTest
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Running InverseParkTransform test group"
    )
    
    add_custom_target(run_clarke_tests
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/FOC_math_Tests_CPPUTest -sg ClarkeTransform
        DEPENDS FOC_math_Tests_CPPUTest
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Running ClarkeTransform test group"
    )

    add_custom_target(run_motor_simulator_tests
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/FOC_math_Tests_CPPUTest -sg MotorSimulator
        DEPENDS FOC_math_Tests_CPPUTest
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Running MotorSimulator test groups"
    )

    add_custom_target(run_foc_integration_tests
        COMMAND ${CMAKE_CURRENT_BINARY_DIR}/FOC_math_Tests_CPPUTest -sg FOC_OpenLoop_Integration -sg ADC_Integration -sg Load_Disturbance
        DEPENDS FOC_math_Tests_CPPUTest
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Running FOC Integration test groups"
    )
endif()

# 打印构建信息
message(STATUS "=== CppUTest Test Configuration ===")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "CXX Compiler: ${CMAKE_CXX_COMPILER}")
if(CppUTest_FOUND)
    message(STATUS "CppUTest: FOUND")
    message(STATUS "Build directory: ${CMAKE_CURRENT_BINARY_DIR}")
else()
    message(STATUS "CppUTest: NOT FOUND")
endif()
message(STATUS "=====================================")
