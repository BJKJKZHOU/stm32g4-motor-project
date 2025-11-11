# ============================================================================
#   File Name     : build_only.ps1
#   Description   : CppUTest 自动构建脚本（静默版）
#   Author        : ZHOUHENG
#   Date          : 2025-11-11
#   命令          ：cd test_cpputest && .\build_only.ps1 && cd ..   
# ============================================================================

# 设置构建目录
$buildDir = "build"

# 创建构建目录
if (-not (Test-Path $buildDir)) {
    New-Item -ItemType Directory -Path $buildDir | Out-Null
}

# 进入构建目录
Set-Location $buildDir

# 配置CMake
$cmakeArgs = @(
    "-G", "Visual Studio 17 2022",
    "-A", "Win32",
    ".."
)

# 执行CMake配置
& cmake @cmakeArgs | Out-Null
if ($LASTEXITCODE -ne 0) {
    Set-Location ..
    exit 1
}

# 构建项目
cmake --build . --config Debug | Out-Null
if ($LASTEXITCODE -ne 0) {
    Set-Location ..
    exit 1
}

# 检查可执行文件是否存在
$exePath = "Debug\FOC_math_Tests_CPPUTest.exe"
if (-not (Test-Path $exePath)) {
    Set-Location ..
    exit 1
}

# 返回上级目录
Set-Location ..

exit 0