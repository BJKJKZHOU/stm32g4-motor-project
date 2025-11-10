# ============================================================================
#   File Name     : run_cpputest.ps1
#   Description   : CppUTest 构建和运行脚本
#   Author        : ZHOUHENG
#   Date          : 2025-11-10
# ============================================================================

Write-Host "========================================"
Write-Host "FOC_math CppUTest Build and Run Script"
Write-Host "Using: MSVC (Visual Studio 2022) + CppUTest"
Write-Host "========================================`n"

# 设置构建目录
$buildDir = "build"

# 检查CppUTest是否已安装
Write-Host "Checking CppUTest installation..."
$vcpkgRoot = $env:VCPKG_ROOT
if (-not $vcpkgRoot) {
    Write-Host "Warning: VCPKG_ROOT environment variable not set." -ForegroundColor Yellow
    Write-Host "CppUTest might not be found automatically.`n" -ForegroundColor Yellow
} else {
    Write-Host "VCPKG_ROOT: $vcpkgRoot" -ForegroundColor Green
}

# 创建构建目录
if (-not (Test-Path $buildDir)) {
    Write-Host "`nCreating build directory..."
    New-Item -ItemType Directory -Path $buildDir | Out-Null
}

# 进入构建目录
Set-Location $buildDir

# 配置CMake
Write-Host "`n========================================"
Write-Host "Configuring CMake..."
Write-Host "========================================`n"

# 构建CMake命令 - 使用Visual Studio 2022 x86架构（匹配CppUTest.lib）
$cmakeArgs = @(
    "-G", "Visual Studio 17 2022",
    "-A", "Win32",
    ".."
)

Write-Host "Using Visual Studio 2022 (MSVC) x86 compiler" -ForegroundColor Green
Write-Host "Note: Using x86 (32-bit) to match CppUTest.lib architecture" -ForegroundColor Yellow

# 执行CMake配置
$cmakeResult = & cmake @cmakeArgs
if ($LASTEXITCODE -ne 0) {
    Write-Host "`nCMake configuration failed!" -ForegroundColor Red
    Write-Host "Please check the error messages above.`n" -ForegroundColor Red
    
    if (-not $vcpkgRoot) {
        Write-Host "Tip: Install CppUTest using vcpkg:" -ForegroundColor Yellow
        Write-Host "  1. Install vcpkg from: https://github.com/microsoft/vcpkg" -ForegroundColor Yellow
        Write-Host "  2. Run: vcpkg install cpputest" -ForegroundColor Yellow
        Write-Host "  3. Set VCPKG_ROOT environment variable`n" -ForegroundColor Yellow
    }
    
    Set-Location ..
    exit 1
}

# 构建项目
Write-Host "`n========================================"
Write-Host "Building project..."
Write-Host "========================================`n"

cmake --build . --config Debug
if ($LASTEXITCODE -ne 0) {
    Write-Host "`nBuild failed!" -ForegroundColor Red
    Set-Location ..
    exit 1
}

# 检查可执行文件是否存在（MSVC输出到Debug子目录）
$exePath = "Debug\FOC_math_Tests_CPPUTest.exe"
if (-not (Test-Path $exePath)) {
    Write-Host "`nTest executable not found: $exePath" -ForegroundColor Red
    Write-Host "Build might have failed or CppUTest is not installed.`n" -ForegroundColor Red
    Set-Location ..
    exit 1
}

# 运行测试
Write-Host "`n========================================"
Write-Host "Running tests..."
Write-Host "========================================`n"

# 运行测试并显示详细输出
& ".\$exePath" -v

$testResult = $LASTEXITCODE

# 返回上级目录
Set-Location ..

# 输出结果
Write-Host "`n========================================"
if ($testResult -eq 0) {
    Write-Host "All tests passed!" -ForegroundColor Green
} else {
    Write-Host "Some tests failed!" -ForegroundColor Red
}
Write-Host "========================================`n"

exit $testResult