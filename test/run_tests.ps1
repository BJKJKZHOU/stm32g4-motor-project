# ============================================================================
#   Script Name   : run_tests.ps1
#   Description   : PowerShell script to build and run FOC_math unit tests
#   Author        : ZHOUHENG
#   Date          : 2025-11-10
# ============================================================================

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "FOC_math Unit Test Build and Run Script" -ForegroundColor Cyan
Write-Host "Using: Clang (LLVM)" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Get script directory
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $ScriptDir

# Check if build directory exists
if (-not (Test-Path "build")) {
    Write-Host "Creating build directory..." -ForegroundColor Yellow
    New-Item -ItemType Directory -Path "build" | Out-Null
}

# Enter build directory
Set-Location "build"

# Copy CMakeLists.txt if not exists or if source is newer
if (-not (Test-Path "CMakeLists.txt")) {
    Write-Host "Copying CMakeLists.txt..." -ForegroundColor Yellow
    Copy-Item "..\CMakeLists.txt" "CMakeLists.txt"
} else {
    $sourceHash = Get-FileHash "..\CMakeLists.txt" -Algorithm MD5
    $destHash = Get-FileHash "CMakeLists.txt" -Algorithm MD5
    if ($sourceHash.Hash -ne $destHash.Hash) {
        Write-Host "CMakeLists.txt has changed, updating..." -ForegroundColor Yellow
        Copy-Item "..\CMakeLists.txt" "CMakeLists.txt" -Force
        Write-Host "Cleaning old build files..." -ForegroundColor Yellow
        if (Test-Path "CMakeCache.txt") { Remove-Item "CMakeCache.txt" }
        if (Test-Path "CMakeFiles") { Remove-Item "CMakeFiles" -Recurse -Force }
    }
}

# Configure with CMake (only if needed)
if (-not (Test-Path "CMakeCache.txt")) {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Cyan
    Write-Host "Configuring with CMake..." -ForegroundColor Cyan
    Write-Host "========================================" -ForegroundColor Cyan
    
    # 检查是否安装了Ninja
    $ninjaExists = Get-Command ninja -ErrorAction SilentlyContinue
    if ($ninjaExists) {
        Write-Host "Using Ninja build system" -ForegroundColor Green
        cmake -G "Ninja" .
    } else {
        Write-Host "Ninja not found, using MinGW Makefiles" -ForegroundColor Yellow
        cmake -G "MinGW Makefiles" .
    }
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host ""
        Write-Host "ERROR: CMake configuration failed!" -ForegroundColor Red
        Write-Host "Please check:" -ForegroundColor Red
        Write-Host "  1. CMake is installed and in PATH" -ForegroundColor Red
        Write-Host "  2. Clang is installed at D:\Program Files\LLVM\bin" -ForegroundColor Red
        Write-Host "  3. Ninja or MinGW is installed (optional)" -ForegroundColor Red
        Set-Location ..
        Read-Host "Press Enter to exit"
        exit 1
    }
} else {
    Write-Host "CMake cache exists, skipping configuration..." -ForegroundColor Green
}

# Build the project
Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Building project..." -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
cmake --build . --config Debug --clean-first
if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "ERROR: Build failed!" -ForegroundColor Red
    Set-Location ..
    Read-Host "Press Enter to exit"
    exit 1
}

# Run the tests
Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Running tests..." -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
# Clang/Ninja不使用Debug子目录
if (Test-Path "FOC_math_Tests.exe") {
    & ".\FOC_math_Tests.exe"
    $TestResult = $LASTEXITCODE
    Set-Location ..
} elseif (Test-Path "Debug\FOC_math_Tests.exe") {
    Set-Location "Debug"
    & ".\FOC_math_Tests.exe"
    $TestResult = $LASTEXITCODE
    Set-Location ..\..
    
    Write-Host ""
    if ($TestResult -eq 0) {
        Write-Host "========================================" -ForegroundColor Green
        Write-Host "All tests passed successfully!" -ForegroundColor Green
        Write-Host "========================================" -ForegroundColor Green
    } else {
        Write-Host "========================================" -ForegroundColor Yellow
        Write-Host "Some tests failed! Exit code: $TestResult" -ForegroundColor Yellow
        Write-Host "========================================" -ForegroundColor Yellow
    }
    
    Read-Host "Press Enter to exit"
    exit $TestResult
} else {
    Write-Host "ERROR: Test executable not found!" -ForegroundColor Red
    Set-Location ..\..
    Read-Host "Press Enter to exit"
    exit 1
}