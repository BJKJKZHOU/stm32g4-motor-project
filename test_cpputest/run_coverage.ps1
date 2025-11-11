# ============================================================================
#   File Name     : run_coverage.ps1
#   Description   : OpenCppCoverage 代码覆盖率报告生成脚本 - 仅覆盖MotorControl文件夹
#   Author        : ZHOUHENG
#   Date          : 2025-11-11
# ============================================================================

# 检查 OpenCppCoverage 是否安装
$OpenCppCoveragePath = "OpenCppCoverage"
if (-not (Get-Command $OpenCppCoveragePath -ErrorAction SilentlyContinue)) {
    Write-Error "错误: 未找到 OpenCppCoverage，请先安装。"
    exit 1
}

# 设置构建目录和可执行文件路径
$buildDir = "build"
$exePath = "Debug\FOC_math_Tests_CPPUTest.exe"
$fullExePath = Join-Path $buildDir $exePath

# 检查可执行文件是否存在
if (-not (Test-Path $fullExePath)) {
    Write-Error "错误: 测试可执行文件 '$fullExePath' 不存在。"
    exit 1
}

# 设置覆盖率报告目录
$coverageDir = "coverage_report"
$htmlReportDir = Join-Path $coverageDir "html"

# 创建覆盖率报告目录
if (Test-Path $coverageDir) {
    Remove-Item -Recurse -Force $coverageDir
}
New-Item -ItemType Directory -Path $coverageDir | Out-Null
New-Item -ItemType Directory -Path $htmlReportDir | Out-Null

# 构建 OpenCppCoverage 命令参数
$coverageArgs = @(
    "--sources", "MotorControl",
    "--excluded_sources", "*test*",
    "--excluded_sources", "*stub*", 
    "--excluded_sources", "AZURE_RTOS",
    "--excluded_sources", "FileX",
    "--excluded_sources", "USBX",
    "--excluded_sources", "Core",
    "--excluded_sources", "Communication", 
    "--excluded_sources", "Drivers",
    "--excluded_sources", "cmake",
    "--export_type", "html:$htmlReportDir",
    "--cover_children",
    $fullExePath,
    "-v"
)

# 运行 OpenCppCoverage
$coverageResult = & $OpenCppCoveragePath @coverageArgs 2>&1
$coverageExitCode = $LASTEXITCODE

if ($coverageExitCode -eq 0) {
    Write-Host "成功: 代码覆盖率报告生成完成"
} else {
    Write-Error "失败: 代码覆盖率报告生成失败"
    $errorMessage = $coverageResult | Out-String
    Write-Error $errorMessage.Trim()
}

exit $coverageExitCode