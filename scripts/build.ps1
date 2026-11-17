# ESP-IDF 构建脚本（不依赖 VS Code 扩展）
#
# 用法：
#   .\scripts\build.ps1              编译
#   .\scripts\build.ps1 flash        编译 + 烧录
#   .\scripts\build.ps1 flash monitor  编译 + 烧录 + 串口监视
#   .\scripts\build.ps1 menuconfig   打开配置菜单
#   .\scripts\build.ps1 fullclean    清理
#
# 这个脚本自己设置所有环境变量，所以不依赖 ESP-IDF 扩展、
# 也不依赖系统 PATH 里有没有 cmake/ninja。

param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$IdfArgs = @('build')
)

$ErrorActionPreference = 'Stop'

# 让中文正常显示（脚本本身以 UTF-8 BOM 保存，PowerShell 5.1 才能正确解码）
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch {}

# git-bash / MSYS 的环境会干扰 idf_tools.py，先清掉
foreach ($v in 'MSYSTEM','MSYSCON','MINGW_PREFIX','MINGW_CHOST','MSYS2_PATH_TYPE') {
    Remove-Item "Env:$v" -ErrorAction SilentlyContinue
}

$env:IDF_PATH            = 'C:\esp\v6.1.1\esp-idf'
$env:IDF_TOOLS_PATH      = 'C:\Espressif\tools'
$env:IDF_PYTHON_ENV_PATH = 'C:\Espressif\tools\python\v6.1.1\venv'
$env:ESP_IDF_VERSION     = '6.1.1'
$env:ESP_ROM_ELF_DIR     = 'C:\Espressif\tools\esp-rom-elfs\20260528\'

$toolPath = @(
    'C:\Espressif\tools\riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin'
    'C:\Espressif\tools\cmake\4.0.3\bin'
    'C:\Espressif\tools\ninja\1.12.1'
    'C:\Espressif\tools\idf-exe\1.0.3'
) -join ';'

$env:PATH = ($toolPath + ';' + $env:PATH)

$projectDir = Split-Path -Parent $PSScriptRoot
$idfPy      = 'C:\Espressif\tools\python\v6.1.1\venv\Scripts\python.exe'
$idfScript  = 'C:\esp\v6.1.1\esp-idf\tools\idf.py'

Write-Host "[build] 工程: $projectDir" -ForegroundColor Cyan
Write-Host "[build] 动作: $($IdfArgs -join ' ')" -ForegroundColor Cyan

& $idfPy $idfScript -C $projectDir @IdfArgs
exit $LASTEXITCODE
