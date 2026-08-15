# Flash YFOCV3 with OpenOCD (CMSIS-DAP / SWD), same flow as YFOCV3_ST/flash_firmware.ps1.
# Usage:
#   .\scripts\flash.ps1
#   .\scripts\flash.ps1 -BuildBeforeFlash
#   .\scripts\flash.ps1 -Preset lc-voltage-debug

param(
    [ValidateSet('lc-voltage', 'lc-voltage-debug')]
    [string]$Preset = 'lc-voltage',

    [string]$ElfFile = '',
    [string]$OpenOcdExe = 'D:\OpenOCD-20240916-0.12.0\bin\openocd.exe',
    [string]$OpenOcdScripts = 'D:\OpenOCD-20240916-0.12.0\share\openocd\scripts',
    [string]$ConfigFile = "$PSScriptRoot\openocd-stm32g431.cfg",

    [Alias('Build')]
    [switch]$BuildBeforeFlash,
    [switch]$NoResetRun
)

$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot

function Invoke-Native {
    param(
        [string]$Exe,
        [string[]]$Arguments
    )
    & $Exe @Arguments
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

if ($ElfFile -eq '') {
    $ElfFile = Join-Path $Root "build\$Preset\YFOCV3.elf"
}

if ($BuildBeforeFlash) {
    $buildScript = Join-Path $PSScriptRoot 'build.ps1'
    if (-not (Test-Path -LiteralPath $buildScript)) {
        throw "Build script not found: $buildScript"
    }
    & $buildScript -Preset $Preset
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

if (-not (Test-Path -LiteralPath $OpenOcdExe)) {
    throw "OpenOCD executable not found: $OpenOcdExe"
}
if (-not (Test-Path -LiteralPath $OpenOcdScripts)) {
    throw "OpenOCD scripts directory not found: $OpenOcdScripts"
}
if (-not (Test-Path -LiteralPath $ConfigFile)) {
    throw "OpenOCD config not found: $ConfigFile"
}
if (-not (Test-Path -LiteralPath $ElfFile)) {
    throw "ELF file not found: $ElfFile. Run .\scripts\build.ps1 -Preset $Preset first."
}

$resolvedElf = (Resolve-Path -LiteralPath $ElfFile).Path
$elfForOpenOcd = $resolvedElf.Replace('\', '/')

$commands = @(
    'tcl port disabled',
    'gdb port disabled',
    'init',
    'reset halt',
    'stm32l4x mass_erase 0',
    "program `"$elfForOpenOcd`" verify"
)
if (-not $NoResetRun) {
    $commands += 'reset run'
}
$commands += 'shutdown'

$ocdArgs = @(
    '-s', $OpenOcdScripts,
    '-f', $ConfigFile
)
foreach ($command in $commands) {
    $ocdArgs += @('-c', $command)
}

Write-Host "Preset: $Preset"
Write-Host "ELF: $resolvedElf"
Write-Host "OpenOCD: $OpenOcdExe"
Write-Host "Erase: stm32l4x mass_erase 0"

Invoke-Native -Exe $OpenOcdExe -Arguments $ocdArgs
Write-Host "OK: flashed $resolvedElf"
