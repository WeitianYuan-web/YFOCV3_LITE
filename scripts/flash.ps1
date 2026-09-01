# Flash YFOCV3 onto GD32F303 with OpenOCD (CMSIS-DAP / SWD).
# Default OpenOCD is GigaDevice's (gd32f30x flash driver). Stock 0.12 stm32f1x
# mis-probes this chip as 512 KB; mass_erase then times out.
# Usage:
#   .\scripts\flash.ps1
#   .\scripts\flash.ps1 -BuildBeforeFlash
#   .\scripts\flash.ps1 -Preset lc-voltage-debug

param(
    [ValidateSet('lc-voltage', 'lc-voltage-debug')]
    [string]$Preset = 'lc-voltage',

    [string]$ElfFile = '',
    [string]$OpenOcdExe = '',
    [string]$OpenOcdScripts = '',
    [string]$ConfigFile = "$PSScriptRoot\openocd-gd32f303.cfg",

    [Alias('Build')]
    [switch]$BuildBeforeFlash,
    [switch]$NoResetRun
)

$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Gd32OcdHome = 'D:\GD32EB_v1.5.11_Rel\GD32EB_v1.5.11_Rel\GD32EB\plugins\com.gd.tools.openocd_1.0.2.202606101542\Tools\OpenOCD'

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

if ($OpenOcdExe -eq '') {
    $OpenOcdExe = Join-Path $Gd32OcdHome 'bin\openocd.exe'
}
if ($OpenOcdScripts -eq '') {
    $OpenOcdScripts = Join-Path $Gd32OcdHome 'openocd\scripts'
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
    throw "OpenOCD executable not found: $OpenOcdExe (need GigaDevice OpenOCD with gd32f30x flash driver)"
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

# Same flow as F303__C Makefile: program (page-erase as needed) + verify + reset.
# Do not stm32f1x mass_erase.
$commands = @(
    'tcl port disabled',
    'gdb port disabled'
)
if ($NoResetRun) {
    $commands += "program `"$elfForOpenOcd`" verify exit"
} else {
    $commands += "program `"$elfForOpenOcd`" verify reset exit"
}

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
Write-Host "Flash: gd32f30x program+verify (no mass_erase)"

Invoke-Native -Exe $OpenOcdExe -Arguments $ocdArgs
Write-Host "OK: flashed $resolvedElf"
