# Build YFOCV3 firmware on Windows (CLion cmake/ninja, CubeIDE ARM GCC).
# Usage: .\scripts\build.ps1 [-Preset lc-voltage|lc-voltage-debug]

[CmdletBinding()]
param(
    [ValidateSet('lc-voltage', 'lc-voltage-debug')]
    [string]$Preset = 'lc-voltage'
)

$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot

function Find-Cmake {
    $cmd = Get-Command cmake -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }

    $globs = @(
        'C:\Program Files\JetBrains\CLion *\bin\cmake\win\x64\bin\cmake.exe'
        'C:\Program Files (x86)\JetBrains\CLion *\bin\cmake\win\x64\bin\cmake.exe'
        'D:\Program Files\JetBrains\CLion *\bin\cmake\win\x64\bin\cmake.exe'
        'D:\JetBrains\CLion *\bin\cmake\win\x64\bin\cmake.exe'
        "$env:LOCALAPPDATA\Programs\CLion\bin\cmake\win\x64\bin\cmake.exe"
        'C:\Program Files\CMake\bin\cmake.exe'
        'C:\ST\STM32CubeCLT*\CMake\bin\cmake.exe'
        'D:\ST\STM32CubeCLT*\CMake\bin\cmake.exe'
    )
    $hits = foreach ($g in $globs) {
        Get-Item -Path $g -ErrorAction SilentlyContinue
    }
    if ($hits) {
        return ($hits | Sort-Object FullName -Descending | Select-Object -First 1).FullName
    }
    throw "cmake not found. Install CLion or CMake, or add cmake.exe to PATH."
}

$cmake = Find-Cmake
Write-Host "cmake: $cmake"

Push-Location $Root
try {
    & $cmake --preset $Preset
    if ($LASTEXITCODE -ne 0) { throw "cmake configure failed ($LASTEXITCODE)" }
    & $cmake --build --preset $Preset
    if ($LASTEXITCODE -ne 0) { throw "cmake build failed ($LASTEXITCODE)" }
    Write-Host "OK: $Root\build\$Preset\YFOCV3.elf"
}
finally {
    Pop-Location
}
