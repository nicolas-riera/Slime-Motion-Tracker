<#
.SYNOPSIS
  Builds SlimeVR-lite and registers the driver with SteamVR.

.DESCRIPTION
  1. Configures and builds the CMake project (Release).
  2. Copies build\slimevr_lite to %LOCALAPPDATA%\SlimeVR-lite\driver\slimevr_lite
     (SteamVR keeps the DLL locked while running, so SteamVR must be closed).
  3. Registers that folder with vrpathreg.

  Run from the repository root:  powershell -ExecutionPolicy Bypass -File tools\install.ps1
  Uninstall:                      powershell -ExecutionPolicy Bypass -File tools\install.ps1 -Uninstall
#>
param(
    [switch]$Uninstall,
    [switch]$NoBuild
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$target = Join-Path $env:LOCALAPPDATA 'SlimeVR-lite\driver\slimevr_lite'

function Find-VrPathReg {
    $vrpaths = Join-Path $env:LOCALAPPDATA 'openvr\openvrpaths.vrpath'
    if (Test-Path $vrpaths) {
        $runtime = (Get-Content $vrpaths -Raw | ConvertFrom-Json).runtime | Select-Object -First 1
        $exe = Join-Path $runtime 'bin\win64\vrpathreg.exe'
        if (Test-Path $exe) { return $exe }
    }
    $fallback = "${env:ProgramFiles(x86)}\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe"
    if (Test-Path $fallback) { return $fallback }
    throw 'vrpathreg.exe not found: is SteamVR installed?'
}

if (Get-Process vrserver -ErrorAction SilentlyContinue) {
    throw 'SteamVR is running: close it first (the driver DLL is locked while SteamVR runs).'
}

$vrpathreg = Find-VrPathReg

if ($Uninstall) {
    & $vrpathreg removedriver $target | Out-Null
    Write-Host "Driver unregistered. Files left in $target (delete them if you want)."
    return
}

if (-not $NoBuild) {
    Write-Host 'Building (Release)...'
    cmake -S $root -B (Join-Path $root 'build') -G 'Visual Studio 17 2022' -A x64 | Out-Null
    cmake --build (Join-Path $root 'build') --config Release
    if ($LASTEXITCODE -ne 0) { throw 'build failed' }
}

$package = Join-Path $root 'build\slimevr_lite'
if (-not (Test-Path (Join-Path $package 'bin\win64\driver_slimevr_lite.dll'))) {
    throw "driver package not found in $package"
}

Get-Process slimevr_lite_overlay -ErrorAction SilentlyContinue | Stop-Process -Force
New-Item -ItemType Directory -Force $target | Out-Null
Copy-Item -Recurse -Force (Join-Path $package '*') $target

& $vrpathreg adddriver $target | Out-Null
Write-Host "Installed to $target and registered with SteamVR."
Write-Host 'Make sure the SlimeVR server is not running (both use UDP port 6969), then start SteamVR.'
