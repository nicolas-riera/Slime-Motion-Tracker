<#
.SYNOPSIS
  Builds the installer: build\SlimeMotionTracker-<version>.msi

.DESCRIPTION
  Builds the project (Release, without hot deploying), then packs build\slime_motion_tracker
  into a per-user MSI with the WiX Toolset (v7; its OSMF EULA must have been accepted:
  wix eula accept wix7). The version comes from kVersion in engine/src/engine.cpp.

  Run from the repository root:  powershell -ExecutionPolicy Bypass -File installer\build_msi.ps1
#>
param([switch]$NoBuild)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$build = Join-Path $root 'build'

$match = Select-String -Path (Join-Path $root 'engine\src\engine.cpp') -Pattern 'kVersion = "([0-9.]+)"'
if (-not $match) { throw 'kVersion not found in engine/src/engine.cpp' }
$version = $match.Matches[0].Groups[1].Value

if (-not $NoBuild) {
    Write-Host 'Building (Release)...'
    cmake -S $root -B $build -G 'Visual Studio 17 2022' -A x64 | Out-Null
    $env:SMT_NO_DEPLOY = '1'
    cmake --build $build --config Release --target driver_package
    Remove-Item Env:SMT_NO_DEPLOY
    if ($LASTEXITCODE -ne 0) { throw 'build failed' }
}

$out = Join-Path $build "SlimeMotionTracker-$version.msi"
wix build (Join-Path $PSScriptRoot 'SlimeMotionTracker.wxs') -arch x64 -d "Version=$version" `
    -bindpath "pkg=$(Join-Path $build 'slime_motion_tracker')" -bindpath "installer=$PSScriptRoot" `
    -bindpath "res=$(Join-Path $root 'overlay\resources')" -pdbtype none -o $out
if ($LASTEXITCODE -ne 0) { throw 'wix build failed' }
Write-Host "Installer: $out"
