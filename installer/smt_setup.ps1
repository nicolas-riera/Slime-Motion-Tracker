<#
.SYNOPSIS
  Installer helper (run by the MSI): registers or unregisters the SteamVR driver.

.DESCRIPTION
  register:   adds the driver folder (the parent of this script's folder) to SteamVR with
              vrpathreg, then tells the user how to start (unless -Quiet).
  unregister: removes it from SteamVR.
  A missing SteamVR is reported, not fatal: the files stay installed and a reinstall
  registers them.
#>
param(
    [Parameter(Mandatory)][ValidateSet('register', 'unregister')][string]$Action,
    [switch]$Quiet
)

$ErrorActionPreference = 'Stop'
$driver = Split-Path -Parent $PSScriptRoot
$title = 'Slime Motion Tracker'

function Show-Message([string]$text, [string]$icon = 'Information') {
    if ($Quiet) { return }
    Add-Type -AssemblyName System.Windows.Forms
    [void][System.Windows.Forms.MessageBox]::Show($text, $title, 'OK', $icon)
}

function Find-VrPathReg {
    $vrpaths = Join-Path $env:LOCALAPPDATA 'openvr\openvrpaths.vrpath'
    if (Test-Path $vrpaths) {
        $runtime = (Get-Content $vrpaths -Raw | ConvertFrom-Json).runtime | Select-Object -First 1
        if ($runtime) {
            $exe = Join-Path $runtime 'bin\win64\vrpathreg.exe'
            if (Test-Path $exe) { return $exe }
        }
    }
    $fallback = "${env:ProgramFiles(x86)}\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe"
    if (Test-Path $fallback) { return $fallback }
    return $null
}

$vrpathreg = Find-VrPathReg
if ($Action -eq 'unregister') {
    if ($vrpathreg) { & $vrpathreg removedriver $driver | Out-Null }
    exit 0
}

if (-not $vrpathreg) {
    Show-Message ("SteamVR was not found, so the driver could not be registered.`n`n" +
        "Install SteamVR and start it once, then run this installer again (Repair).") 'Warning'
    exit 0
}
& $vrpathreg adddriver $driver | Out-Null
Show-Message ("Slime Motion Tracker is installed.`n`n" +
    "1. Make sure the SlimeVR server is not running (both use UDP port 6969).`n" +
    "2. Start SteamVR.`n" +
    "3. Open Slime Motion Tracker from the Start menu to assign and calibrate your trackers.")
exit 0
