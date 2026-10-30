<#
.SYNOPSIS
  Project 8 - installs the desktop tools used by this repository (idempotent).

.DESCRIPTION
  Installs, if missing:
    - OpenSCAD            (winget OpenSCAD.OpenSCAD)
    - Bambu Studio        (winget Bambulab.BambuStudio)
    - Arduino IDE 2       (winget ArduinoSA.IDE.stable)
    - Mission Planner     (MSI from firmware.ardupilot.org, silent /passive install)
    - Python packages     (pymavlink numpy-stl trimesh)
  Already-installed items are detected (winget list / Get-Command / registry / pip show) and skipped.
  Compatible with Windows PowerShell 5.1 (no && / ternary / ?? operators).

.PARAMETER DryRun
  Print what would be done without installing anything.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\setup_external_tools.ps1 -DryRun
  powershell -ExecutionPolicy Bypass -File .\setup_external_tools.ps1
#>
[CmdletBinding()]
param(
    [switch]$DryRun
)

# 'Continue' (not 'Stop'): in PS 5.1 native-command stderr would otherwise throw NativeCommandError
$ErrorActionPreference = 'Continue'

$MissionPlannerUrl = 'https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.msi'
$PythonPackages    = @('pymavlink', 'numpy-stl', 'trimesh')

# Plan-specified ids are tried first; alternates cover known publisher-id spellings in the winget repo.
$WingetApps = @(
    @{ Name = 'OpenSCAD';     Ids = @('OpenSCAD.OpenSCAD');                       Exe = 'openscad' },
    @{ Name = 'Bambu Studio'; Ids = @('Bambultd.BambuStudio', 'Bambulab.BambuStudio'); Exe = 'bambu-studio' },
    @{ Name = 'Arduino IDE';  Ids = @('ArduinoSA.IDE.stable');                    Exe = 'arduino-ide' }
)

$script:Summary = New-Object System.Collections.ArrayList

function Write-Step([string]$msg) { Write-Host "==> $msg" -ForegroundColor Cyan }
function Add-Result([string]$item, [string]$status) {
    [void]$script:Summary.Add([pscustomobject]@{ Item = $item; Status = $status })
}

function Test-WingetAvailable {
    $cmd = Get-Command winget -ErrorAction SilentlyContinue
    return ($null -ne $cmd)
}

function Test-WingetInstalled([string]$id) {
    if (-not (Test-WingetAvailable)) { return $false }
    $out = & winget list --id $id -e --accept-source-agreements 2>$null | Out-String
    return ($out -match [regex]::Escape($id))
}

function Test-WingetIdExists([string]$id) {
    $out = & winget show --id $id -e --accept-source-agreements 2>$null | Out-String
    return ($LASTEXITCODE -eq 0 -and $out -match [regex]::Escape($id))
}

function Install-WingetApp($app) {
    Write-Step "Checking $($app.Name)"
    if (Get-Command $app.Exe -ErrorAction SilentlyContinue) {
        Write-Host "    already on PATH ($($app.Exe)) - skipping"
        Add-Result $app.Name 'already installed'
        return
    }
    if (-not (Test-WingetAvailable)) {
        Write-Warning "winget not found. Install 'App Installer' from the Microsoft Store, or install $($app.Name) manually."
        Add-Result $app.Name 'SKIPPED (no winget)'
        return
    }
    foreach ($id in $app.Ids) {
        if (Test-WingetInstalled $id) {
            Write-Host "    winget reports $id installed - skipping"
            Add-Result $app.Name 'already installed'
            return
        }
    }
    foreach ($id in $app.Ids) {
        if ($DryRun) {
            Write-Host "    [DryRun] winget install --id $id -e --accept-package-agreements --accept-source-agreements"
            Add-Result $app.Name "would install ($id)"
            return
        }
        if (-not (Test-WingetIdExists $id)) {
            Write-Host "    winget id $id not found, trying next"
            continue
        }
        & winget install --id $id -e --accept-package-agreements --accept-source-agreements
        if ($LASTEXITCODE -eq 0) {
            Add-Result $app.Name "installed ($id)"
        } else {
            Add-Result $app.Name "FAILED (winget exit $LASTEXITCODE)"
        }
        return
    }
    Add-Result $app.Name 'FAILED (no matching winget id)'
}

function Test-MissionPlannerInstalled {
    $paths = @(
        "${env:ProgramFiles(x86)}\Mission Planner\MissionPlanner.exe",
        "$env:ProgramFiles\Mission Planner\MissionPlanner.exe"
    )
    foreach ($p in $paths) {
        if ($p -and (Test-Path $p)) { return $true }
    }
    $keys = @(
        'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*',
        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*'
    )
    foreach ($k in $keys) {
        $hit = Get-ItemProperty $k -ErrorAction SilentlyContinue |
               Where-Object { $_.DisplayName -like 'Mission Planner*' }
        if ($hit) { return $true }
    }
    return $false
}

function Install-MissionPlanner {
    Write-Step 'Checking Mission Planner'
    if (Test-MissionPlannerInstalled) {
        Write-Host '    already installed - skipping'
        Add-Result 'Mission Planner' 'already installed'
        return
    }
    $msi = Join-Path $env:TEMP 'MissionPlanner-latest.msi'
    if ($DryRun) {
        Write-Host "    [DryRun] download $MissionPlannerUrl -> $msi"
        Write-Host "    [DryRun] msiexec /i `"$msi`" /passive /norestart"
        Add-Result 'Mission Planner' 'would install (MSI)'
        return
    }
    try {
        [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
        Write-Host "    downloading to $msi"
        $oldPref = $ProgressPreference
        $ProgressPreference = 'SilentlyContinue'
        Invoke-WebRequest -Uri $MissionPlannerUrl -OutFile $msi -UseBasicParsing
        $ProgressPreference = $oldPref
        Write-Host '    running msiexec (admin prompt may appear)'
        $proc = Start-Process -FilePath 'msiexec.exe' -ArgumentList @('/i', "`"$msi`"", '/passive', '/norestart') -Wait -PassThru
        if ($proc.ExitCode -eq 0 -or $proc.ExitCode -eq 3010) {
            Add-Result 'Mission Planner' 'installed'
        } else {
            Add-Result 'Mission Planner' "FAILED (msiexec exit $($proc.ExitCode))"
        }
    } catch {
        Write-Warning "Mission Planner install failed: $($_.Exception.Message)"
        Add-Result 'Mission Planner' 'FAILED (download/install error)'
    }
}

function Get-PythonCommand {
    foreach ($c in @('python', 'py')) {
        $cmd = Get-Command $c -ErrorAction SilentlyContinue
        if ($cmd -and $cmd.Source -notlike '*WindowsApps*') { return $c }
    }
    return $null
}

function Install-PythonPackages {
    Write-Step 'Checking Python packages'
    $py = Get-PythonCommand
    if (-not $py) {
        Write-Warning 'Python not found on PATH (install Python 3.10+ from python.org or: winget install Python.Python.3.11)'
        Add-Result 'Python packages' 'SKIPPED (no python)'
        return
    }
    foreach ($pkg in $PythonPackages) {
        & $py -m pip show $pkg 2>$null | Out-Null
        if ($LASTEXITCODE -eq 0) {
            Write-Host "    $pkg already installed - skipping"
            Add-Result "pip: $pkg" 'already installed'
            continue
        }
        if ($DryRun) {
            Write-Host "    [DryRun] $py -m pip install $pkg"
            Add-Result "pip: $pkg" 'would install'
            continue
        }
        & $py -m pip install $pkg
        if ($LASTEXITCODE -eq 0) {
            Add-Result "pip: $pkg" 'installed'
        } else {
            Add-Result "pip: $pkg" "FAILED (pip exit $LASTEXITCODE)"
        }
    }
}

# ---------------- main ----------------
if ($DryRun) { Write-Host 'DRY RUN - nothing will be installed.' -ForegroundColor Yellow }

foreach ($app in $WingetApps) {
    try { Install-WingetApp $app } catch {
        Write-Warning "$($app.Name): $($_.Exception.Message)"
        Add-Result $app.Name 'FAILED (exception)'
    }
}
Install-MissionPlanner
try { Install-PythonPackages } catch {
    Write-Warning "Python packages: $($_.Exception.Message)"
    Add-Result 'Python packages' 'FAILED (exception)'
}

Write-Host ''
Write-Step 'Summary'
$script:Summary | Format-Table -AutoSize | Out-String | Write-Host
Write-Host 'Next: open Mission Planner, connect the SpeedyBee F405 V4 over USB, and load config/ardupilot/stage1_bare_5in.param (see docs/wiring_and_bringup_guide.md).'
