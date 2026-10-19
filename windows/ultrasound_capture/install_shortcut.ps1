param([string] $Executable = (Join-Path $PSScriptRoot 'dist\PausUltrasound.exe'))
$ErrorActionPreference = 'Stop'
$resolved = (Resolve-Path -LiteralPath $Executable).ProviderPath
$desktop = [Environment]::GetFolderPath('DesktopDirectory')
$shell = New-Object -ComObject WScript.Shell
$shortcut = $shell.CreateShortcut((Join-Path $desktop 'Paus Ultrasound.lnk'))
$shortcut.TargetPath = $resolved
$shortcut.WorkingDirectory = Split-Path -Parent $resolved
$shortcut.Description = 'PAUS ultrasound window capture'
$shortcut.Save()
Write-Output (Join-Path $desktop 'Paus Ultrasound.lnk')
