param([string]$OutputName = 'Motor_PID_Panel.exe')
$ErrorActionPreference = 'Stop'
$compilerPath = Join-Path $env:WINDIR 'Microsoft.NET\Framework64\v4.0.30319\csc.exe'
if (-not (Test-Path -LiteralPath $compilerPath)) { $compilerPath = Join-Path $env:WINDIR 'Microsoft.NET\Framework\v4.0.30319\csc.exe' }
& $compilerPath /nologo /target:winexe /platform:anycpu /optimize+ /codepage:65001 /warn:4 /warnaserror+ /reference:System.dll /reference:System.Drawing.dll /reference:System.Windows.Forms.dll (('/out:' + (Join-Path $PSScriptRoot $OutputName))) (Join-Path $PSScriptRoot 'Program.cs')
if ($LASTEXITCODE -ne 0) { throw 'GUI compile failed' }
Write-Output "$OutputName build PASS"
