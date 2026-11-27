param([string]$Compiler = '')
$ErrorActionPreference = 'Stop'
if (-not $Compiler -or -not (Test-Path -LiteralPath $Compiler)) {
    foreach ($candidate in @('C:\Program Files\MinGW\bin\gcc.exe', 'C:\msys64\ucrt64\bin\gcc.exe')) {
        if (Test-Path -LiteralPath $candidate) { $Compiler = $candidate; break }
    }
    if (-not $Compiler -or -not (Test-Path -LiteralPath $Compiler)) {
        $cmd = Get-Command gcc.exe -ErrorAction SilentlyContinue
        if ($cmd) { $Compiler = $cmd.Source }
    }
}
if (-not $Compiler -or -not (Test-Path -LiteralPath $Compiler)) {
    throw 'gcc.exe not found for host unit tests.'
}
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location $projectRoot
try {
    $flags = @('-std=c11','-Wall','-Wextra','-Werror','-O1','-g','-DUNIT_TEST','-Itests','-ICore/Inc')
    $sources = @('tests/test_firmware.c','Core/Src/icm20602.c','Core/Src/motor.c','Core/Src/pid.c','Core/Src/balance_controller.c','Core/Src/app.c')
    & $Compiler @flags @sources '-lm' '-o' 'tests/test_firmware.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Host test build failed' }
    & 'tests/test_firmware.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
} finally {
    Pop-Location
}
