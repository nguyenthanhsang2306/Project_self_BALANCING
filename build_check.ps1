param([string]$Compiler = 'C:\ST\STM32CubeIDE_1.18.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin\arm-none-eabi-gcc.exe')
$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $Compiler)) {
    $cmd = Get-Command arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue
    if ($cmd) {
        $Compiler = $cmd.Source
    } else {
        $patterns = @(
            'D:\STM32CubeIDE*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*\tools\bin\arm-none-eabi-gcc.exe',
            'C:\ST\*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*\tools\bin\arm-none-eabi-gcc.exe'
        )
        foreach ($pattern in $patterns) {
            $found = Get-Item -Path $pattern -ErrorAction SilentlyContinue | Select-Object -First 1
            if ($found) { $Compiler = $found.FullName; break }
        }
    }
}
if (-not (Test-Path -LiteralPath $Compiler)) {
    throw "arm-none-eabi-gcc.exe not found. Please open Project_self_tuning in STM32CubeIDE and press Build."
}
$projectRoot = $PSScriptRoot
$buildDir = Join-Path $projectRoot 'build_check'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$flags = @('-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-std=gnu11','-O1','-g3','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror','-DSTM32L476xx','-DUSE_HAL_DRIVER','--specs=nano.specs')
foreach ($include in @('Core/Inc','Drivers/STM32L4xx_HAL_Driver/Inc','Drivers/STM32L4xx_HAL_Driver/Inc/Legacy','Drivers/CMSIS/Device/ST/STM32L4xx/Include','Drivers/CMSIS/Include')) {
    $flags += '-I' + (Join-Path $projectRoot $include)
}
$sources = @(Get-ChildItem -LiteralPath (Join-Path $projectRoot 'Core/Src') -Filter '*.c')
$sources += Get-ChildItem -LiteralPath (Join-Path $projectRoot 'Drivers/STM32L4xx_HAL_Driver/Src') -Filter '*.c'
$sources += Get-ChildItem -LiteralPath (Join-Path $projectRoot 'Core/Startup') -Filter '*.s'
$objects = @()
foreach ($source in $sources) {
    $object = Join-Path $buildDir ($source.BaseName + '.o')
    & $Compiler @flags -c $source.FullName -o $object
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $($source.Name)" }
    $objects += $object
}
$elf = Join-Path $buildDir 'Project_self_tuning.elf'
$map = Join-Path $buildDir 'Project_self_tuning.map'
& $Compiler @flags @objects '-T' (Join-Path $projectRoot 'STM32L476RGTX_FLASH.ld') '--specs=nosys.specs' '-Wl,--gc-sections' "-Wl,-Map=$map" '-Wl,--start-group' '-lc' '-lm' '-Wl,--end-group' '-o' $elf
if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
Write-Output "ARM compile and link PASS: $elf"

$toolDir = Split-Path -Parent $Compiler
& (Join-Path $toolDir 'arm-none-eabi-objcopy.exe') -O ihex $elf (Join-Path $buildDir 'Project_self_tuning.hex')
if ($LASTEXITCODE -ne 0) { throw 'HEX failed' }
& (Join-Path $toolDir 'arm-none-eabi-objcopy.exe') -O binary $elf (Join-Path $buildDir 'Project_self_tuning.bin')
if ($LASTEXITCODE -ne 0) { throw 'BIN failed' }
& (Join-Path $toolDir 'arm-none-eabi-size.exe') $elf

# Keep ready-to-flash files together, separate from intermediate objects.
$firmwareDir = Join-Path $projectRoot 'Firmware'
New-Item -ItemType Directory -Force -Path $firmwareDir | Out-Null
foreach ($extension in @('elf','hex','bin')) {
    Copy-Item -LiteralPath (Join-Path $buildDir ('Project_self_tuning.' + $extension)) -Destination $firmwareDir -Force
}
Remove-Item -LiteralPath $buildDir -Recurse -Force -ErrorAction SilentlyContinue
Write-Output "Firmware ready: $firmwareDir"
