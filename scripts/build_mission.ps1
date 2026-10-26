param(
    [ValidateSet('Debug','Release')][string]$Configuration = 'Debug',
    [ValidateSet('FULL','AC','CB','BD','DA')][string]$Mode = 'FULL',
    [string]$ArmBin = (Join-Path $env:USERPROFILE 'AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin'),
    [string]$Ninja = (Join-Path $env:USERPROFILE 'AppData\Local\stm32cube\bundles\ninja\1.13.2+st.1\bin\ninja.exe')
)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskPreset = "Mission-$Configuration"
if ($Mode -eq 'FULL') { $taskBuild = Join-Path $taskRoot "build\$taskPreset" }
else { $taskBuild = Join-Path $taskRoot "build\$taskPreset-$Mode" }
if (-not (Test-Path -LiteralPath (Join-Path $ArmBin 'arm-none-eabi-gcc.exe'))) { throw "ARM toolchain missing: $ArmBin" }
if (-not (Test-Path -LiteralPath $Ninja)) { throw "Ninja missing: $Ninja" }
$taskOldPath = $env:PATH
Push-Location -LiteralPath $taskRoot
try {
    $env:PATH = "$ArmBin;$(Split-Path -Parent $Ninja);$taskOldPath"
    & cmake --preset $taskPreset -B $taskBuild "-DMISSION_MODE=$Mode" "-DCMAKE_MAKE_PROGRAM:FILEPATH=$Ninja"
    if ($LASTEXITCODE -ne 0) { throw 'ARM configure failed.' }
    & cmake --build $taskBuild --parallel 4
    if ($LASTEXITCODE -ne 0) { throw 'ARM build failed.' }
    Write-Output "Firmware: $(Join-Path $taskBuild 'C10B_Test.elf')"
} finally {
    $env:PATH = $taskOldPath
    Pop-Location
}
