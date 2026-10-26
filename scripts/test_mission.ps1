param(
    [string]$Compiler = 'C:/MinGW/bin/gcc.exe',
    [string]$Ninja = 'D:/python311/Scripts/ninja.exe'
)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskBuild = Join-Path $taskRoot 'build\Mission-Host'
& cmake -S (Join-Path $taskRoot 'tests') -B $taskBuild -G Ninja "-DCMAKE_MAKE_PROGRAM=$Ninja" "-DCMAKE_C_COMPILER=$Compiler" -DCMAKE_BUILD_TYPE=Debug
if ($LASTEXITCODE -ne 0) { throw 'Host configure failed.' }
& cmake --build $taskBuild --parallel 4
if ($LASTEXITCODE -ne 0) { throw 'Host build failed.' }
& ctest --test-dir $taskBuild --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'Host tests failed.' }
