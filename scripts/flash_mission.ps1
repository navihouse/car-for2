param(
    [ValidateSet('FULL','AC','CB','BD','DA')][string]$Mode = 'FULL',
    [ValidateSet('Debug','Release')][string]$Configuration = 'Debug',
    [switch]$NoBuild,
    # UnderReset = user's known-good command (mode=UR reset=HWrst).
    # Normal     = proven fallback on this board when NRST is not wired to SWD.
    [ValidateSet('UnderReset','Normal','HotPlug')][string]$Connect = 'UnderReset',
    [switch]$NoFallback,
    [switch]$SkipVerify,
    [switch]$DryRun,
    [string]$Programmer,
    [string]$ArmBin = (Join-Path $env:USERPROFILE 'AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin'),
    [string]$Ninja = (Join-Path $env:USERPROFILE 'AppData\Local\stm32cube\bundles\ninja\1.13.2+st.1\bin\ninja.exe')
)
$ErrorActionPreference = 'Stop'

$taskRoot   = Split-Path -Parent $PSScriptRoot
$project    = 'C10B_Test'
$preset     = "Mission-$Configuration"
if ($Mode -eq 'FULL') { $buildDir = Join-Path $taskRoot "build\$preset" }
else { $buildDir = Join-Path $taskRoot "build\$preset-$Mode" }
$elfPath = Join-Path $buildDir "$project.elf"
$hexPath = Join-Path $buildDir "$project.hex"
$binPath = Join-Path $buildDir "$project.bin"

function Find-Programmer {
    if ($Programmer) {
        if (-not (Test-Path -LiteralPath $Programmer)) { throw "Programmer CLI not found: $Programmer" }
        return $Programmer
    }
    $bundleRoot = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer'
    if (Test-Path -LiteralPath $bundleRoot) {
        $found = Get-ChildItem -LiteralPath $bundleRoot -Directory -ErrorAction SilentlyContinue |
            Sort-Object { try { [version]$_.Name } catch { [version]'0.0.0' } } -Descending |
            ForEach-Object { Join-Path $_.FullName 'bin\STM32_Programmer_CLI.exe' } |
            Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
        if ($found) { return $found }
    }
    $ideRoot = Join-Path $env:USERPROFILE 'AppData\Local\stm32cube\bundles'
    $found = Get-ChildItem -LiteralPath $ideRoot -Recurse -Filter 'STM32_Programmer_CLI.exe' -ErrorAction SilentlyContinue |
        Select-Object -First 1 -ExpandProperty FullName
    if ($found) { return $found }
    throw 'STM32_Programmer_CLI.exe not found. Pass -Programmer <path>.'
}

function Get-ConnectArgs {
    param([string]$Kind)
    switch ($Kind) {
        'UnderReset' { return @('-c','port=SWD','mode=UR','reset=HWrst','freq=100') }
        'Normal'     { return @('-c','port=SWD','mode=NORMAL','reset=SWrst','freq=100') }
        'HotPlug'    { return @('-c','port=SWD','mode=HOTPLUG','reset=HWrst','freq=100') }
        default { throw "Unknown connect mode: $Kind" }
    }
}

# 1. Build (or at least link) so the .elf matches the current sources.
if (-not $NoBuild) {
    Write-Output "[1/4] Building $Mode ($Configuration) ..."
    & (Join-Path $PSScriptRoot 'build_mission.ps1') -Mode $Mode -Configuration $Configuration -ArmBin $ArmBin -Ninja $Ninja
    if ($LASTEXITCODE -ne 0) { throw 'Build failed; not flashing.' }
} else {
    Write-Output '[1/4] Skipped build (-NoBuild).'
}

if (-not (Test-Path -LiteralPath $elfPath)) {
    throw "ELF missing: $elfPath  (build first, or check -Mode/-Configuration)"
}

# 2. Make sure the .hex is newer than the .elf.
Write-Output '[2/4] Preparing .hex ...'
$objcopy = Join-Path $ArmBin 'arm-none-eabi-objcopy.exe'
$needHex = (-not (Test-Path -LiteralPath $hexPath)) -or
           ((Get-Item -LiteralPath $hexPath).LastWriteTimeUtc -lt (Get-Item -LiteralPath $elfPath).LastWriteTimeUtc)
if ($needHex) {
    if (-not (Test-Path -LiteralPath $objcopy)) { throw "arm-none-eabi-objcopy missing: $objcopy" }
    & $objcopy -O ihex $elfPath $hexPath
    if ($LASTEXITCODE -ne 0) { throw 'objcopy failed.' }
    & $objcopy -O binary $elfPath $binPath
    Write-Output "      regenerated $(Split-Path -Leaf $hexPath) from the current ELF"
} else {
    Write-Output "      up to date: $(Split-Path -Leaf $hexPath) ($((Get-Item -LiteralPath $hexPath).Length) bytes)"
}

# 3. Flash.
$cli = Find-Programmer
$attempts = if ($NoFallback -or $Connect -eq 'HotPlug') { @($Connect) }
            elseif ($Connect -eq 'UnderReset') { @('UnderReset','Normal') }
            else { @('Normal','UnderReset') }

$downloadArgs = @('-d', $hexPath)
if (-not $SkipVerify) { $downloadArgs += '-v' }
$downloadArgs += '-rst'

$ok = $false
foreach ($kind in $attempts) {
    $cliArgs = @(Get-ConnectArgs -Kind $kind) + $downloadArgs
    $pretty = "$cli " + (($cliArgs | ForEach-Object { if ($_ -match '\s') { "'$_'" } else { $_ } }) -join ' ')
    Write-Output "[3/4] Flash ($kind):"
    Write-Output "      $pretty"
    if ($DryRun) { Write-Output '      (dry run: nothing sent)'; $ok = $true; break }

    # The CLI reports progress on stderr; 'Stop' would abort on the first line.
    $savedEap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try { $log = & $cli @cliArgs 2>&1 | ForEach-Object { "$_" }; $exit = $LASTEXITCODE }
    finally { $ErrorActionPreference = $savedEap }
    $log | ForEach-Object { Write-Output "      $_" }
    $text = $log -join "`n"
    $verified = $text -match 'Download verified successfully'
    $complete = $text -match 'File download complete'
    if ($exit -eq 0 -and ($verified -or $complete -or $SkipVerify)) {
        Write-Output "[3/4] OK: flashed $project.hex via $kind"
        $ok = $true
        break
    }
    Write-Output "[3/4] FAILED via $kind (exit $exit)"
}

if (-not $ok) {
    throw @'
Flash failed for every connect mode.
Check: SWD wiring (SWDIO/SWCLK/GND), 3.3V supply, no STM32CubeIDE debug session holding the target,
and whether NRST is wired (HardwareReset needs NRST; otherwise use -Connect Normal).
'@
}

# 4. Summarise.
$size = (Get-Item -LiteralPath $hexPath).Length
Write-Output "[4/4] Done. hex=$hexPath ($size bytes, $(Get-Date -Format 'HH:mm:ss'))"
Write-Output '      Target was reset; it sits in MISSION / IDLE until a short USER_KEY click.'
