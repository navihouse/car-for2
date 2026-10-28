param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$BuildDir = 'build\Mission-Debug',
    [string]$ProgrammerPath,
    [switch]$DryRun
)
# Clears the latched SpeedControl fault word so Mission_Start stops rejecting
# with MISSION_SPEED_FAULT. It writes exactly one RAM word (speed_control.fault),
# never touches flash, never resets the MCU. Motors stay stopped because
# speed_control.running stays 0.
$ErrorActionPreference = 'Stop'
$map = Get-Content -LiteralPath (Join-Path $ProjectRoot (Join-Path $BuildDir 'C10B_Test.map')) -Raw
$symbol = [regex]::Match($map, '(?m)^\s+0x([0-9a-fA-F]+)\s+speed_control\s*$')
if (-not $symbol.Success) { throw 'Map has no speed_control. Build the mission firmware first.' }
if (-not $ProgrammerPath) {
    $ProgrammerPath = Join-Path $env:USERPROFILE 'AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
}
if (-not (Test-Path -LiteralPath $ProgrammerPath)) { throw "Programmer missing: $ProgrammerPath" }

$base = [Convert]::ToUInt32($symbol.Groups[1].Value, 16)
$faultAddress = $base + 4
$faultNames = @('NONE','LEFT_NO_FEEDBACK','RIGHT_NO_FEEDBACK','REVERSED_FEEDBACK','SAMPLE_TIMEOUT','BAD_TARGET')

function Read-Words([uint32]$Address, [int]$Count) {
    $lines = & $ProgrammerPath -c port=SWD mode=HOTPLUG -r32 ('0x{0:X8}' -f $Address) ('0x{0:X}' -f ($Count * 4)) 2>&1
    if ($LASTEXITCODE -ne 0) { throw "ST-LINK read failed: $($lines -join ' ')" }
    $words = [Collections.Generic.List[uint32]]::new()
    foreach ($line in $lines) {
        $match = [regex]::Match([string]$line, '^\s*0x[0-9a-fA-F]+\s*:\s*((?:[0-9a-fA-F]{8}(?:\s+|$))+)')
        if ($match.Success) { foreach ($word in ($match.Groups[1].Value.Trim() -split '\s+')) { $words.Add([Convert]::ToUInt32($word, 16)) } }
    }
    if ($words.Count -lt $Count) { throw "Short read at 0x{0:X8}: $($words.Count)/$Count." -f $Address }
    return $words
}

$before = Read-Words $base 2
$running = $before[0]
$fault = $before[1]
if ($running -gt 1) { throw "speed_control.running=$running looks wrong; refusing to write (rebuild and retry)." }
if ($fault -gt 5) { throw "speed_control.fault=$fault is not a Speed_Fault value; refusing to write." }

$faultText = if ($fault -lt $faultNames.Count) { $faultNames[$fault] } else { "code $fault" }
"speed_control @ 0x{0:X8}: running={1} fault={2} ({3})" -f $base, $running, $fault, $faultText

if ($fault -eq 0) { 'Nothing to clear: fault is already NONE.'; return }

if ($DryRun) {
    "Would write 0x00000000 to 0x{0:X8} (speed_control.fault)." -f $faultAddress
    return
}

$write = & $ProgrammerPath -c port=SWD mode=HOTPLUG -w32 ('0x{0:X8}' -f $faultAddress) 0x00000000 2>&1
if ($LASTEXITCODE -ne 0) { throw "ST-LINK write failed: $($write -join ' ')" }
$after = Read-Words $base 2
"after write: running={0} fault={1}" -f $after[0], $after[1]
if ($after[1] -ne 0) { throw 'Fault word did not clear; check the map/firmware.' }
'Cleared. Press USER_KEY to start again; the mission will re-check all preconditions.'