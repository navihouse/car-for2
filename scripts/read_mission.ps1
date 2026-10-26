param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$BuildDir = 'build\Mission-Debug',
    [string]$DumpFile,
    [string]$ProgrammerPath,
    [switch]$AsJson
)
$ErrorActionPreference = 'Stop'
if ($DumpFile) {
    $lines = Get-Content -LiteralPath $DumpFile
} else {
    $map = Get-Content -LiteralPath (Join-Path $ProjectRoot (Join-Path $BuildDir 'C10B_Test.map')) -Raw
    $symbol = [regex]::Match($map, '(?m)^\s+0x([0-9a-fA-F]+)\s+mission_diag\s*$')
    if (-not $symbol.Success) { throw 'Map has no mission_diag. Build the mission firmware first.' }
    if (-not $ProgrammerPath) {
        $ProgrammerPath = Join-Path $env:USERPROFILE 'AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
    }
    if (-not (Test-Path -LiteralPath $ProgrammerPath)) { throw "Programmer missing: $ProgrammerPath" }
    # Read only. No flash, reset, halt, run or motor commands.
    $lines = & $ProgrammerPath -c port=SWD mode=HOTPLUG -r32 ('0x' + $symbol.Groups[1].Value) 0x60 2>&1
    if ($LASTEXITCODE -ne 0) { throw "ST-LINK read failed: $($lines -join ' ')" }
}
$words = [Collections.Generic.List[uint32]]::new()
foreach ($line in $lines) {
    $match = [regex]::Match([string]$line, '^\s*0x[0-9a-fA-F]+\s*:\s*((?:[0-9a-fA-F]{8}(?:\s+|$))+)')
    if ($match.Success) {
        foreach ($word in ($match.Groups[1].Value.Trim() -split '\s+')) {
            $words.Add([Convert]::ToUInt32($word, 16))
        }
    }
}
if ($words.Count -ne 24) { throw "Incomplete mission record: $($words.Count)/24 words." }
if ($words[0] -ne 0x4D534E31 -or $words[1] -ne 1) { throw 'Wrong mission magic/version.' }
if (($words[2] -band 1) -or $words[2] -ne $words[23]) { throw 'Mission snapshot changed during read; retry.' }
$modes = @('FULL','AC','CB','BD','DA')
$states = @('IDLE','RUNNING','DONE','STOPPED','REJECTED','FAULT')
$segments = @('AC','CB','BD','DA')
$reasons = @('READY','NOT_IMPLEMENTED','USER_STOP','BAD_CONFIG','BAD_TARGET','SEGMENT_FAULT',
    'GYRO_UNAVAILABLE','LINE_UNAVAILABLE','ENCODER_STALE','SPEED_FAULT','CALIBRATING')
$points = @('NONE','A','B','C','D')
if ($words[3] -ge $modes.Count -or $words[4] -ge $states.Count -or $words[5] -ge $reasons.Count -or
    $words[6] -ge $segments.Count -or $words[11] -ge $points.Count) { throw 'Invalid mission enumeration.' }
$record = [pscustomobject][ordered]@{
    Version = $words[1]; Sequence = $words[2]; Mode = $modes[$words[3]]
    State = $states[$words[4]]; Reason = $reasons[$words[5]]; Segment = $segments[$words[6]]
    Segment_fault = $words[7]; Uptime_ms = $words[8]; Elapsed_ms = $words[9]
    Arrivals = $words[10]; Last_point = $points[$words[11]]
    Last_event_ms = $words[12]; Event_count = $words[13]
    Left_target_cps = [BitConverter]::ToInt32([BitConverter]::GetBytes($words[14]),0)
    Right_target_cps = [BitConverter]::ToInt32([BitConverter]::GetBytes($words[15]),0)
    Speed_fault = $words[16]; Gyro_ready = [bool]$words[17]; Gyro_age_ms = $words[18]
    Line_calibrated = [bool]$words[19]; Line_age_ms = $words[20]
    Line_mask = ('0x{0:X2}' -f $words[21]); Calibrating = [bool]$words[22]
}
if ($AsJson) { $record | ConvertTo-Json -Compress } else { $record | Format-List }
