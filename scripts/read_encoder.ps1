param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$BuildDir = 'build\Mission-Debug',
    [string]$ProgrammerPath,
    [int]$IntervalMs = 300,
    [int]$Count = 1
)
$ErrorActionPreference = 'Stop'
$map = Get-Content -LiteralPath (Join-Path $ProjectRoot (Join-Path $BuildDir 'C10B_Test.map')) -Raw
$symbol = [regex]::Match($map, '(?m)^\s+0x([0-9a-fA-F]+)\s+encoder_data\s*$')
if (-not $symbol.Success) { throw 'Map has no encoder_data. Build the mission firmware first.' }
if (-not $ProgrammerPath) {
    $ProgrammerPath = Join-Path $env:USERPROFILE 'AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
}
if (-not (Test-Path -LiteralPath $ProgrammerPath)) { throw "Programmer missing: $ProgrammerPath" }

function Signed-Of([uint32]$Word) { [BitConverter]::ToInt32([BitConverter]::GetBytes($Word),0) }
function Float-Of([uint32]$Word) { [BitConverter]::ToSingle([BitConverter]::GetBytes($Word),0) }
function Long-Of([uint32]$Low, [uint32]$High) {
    [BitConverter]::ToInt64([BitConverter]::GetBytes(([uint64]$High -shl 32) -bor [uint64]$Low),0)
}

for ($pass = 1; $pass -le $Count; $pass++) {
    # One consistent snapshot: Encoder_Data is the only source, read is not atomic,
    # so sample_count is re-read afterwards and the record is rejected on change.
    $lineWords = & $ProgrammerPath -c port=SWD mode=HOTPLUG -r32 ('0x' + $symbol.Groups[1].Value) 0x30 2>&1
    if ($LASTEXITCODE -ne 0) { throw "ST-LINK read failed: $($lineWords -join ' ')" }
    $words = [Collections.Generic.List[uint32]]::new()
    foreach ($line in $lineWords) {
        $match = [regex]::Match([string]$line, '^\s*0x[0-9a-fA-F]+\s*:\s*((?:[0-9a-fA-F]{8}(?:\s+|$))+)')
        if ($match.Success) { foreach ($word in ($match.Groups[1].Value.Trim() -split '\s+')) { $words.Add([Convert]::ToUInt32($word, 16)) } }
    }
    if ($words.Count -ne 12) { throw "Incomplete Encoder_Data: $($words.Count)/12 words." }
    # Layout has 4 bytes of padding before the 8-byte aligned int64 totals.
    [pscustomobject][ordered]@{
        Pass = $pass
        Left_raw = $words[0] -band 0xFFFF
        Right_raw = $words[0] -shr 16
        Left_delta = Signed-Of $words[1]
        Right_delta = Signed-Of $words[2]
        Left_total = Long-Of $words[4] $words[5]
        Right_total = Long-Of $words[6] $words[7]
        Left_cps = [math]::Round((Float-Of $words[8]), 1)
        Right_cps = [math]::Round((Float-Of $words[9]), 1)
        Sample_period_ms = $words[10]
        Sample_count = $words[11]
    } | Format-List
    if ($pass -lt $Count) { Start-Sleep -Milliseconds $IntervalMs }
}
