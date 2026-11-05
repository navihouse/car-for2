param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$BuildDir = 'build\Mission-Debug',
    [string]$DumpFile,
    [string]$ProgrammerPath = (Join-Path $env:USERPROFILE 'AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'),
    [switch]$AsJson
)
$ErrorActionPreference='Stop'
if ($DumpFile) { $taskLines=Get-Content -LiteralPath $DumpFile }
else {
    $taskMap=Get-Content -LiteralPath (Join-Path $ProjectRoot (Join-Path $BuildDir 'C10B_Test.map')) -Raw
    $taskSymbol=[regex]::Match($taskMap,'(?m)^\s+0x([0-9a-fA-F]+)\s+mission_a_heading\s*$')
    if (-not $taskSymbol.Success) { throw 'Map has no mission_a_heading. Build the current firmware first.' }
    # Read only: no reset, halt, flash, run, or motor commands.
    $taskLines=& $ProgrammerPath -c port=SWD mode=HOTPLUG -r32 ('0x'+$taskSymbol.Groups[1].Value) 0x30 2>&1
    if ($LASTEXITCODE -ne 0) { throw "ST-LINK read failed: $($taskLines -join ' ')" }
}
$taskWords=[Collections.Generic.List[uint32]]::new()
foreach ($taskLine in $taskLines) {
    $taskMatch=[regex]::Match([string]$taskLine,'^\s*0x[0-9a-fA-F]+\s*:\s*((?:[0-9a-fA-F]{8}(?:\s+|$))+)')
    if ($taskMatch.Success) {
        foreach ($taskWord in ($taskMatch.Groups[1].Value.Trim() -split '\s+')) {
            $taskWords.Add([Convert]::ToUInt32($taskWord,16))
        }
    }
}
if ($taskWords.Count -ne 12) { throw "Incomplete A-heading record: $($taskWords.Count)/12 words." }
if ($taskWords[0] -ne 0x41484431 -or $taskWords[1] -notin @(1,2,3)) { throw 'Wrong A-heading magic/version.' }
if (($taskWords[2] -band 1) -or $taskWords[2] -ne $taskWords[11]) { throw 'A-heading snapshot changed during read; retry.' }
if ($taskWords[3] -gt 1 -or ([bool]$taskWords[3] -ne ($taskWords[4] -gt 0))) { throw 'Invalid A-heading reference state.' }
function Signed-Deg([uint32]$Word) { [BitConverter]::ToInt32([BitConverter]::GetBytes($Word),0)/1000.0 }
$taskRecord=[pscustomobject][ordered]@{
    Version=$taskWords[1]; Sequence=$taskWords[2]; Valid=[bool]$taskWords[3]; Departures=$taskWords[4]
    A0_deg=(Signed-Deg $taskWords[5]); A1_deg=(Signed-Deg $taskWords[6]); Total_deg=(Signed-Deg $taskWords[7])
    A2_deg=(Signed-Deg $taskWords[8]); Next_target_deg=(Signed-Deg $taskWords[9]); Max_correction_deg=(Signed-Deg $taskWords[10])
}
if ($AsJson) { $taskRecord | ConvertTo-Json -Compress } else { $taskRecord | Format-List }
