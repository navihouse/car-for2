param(
    [Parameter(Mandatory=$true)][string]$RouteTest,
    [Parameter(Mandatory=$true)][string]$OutputDir
)
$ErrorActionPreference='Stop'
$taskReader=Join-Path (Split-Path -Parent $PSScriptRoot) 'scripts\read_mission.ps1'
$taskDump=Join-Path $OutputDir 'mission-v3-route-dump.txt'
& $RouteTest $taskDump
if ($LASTEXITCODE -ne 0) { throw 'Actual route test failed before exporting its diagnostics.' }
$taskRecord=& $taskReader -DumpFile $taskDump -AsJson | ConvertFrom-Json
if ($taskRecord.Version -ne 3 -or $taskRecord.State -ne 'DONE' -or $taskRecord.Phase -ne 'COMPLETE' -or
    $taskRecord.Motion_mode -ne 'BRAKE' -or $taskRecord.Arrivals -ne 16 -or $taskRecord.Last_point -ne 'A' -or
    $taskRecord.Lap -ne 3 -or $taskRecord.Event_count -ne 16 -or
    $taskRecord.Distance_mm -lt $taskRecord.Point_min_mm -or
    $taskRecord.Distance_mm -gt $taskRecord.Point_max_mm) { throw 'C diagnostic ABI and PowerShell decoder disagree.' }

# Exercise signed millidegrees using the real C-layout dump, keeping its other
# fields and sequence intact. No hardware reader is used by this test.
$taskLines=Get-Content -LiteralPath $taskDump
$taskSignedHex='{0:X8}' -f [BitConverter]::ToUInt32([BitConverter]::GetBytes([int32]-38660),0)
$taskLines[7]=$taskLines[7] -replace ': [0-9A-F]{8} [0-9A-F]{8} ',(": $taskSignedHex 000003E8 ")
$taskSigned=Join-Path $OutputDir 'mission-v3-signed-dump.txt'
Set-Content -LiteralPath $taskSigned -Value $taskLines -Encoding ascii
$taskRecord=& $taskReader -DumpFile $taskSigned -AsJson | ConvertFrom-Json
if ([math]::Abs($taskRecord.Target_yaw_deg+38.66) -gt 0.000001 -or $taskRecord.Heading_error_deg -ne 1) {
    throw 'Signed millidegree decoding failed.'
}
$taskLines[-1]=$taskLines[-1] -replace '[0-9A-F]{8}$','00000001'
$taskTorn=Join-Path $OutputDir 'mission-v3-torn-dump.txt'
Set-Content -LiteralPath $taskTorn -Value $taskLines -Encoding ascii
$taskRejected=$false
try { & $taskReader -DumpFile $taskTorn -AsJson | Out-Null }
catch {
    if ($_.Exception.Message -like '*changed during read*') { $taskRejected=$true }
    else { throw }
}
if (-not $taskRejected) { throw 'Decoder accepted a torn diagnostic snapshot.' }
Write-Output 'Mission V3: actual C ABI over four laps, signed units and torn-read rejection passed.'
