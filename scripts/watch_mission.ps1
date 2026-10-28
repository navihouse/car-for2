<#
.SYNOPSIS
    Follow the mission live over SWD and print every state/segment/phase change.

.DESCRIPTION
    read_mission.ps1 shows one snapshot, which is enough after a stop but blind
    during a run: the interesting failures are transitions (which turn was
    commanded, which heading the car actually had, how far it travelled before
    an edge was confirmed). This polls the same snapshot and prints only the
    rows that change, so a whole four-lap run fits in a terminal buffer.

    Stop with Ctrl+C. Nothing is written to the board.

.PARAMETER IntervalMs
    Poll period. 100 ms is plenty: the mission state machine ticks at 10 ms but
    the phases you care about last hundreds of milliseconds.

.PARAMETER DurationSec
    Stop after this many seconds (0 = run until Ctrl+C).

.EXAMPLE
    .\scripts\watch_mission.ps1
    .\scripts\watch_mission.ps1 -IntervalMs 200 -DurationSec 180
#>
param(
    [int]$IntervalMs = 100,
    [int]$DurationSec = 0
)
$ErrorActionPreference = 'Stop'
$reader = Join-Path $PSScriptRoot 'read_mission.ps1'
if (-not (Test-Path $reader)) { throw "read_mission.ps1 not found next to this script." }

$started = Get-Date
$last = $null
$header = '{0,7} {1,-6} {2,-8} {3,-9} {4,-6} {5,7} {6,8} {7,8} {8,6} {9,6}' -f `
    't(s)', 'lap', 'state', 'segment', 'phase', 'dist', 'target', 'hdg_err', 'arr', 'cap'
Write-Host $header
Write-Host ('-' * $header.Length)

while ($true) {
    if ($DurationSec -gt 0 -and ((Get-Date) - $started).TotalSeconds -ge $DurationSec) { break }
    $record = $null
    for ($attempt = 0; $attempt -lt 4 -and $null -eq $record; $attempt++) {
        try {
            $json = & $reader -AsJson 2>$null
            if ($LASTEXITCODE -eq 0 -and $json) { $record = ($json | ConvertFrom-Json) }
        } catch { }
        if ($null -eq $record) { Start-Sleep -Milliseconds 30 }
    }
    if ($null -eq $record) { Start-Sleep -Milliseconds $IntervalMs; continue }

    $key = '{0}/{1}/{2}/{3}/{4}/{5}/{6}' -f $record.State, $record.Segment, $record.Phase, $record.Lap,
        $record.Speed_fault, [math]::Floor($record.Distance_mm / 200.0), [math]::Round($record.Heading_error_deg / 10.0)
    if ($key -ne $last) {
        $t = ((Get-Date) - $started).TotalSeconds
        $line = '{0,7:F1} {1,-6} {2,-8} {3,-9} {4,-6} {5,7} {6,8:F1} {7,8:F1} {8,6} {9,6}' -f `
            $t, $record.Lap, $record.State, $record.Segment, $record.Phase, `
            $record.Distance_mm, $record.Target_yaw_deg, $record.Heading_error_deg, `
            $record.Arrivals, $record.Capture_frames
        Write-Host $line
        $last = $key
    }
    Start-Sleep -Milliseconds $IntervalMs
}
Write-Host 'stopped.'
