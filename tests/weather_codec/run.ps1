# Host-side regression test for the weather decoder module.
#
# Builds the real rf_weather_decoders.cpp with a tiny Arduino-String shim and
# runs it against the sample .sub captures listed in WEATHER_STATION.md.
#
# Usage (from the firmware root):
#   powershell -File tests/weather_codec/run.ps1
# Optional sample root:
#   powershell -File tests/weather_codec/run.ps1 -Samples "D:\Weather_stations"

param(
    [string]$Samples = "C:\Users\admin\Desktop\Weather_stations"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$here = $PSScriptRoot
$rft = Join-Path $root "src\modules\rf"

$exe = Join-Path $here "weather_codec_test.exe"
& g++ -std=c++17 -O1 -I $here -I $rft `
    (Join-Path $here "test.cpp") `
    (Join-Path $rft "rf_weather_decoders.cpp") `
    -o $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe $Samples
exit $LASTEXITCODE
