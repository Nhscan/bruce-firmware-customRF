# Host-side codec regression test for the RF Tools automotive module.
#
# Builds rft_decode.cpp / rft_encode.cpp / rft_keyfile.cpp with a tiny
# Arduino-String shim and checks encode -> decode round trips, a synthetic
# Chrysler packet, synthetic Fiat frames (separate and merged pulses), timing
# jitter, and .key parse/serialize.
#
# Usage (from the firmware root):
#   powershell -File tests/rft_codec/run.ps1

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$here = $PSScriptRoot
$rft = Join-Path $root "src\modules\rf\rft"

$sources = @(
    (Join-Path $here "test.cpp"),
    (Join-Path $rft "rft_decode.cpp"),
    (Join-Path $rft "rft_encode.cpp"),
    (Join-Path $rft "rft_keyfile.cpp")
)

$exe = Join-Path $here "rft_codec_test.exe"
& g++ -std=c++17 -O1 -I $here -I $rft $sources -o $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe
exit $LASTEXITCODE
