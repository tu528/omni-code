$ErrorActionPreference = 'Stop'
$compiler = 'C:\msys64\ucrt64\bin\g++.exe'
$env:PATH = (Split-Path -Parent $compiler) + ';' + $env:PATH
$testOutput = Join-Path $env:TEMP 'omni-feeder-test.exe'
& $compiler -std=c++11 -Wall -Wextra -Werror (Join-Path $PSScriptRoot 'feeder_slots_test.cpp') -o $testOutput
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $testOutput
exit $LASTEXITCODE
