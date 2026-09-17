# Run the physics backend inside the default WSL distribution.
$ErrorActionPreference = 'Stop'
$physicsWsl = Join-Path $env:WINDIR 'System32\wsl.exe'
$physicsRepo = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$physicsLinuxRepo = (& $physicsWsl --exec wslpath -a $physicsRepo).Trim()
if ($LASTEXITCODE -ne 0) { throw 'WSL could not resolve the repository path.' }
& $physicsWsl --cd $physicsLinuxRepo --exec "$physicsLinuxRepo/.local/physics-venv/bin/python" -m research_sdk.headless --backend grsim @args
exit $LASTEXITCODE
