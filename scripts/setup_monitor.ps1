param(
    [string]$PythonExecutable = "python",
    [string]$VenvDirectory = ".venv",
    [switch]$Development
)

$ErrorActionPreference = "Stop"
$env:PYTHONUTF8 = "1"
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$venvPath = [IO.Path]::GetFullPath((Join-Path $projectRoot $VenvDirectory))
if (-not $venvPath.StartsWith($projectRoot + [IO.Path]::DirectorySeparatorChar,
        [StringComparison]::OrdinalIgnoreCase)) {
    throw "The virtual environment must be inside this project."
}

Push-Location -LiteralPath $projectRoot
try {
    & $PythonExecutable -c "import sys,struct; assert (3,10) <= sys.version_info[:2] <= (3,12), 'Pinned dependencies require Python 3.10-3.12'; assert struct.calcsize('P') == 8, '64-bit Python required'; print(sys.executable, sys.version.split()[0])"
    if ($LASTEXITCODE -ne 0) { throw "Select a working 64-bit Python 3.10-3.12 with -PythonExecutable." }
    $venvPython = Join-Path $venvPath "Scripts\python.exe"
    if (-not (Test-Path -LiteralPath $venvPython)) {
        & $PythonExecutable -m venv $venvPath
        if ($LASTEXITCODE -ne 0) { throw "Creating the project virtual environment failed." }
    }
    $requirements = if ($Development) { "tools/monitor/requirements-dev.txt" } else { "tools/monitor/requirements.txt" }
    & $venvPython -m pip install --cache-dir (Join-Path $projectRoot ".local/pip-cache") -r $requirements
    if ($LASTEXITCODE -ne 0) { throw "Dependency installation failed; inspect the pip output and network settings." }
    & $venvPython "scripts/check_environment.py" --gui-smoke-test
    if ($LASTEXITCODE -ne 0) { throw "Environment validation failed; inspect the report above." }
    Write-Host "Environment ready: $venvPython"
    Write-Host "Start simulation: `"$venvPython`" tools/monitor/main.py --raw-simulate"
    Write-Host "Firmware compilation additionally requires ARM GCC; flashing requires OpenOCD and ST-Link drivers."
} finally {
    Pop-Location
}
