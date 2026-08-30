param()

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot

# build driver
dotnet publish "$root\calcium-driver\calcium-driver.csproj" -c Release -r win-x64
if ($LASTEXITCODE -ne 0) { throw "dotnet publish (driver) failed with exit code $LASTEXITCODE" }
$driverPublish = Join-Path $root "calcium-driver\bin\Release\net10.0\win-x64\publish"
if (-not (Test-Path (Join-Path $driverPublish "driver_01calcium.dll"))) {
    throw "Driver publish output not found: $driverPublish"
}

# gather driver dlls + icon (both gitignored)
$driverDir = Join-Path $root "calcium-installer\driver"
New-Item -ItemType Directory -Force -Path $driverDir | Out-Null
Get-ChildItem -Path $driverDir -Filter "*.dll" | Remove-Item -Force
Copy-Item (Join-Path $driverPublish "*.dll") $driverDir -Force

# build installer (single-file self-contained exe)
dotnet publish "$root\calcium-installer\calcium-installer.csproj" -c Release -r win-x64
if ($LASTEXITCODE -ne 0) { throw "dotnet publish (installer) failed with exit code $LASTEXITCODE" }
Write-Host "Installer: $(Join-Path $root 'calcium-installer\bin\Release\net10.0-windows\win-x64\publish\calcium-installer.exe')"
