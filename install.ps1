param([string]$SteamVrPath)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$driverName = "01calcium"

if (Get-Process vrserver -ErrorAction SilentlyContinue) {
    throw "vrserver.exe is running - quit SteamVR before installing."
}

# driver: native AOT single dll
dotnet publish "$root\calcium-driver\calcium-driver.csproj" -c Release -r win-x64
if ($LASTEXITCODE -ne 0) { throw "dotnet publish (driver) failed with exit code $LASTEXITCODE" }
$driverPublish = Join-Path $root "calcium-driver\bin\Release\net10.0\win-x64\publish"
if (-not (Test-Path (Join-Path $driverPublish "driver_$driverName.dll"))) {
    throw "Driver publish output not found: $driverPublish"
}

# register in SteamVR
if (-not $SteamVrPath) {
    $steam = (Get-ItemProperty "HKCU:\Software\Valve\Steam" -ErrorAction SilentlyContinue).SteamPath
    if (-not $steam) {
        $steam = (Get-ItemProperty "HKLM:\SOFTWARE\WOW6432Node\Valve\Steam" -ErrorAction SilentlyContinue).InstallPath
    }
    if (-not $steam) { throw "Steam install path not found in registry; pass -SteamVrPath." }
    $SteamVrPath = Join-Path ($steam -replace "/", "\") "steamapps\common\SteamVR"
}

if (-not (Test-Path (Join-Path $SteamVrPath "bin\win64\vrserver.exe"))) {
    throw "Not a SteamVR folder (missing bin\win64\vrserver.exe): $SteamVrPath"
}

$dest = Join-Path $SteamVrPath "drivers\$driverName"
New-Item -ItemType Directory -Force -Path (Join-Path $dest "bin\win64") | Out-Null
Copy-Item (Join-Path $driverPublish "*.dll") (Join-Path $dest "bin\win64\") -Force
Copy-Item (Join-Path $root "driver.vrdrivermanifest") $dest -Force
New-Item -Force -ItemType Directory -Path (Join-Path $dest "settings")
Copy-Item (Join-Path $root "default.vrsettings") (Join-Path $dest "settings") -Recurse -Force
Write-Host "Installed driver to $dest"
