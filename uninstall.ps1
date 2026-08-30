param([string]$SteamVrPath)

$ErrorActionPreference = "Stop"

if (Get-Process vrserver -ErrorAction SilentlyContinue) {
    throw "vrserver.exe is running - quit SteamVR before uninstalling."
}

if (-not $SteamVrPath) {
    $steam = (Get-ItemProperty "HKCU:\Software\Valve\Steam" -ErrorAction SilentlyContinue).SteamPath
    if (-not $steam) {
        $steam = (Get-ItemProperty "HKLM:\SOFTWARE\WOW6432Node\Valve\Steam" -ErrorAction SilentlyContinue).InstallPath
    }
    if (-not $steam) { throw "Steam install path not found in registry; pass -SteamVrPath." }
    $SteamVrPath = Join-Path ($steam -replace "/", "\") "steamapps\common\SteamVR"
}

$dest = Join-Path $SteamVrPath "drivers\01calcium"
if (Test-Path $dest) {
    Remove-Item -Recurse -Force $dest
    Write-Host "Removed $dest"
} else {
    Write-Host "Not installed: $dest"
}
