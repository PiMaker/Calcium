$ErrorActionPreference = 'Stop'

# generate src/Icon.cs from icon.ico next to this script
$ico = Join-Path $PSScriptRoot 'icon.ico'
$out = Join-Path $PSScriptRoot 'src\frontend\Icon.cs'

$bytes = [IO.File]::ReadAllBytes($ico)
$lines = for ($i = 0; $i -lt $bytes.Length; $i += 16) {
    $end = [Math]::Min($i + 15, $bytes.Length - 1)
    '        ' + (($bytes[$i..$end] | ForEach-Object { '0x{0:X2}' -f $_ }) -join ', ') + ','
}
$code = "// Auto-generated from icon.ico by icon-gen.ps1; do not edit." +
    "`npublic static class Icon" +
    "`n{" +
    "`n    public static readonly byte[] Data = new byte[]" +
    "`n    {" +
    "`n" + ($lines -join "`n") +
    "`n    };" +
    "`n}" + "`n"

# skip write when unchanged so builds stay incremental
if ((Test-Path $out) -and ((Get-Content $out -Raw) -eq $code)) { exit 0 }
[IO.File]::WriteAllText($out, $code)
