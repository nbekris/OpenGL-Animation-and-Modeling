param([switch]$NoOpen, [double]$Seconds = 16)
$ErrorActionPreference = 'Stop'
if ($Seconds -le 0 -or [double]::IsNaN($Seconds) -or [double]::IsInfinity($Seconds)) {
    throw 'Seconds must be finite and positive.'
}
$root = Split-Path $PSScriptRoot -Parent
$engine = Join-Path $root 'CS560Framework'
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$installation = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $installation) { throw 'Visual Studio C++ build tools are required.' }
$setup = Join-Path $installation 'VC\Auxiliary\Build\vcvars64.bat'
$outputDir = Join-Path ([IO.Path]::GetTempPath()) ('path-debug-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $outputDir | Out-Null
$batch = Join-Path $outputDir 'build.cmd'
@"
@echo off
call "$setup" >nul
if errorlevel 1 exit /b 1
cl /nologo /EHsc /I"$engine" /I"$engine\include" "$PSScriptRoot\sample_path.cpp" "$engine\Path.cpp" /Fe:sample_path.exe
"@ | Set-Content -LiteralPath $batch
Push-Location $outputDir
try {
    & cmd /c $batch
    if ($LASTEXITCODE -ne 0) { throw 'C++ sampler compilation failed.' }
    $json = & (Join-Path $outputDir 'sample_path.exe')
    if ($LASTEXITCODE -ne 0) { throw 'C++ sampler failed.' }
    $data = $json | ConvertFrom-Json
    $data | Add-Member -NotePropertyName seconds -NotePropertyValue $Seconds
    $template = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'path_debug.html') -Raw
    $output = Join-Path $outputDir 'path_debug.html'
    $template.Replace('__PATH_DATA__', ($data | ConvertTo-Json -Depth 6 -Compress)) | Set-Content -LiteralPath $output -Encoding utf8
    Write-Output "Maximum control-point error: $($data.positionError)"
    Write-Output "Maximum estimated tangent mismatch: $($data.tangentError)"
    Write-Output "Maximum wrap error: $($data.wrapError)"
    Write-Output "Viewer: $output"
    if (-not $NoOpen) { Start-Process -FilePath $output -WindowStyle Hidden }
} finally { Pop-Location }
