$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$engine = Join-Path $root 'CS560Framework'
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$installation = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $installation) { throw 'Visual Studio C++ build tools are required.' }
$setup = Join-Path $installation 'VC\Auxiliary\Build\vcvars64.bat'
$outputDir = Join-Path ([IO.Path]::GetTempPath()) ('locomotion-test-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $outputDir | Out-Null
$batch = Join-Path $outputDir 'build.cmd'
@"
@echo off
call "$setup" >nul
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /I"$engine" /I"$engine\include" /I"$engine\include\glm" "$PSScriptRoot\test_locomotion.cpp" "$engine\Path.cpp" "$engine\LocomotionController.cpp" "$engine\Animator.cpp" "$engine\VQS.cpp" "$engine\Quaternion.cpp" /Fe:locomotion_test.exe
"@ | Set-Content -LiteralPath $batch
Push-Location $outputDir
try {
    & cmd /c $batch
    if ($LASTEXITCODE -ne 0) { throw 'Locomotion test compilation failed.' }
    & (Join-Path $outputDir 'locomotion_test.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Locomotion checks failed.' }
} finally { Pop-Location }
