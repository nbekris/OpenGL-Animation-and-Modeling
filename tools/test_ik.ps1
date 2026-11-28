$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$engine = Join-Path $root 'CS560Framework'
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$installation = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $installation) { throw 'Visual Studio C++ build tools are required.' }
$setup = Join-Path $installation 'VC\Auxiliary\Build\vcvars64.bat'
$outputDir = Join-Path ([IO.Path]::GetTempPath()) ('ik-test-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $outputDir | Out-Null
Copy-Item -LiteralPath (Join-Path $engine 'assimp-vc143-mt.dll') -Destination $outputDir
$batch = Join-Path $outputDir 'build.cmd'
@"
@echo off
call "$setup" >nul
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHsc /I"$engine" /I"$engine\include" /I"$engine\include\glm" "$PSScriptRoot\test_ik.cpp" "$engine\Animator.cpp" "$engine\InverseKinematics.cpp" "$engine\AnimationData.cpp" "$engine\VQS.cpp" "$engine\Quaternion.cpp" "$engine\libs\assimp\assimp-vc143-mt.lib" /Fe:ik_test.exe
"@ | Set-Content -LiteralPath $batch
Push-Location $outputDir
try {
    & cmd /c $batch
    if ($LASTEXITCODE -ne 0) { throw 'IK test compilation failed.' }
    & (Join-Path $outputDir 'ik_test.exe') (Join-Path $engine 'fbx\fbx\roman_D.fbx') (Join-Path $engine 'fbx\fbx\viking_C.fbx')
    if ($LASTEXITCODE -ne 0) { throw 'IK checks failed.' }
} finally { Pop-Location }
