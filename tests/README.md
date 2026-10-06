# Animator checks

From a Visual Studio developer terminal in the repository root:

```bat
msbuild tests\AnimatorTests.vcxproj /p:Configuration=Debug /p:Platform=x64
tests\build\AnimatorTests.exe
```

These tests cover playback conversion, pause/resume, looping, clip changes,
invalid input, unspecified tick rates, zero-duration clips, scene reset, and
pose sampling with hierarchy accumulation and inverse-root conversion.
The test executable uses the release C++ runtime to match the bundled Assimp
DLL, whose scene destructor frees the synthetic scene's allocations.

The application currently advances the playback clock and displays it in the
menu. Pose sampling lives in `Animator`; `Mesh` renders skeleton lines from
that pose. This step preserves the first-frame preview at load and clip
selection. Sampling and refreshing those lines every frame is the next step.
