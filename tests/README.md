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

The CPU tests also cover LBS palette construction, mesh-node references and
instancing, submesh-specific offsets, bind-pose equivalence, normalized weights,
invalid bindings, and Assimp's actual four-influence postprocessor using a
synthetic five-influence scene exported/imported as Assimp binary.

To include supplied asset pose checks, run from the repository root:

```bat
tests\build\AnimatorTests.exe tests\build\skinning-assets.txt
```

The application also provides a hidden OpenGL smoke test. Build the Debug x64
application, then run from `CS560Framework`:

```bat
x64\Debug\CS560Framework.exe --skinning-smoke
```

It checks the actual geometry vertex shader using transform feedback for
single-bone deformation, weighted blending, rotated normals, unweighted vertices,
and singular blends. It reloads the Roman and Egyptian assets, checks changing
and paused G-buffer geometry, checks finite normals, and writes 750x750 RGB
readbacks (bottom row first) to `tests/build/*-skinned.rgb` for visual QA.
The Viking asset is also imported and pose checked. Static nodes such as its
shield preserve nonuniform bind scale; animated scale must remain uniform.
No DQS or skinning method selector is implemented.
