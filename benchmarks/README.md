# Function benchmarks

`CS560Framework/Benchmark.h` is a header-only utility with no GLM or application
dependencies. Supply a callable taking an iteration index, then print or inspect
the returned result. Options control iterations, repetitions, and warmup calls.
Samples are retained in execution order; median and minimum are in ns/call.

Open `CS560Framework.sln`, select **Release / x64**, and set **Benchmark** as
the startup project. Build and run without the debugger (Ctrl+F5). This console
project compiles only the quaternion implementation and benchmark sources; it
does not launch or link the renderer. Production Quaternion exposes ToMatrix;
experimental alternatives live in QuaternionVariants.h/.cpp in this directory.

From an x64 Visual Studio Developer Command Prompt, in the repository root:

```bat
msbuild benchmarks\Benchmark.vcxproj /p:Configuration=Release /p:Platform=x64
benchmarks\build\x64\Release\ToMatrixBenchmark.exe
```

For another function:

```cpp
Benchmark::Options options;
options.iterations = 1000000;
auto result = Benchmark::Run("MyFunction", [&](std::size_t i) {
    auto output = MyFunction(inputs[i % inputs.size()]);
    Benchmark::Consume(output);
}, options);
result.Print(std::cout);
```

For void functions, make their side effects observable instead. Prepare inputs
outside timing, avoid printing or allocating inside the callable unless those
costs are intended, and use varied inputs. Warmup also invokes your function,
so account for mutable state. Reset state outside Run or use separate fixtures.

Use optimized builds with matching compiler/SIMD settings. Timings include loop,
input selection, and Consume overhead; Consume reads every output byte. Compare
equivalent functions with identical harnesses and output types. For tiny functions,
that overhead can dominate; treat small differences cautiously and inspect assembly
before claiming a winner. Repeat comparisons in reversed order to check drift.

The example verifies that ToMatrix (component assignments) and QuaternionVariants::ToMatrixTranspose
(row construction followed by transpose) agree before timing both, including
normalization. It repeats them in reversed order to help detect timing drift.
The transpose function uses mathematical rows so both return the same rotation.

Both implementations compile in separate translation units without link-time
optimization, so this comparison includes function calls. Align compiler, SIMD,
floating-point and link-time optimization settings with the application when
measuring its behavior. Add future benchmark cases here, and keep experimental
implementations outside production headers. The generic Benchmark.h remains
independent of the cases and has no rendering or GLM dependencies.

BenchmarkSmokeTest.cpp is a separate test entry point, excluded from the benchmark
executable. To run it from a Developer Command Prompt:

```bat
cl /nologo /O2 /EHsc /std:c++14 /I CS560Framework benchmarks\BenchmarkSmokeTest.cpp /Fobenchmarks\build\ /Febenchmarks\build\BenchmarkSmokeTest.exe
benchmarks\build\BenchmarkSmokeTest.exe
```
