#include "Benchmark.h"
#include <stdexcept>

int main()
{
    Benchmark::Options options;
    options.iterations = 7;
    options.repetitions = 4;
    options.warmupIterations = 3;
    std::size_t calls = 0;
    auto result = Benchmark::Run("count", [&](std::size_t) { ++calls; }, options);
    if (calls != 31 || result.nanosecondsPerCall.size() != 4)
        return 1;
    auto sorted = result.nanosecondsPerCall;
    std::sort(sorted.begin(), sorted.end());
    if (result.minimumNanoseconds != sorted.front() ||
        result.medianNanoseconds != (sorted[1] + sorted[2]) / 2.0)
        return 2;
    options.iterations = 0;
    try { Benchmark::Run("invalid", [](std::size_t) {}, options); return 3; }
    catch (const std::invalid_argument&) {}
    options.iterations = 1;
    options.repetitions = 0;
    try { Benchmark::Run("invalid", [](std::size_t) {}, options); return 4; }
    catch (const std::invalid_argument&) {}
    try
    {
        Benchmark::Run("throws", [](std::size_t) { throw std::runtime_error("test"); });
        return 5;
    }
    catch (const std::runtime_error&) {}
    return 0;
}
