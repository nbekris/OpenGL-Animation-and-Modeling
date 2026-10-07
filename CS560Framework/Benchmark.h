#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

// Standard-library-only timing utility. No dependencies on rendering or GLM.
class Benchmark
{
public:
    struct Options
    {
        std::size_t iterations = 100000;
        std::size_t repetitions = 9;
        std::size_t warmupIterations = 1000;
    };

    struct Result
    {
        std::string name;
        std::size_t iterations;
        std::vector<double> nanosecondsPerCall;
        double medianNanoseconds;
        double minimumNanoseconds;

        void Print(std::ostream& out) const
        {
            out << name << ": median " << medianNanoseconds
                << " ns/call, minimum " << minimumNanoseconds
                << " ns/call (" << iterations << " iterations x "
                << nanosecondsPerCall.size() << " samples)\n";
        }
    };

    // operation(index) is timed, including any work in the supplied callable.
    // It must make its result observable; use Consume for a returned value.
    // Exceptions propagate. Input setup and printing belong outside Run.
    template<class Operation>
    static Result Run(const std::string& name, Operation&& operation,
                      const Options& options)
    {
        if (options.iterations == 0 || options.repetitions == 0)
        {
            throw std::invalid_argument("Benchmark iterations and repetitions must be positive.");
        }

        Result result{name, options.iterations, {}, 0.0, 0.0};
        result.nanosecondsPerCall.reserve(options.repetitions);
        for (std::size_t i = 0; i < options.warmupIterations; ++i)
        {
            operation(i);
        }

        for (std::size_t sample = 0; sample < options.repetitions; ++sample)
        {
            const auto start = std::chrono::steady_clock::now();
            for (std::size_t i = 0; i < options.iterations; ++i)
            {
                operation(i);
            }
            const auto end = std::chrono::steady_clock::now();
            const double ns = std::chrono::duration<double, std::nano>(end - start).count();
            result.nanosecondsPerCall.push_back(ns / options.iterations);
        }

        auto sorted = result.nanosecondsPerCall;
        std::sort(sorted.begin(), sorted.end());
        const auto middle = sorted.size() / 2;
        result.medianNanoseconds = sorted.size() % 2
            ? sorted[middle] : (sorted[middle - 1] + sorted[middle]) / 2.0;
        result.minimumNanoseconds = sorted.front();
        return result;
    }

    template<class Operation>
    static Result Run(const std::string& name, Operation&& operation)
    {
        return Run(name, operation, Options{});
    }

    // Portable observable reads prevent unused outputs from being discarded.
    // This reads every byte and adds overhead: include the same consumption in
    // every case. It is not a zero-cost compiler barrier or an isolated latency.
    template<class T>
    static void Consume(const T& value)
    {
        const volatile unsigned char* bytes =
            reinterpret_cast<const volatile unsigned char*>(&value);
        for (std::size_t i = 0; i < sizeof(T); ++i)
        {
            (void)bytes[i];
        }
    }
};
