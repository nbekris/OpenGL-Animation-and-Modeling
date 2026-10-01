#include "Benchmark.h"
#include "QuaternionVariants.h"
#include <cmath>
#include <iostream>
#include <vector>

int main()
{
    // Varied inputs are created before timing. No graphics context is needed.
    std::vector<Quaternion> inputs;
    inputs.reserve(4096);
    for (int i = 0; i < 4096; ++i)
    {
        const float t = static_cast<float>(i) / 4096.0f;
        inputs.emplace_back(1.0f + t, t - 0.5f, 0.25f + t, 0.75f - t);
    }

    // Verify equivalence outside the timed region, including known rotations.
    auto check = [](const Quaternion& q)
    {
        const auto direct = q.ToMatrix();
        const auto transpose = QuaternionVariants::ToMatrixTranspose(q);
        for (int col = 0; col < 4; ++col)
            for (int row = 0; row < 4; ++row)
                if (!std::isfinite(direct[col][row]) ||
                    !std::isfinite(transpose[col][row]) ||
                    std::abs(direct[col][row] - transpose[col][row]) > 1e-6f)
                    throw std::runtime_error("Matrix implementations disagree.");
    };
    check(Quaternion(1.0f, 0.0f, 0.0f, 0.0f));
    check(Quaternion(1.0f, 0.0f, 0.0f, 1.0f));
    check(Quaternion(0.0f, 1.0f, 0.0f, 0.0f));
    for (const auto& input : inputs)
        check(input);

    const auto direct = [&](std::size_t i)
    {
        const auto matrix = inputs[i % inputs.size()].ToMatrix();
        Benchmark::Consume(matrix);
    };
    const auto transpose = [&](std::size_t i)
    {
        const auto matrix = QuaternionVariants::ToMatrixTranspose(inputs[i % inputs.size()]);
        Benchmark::Consume(matrix);
    };
    Benchmark::Run("ToMatrix (components)", direct).Print(std::cout);
    Benchmark::Run("ToMatrixTranspose", transpose).Print(std::cout);
    std::cout << "Reversed order:\n";
    Benchmark::Run("ToMatrixTranspose", transpose).Print(std::cout);
    Benchmark::Run("ToMatrix (components)", direct).Print(std::cout);
}
