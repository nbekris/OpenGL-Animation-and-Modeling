#include "Path.h"
#include <iomanip>
#include <iostream>

int main()
{
    Path path;
    const int count = static_cast<int>(path.points.size());
    auto vector = [](const glm::vec3& p) {
        std::cout << '[' << p.x << ',' << p.y << ',' << p.z << ']';
    };
    std::cout << std::setprecision(9) << "{\"points\":[";
    for (int i = 0; i < count; ++i) {
        if (i) std::cout << ',';
        vector(path.points[i]);
    }
    std::cout << "],\"samples\":[";
    for (int i = 0; i <= count * 200; ++i) {
        if (i) std::cout << ',';
        vector(path.Position(i / 200.0));
    }
    float positionError = 0, tangentError = 0, wrapError = 0;
    const double h = 0.001;
    for (int i = 0; i < count; ++i) {
        const auto p = path.Position(i);
        positionError = glm::max(positionError, glm::length(p - path.points[i]));
        const auto left = (p - path.Position(i - h)) / float(h);
        const auto right = (path.Position(i + h) - p) / float(h);
        tangentError = glm::max(tangentError, glm::length(left - right));
    }
    for (double s : {-0.25, 0.0, 0.4, 7.9})
        wrapError = glm::max(wrapError,
            glm::length(path.Position(s) - path.Position(s + count)));
    std::cout << "],\"positionError\":" << positionError
              << ",\"tangentError\":" << tangentError
              << ",\"wrapError\":" << wrapError << '}';
}
