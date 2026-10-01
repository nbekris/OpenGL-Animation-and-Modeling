#pragma once

#include "Quaternion.h"

// Experimental implementations belong to the benchmark, not the production API.
namespace QuaternionVariants
{
    glm::mat4 ToMatrixTranspose(const Quaternion& input);
}
