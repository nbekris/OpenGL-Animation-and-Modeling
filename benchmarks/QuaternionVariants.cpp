#include "QuaternionVariants.h"

glm::mat4 QuaternionVariants::ToMatrixTranspose(const Quaternion& input)
{
	const Quaternion q = input.Normalize();
	const float s = q._s;
	const float x = q._x;
	const float y = q._y;
	const float z = q._z;

	// GLM's constructor takes columns. Build mathematical rows, then transpose.
	const glm::vec4 row0{
		1.0f - 2.0f * (y*y + z*z),
		2.0f * (x*y - s*z),
		2.0f * (x*z + s*y), 0.0f };
	const glm::vec4 row1{
		2.0f * (x*y + s*z),
		1.0f - 2.0f * (x*x + z*z),
		2.0f * (y*z - s*x), 0.0f };
	const glm::vec4 row2{
		2.0f * (x*z - s*y),
		2.0f * (y*z + s*x),
		1.0f - 2.0f * (x*x + y*y), 0.0f };

	return glm::transpose(glm::mat4(row0, row1, row2,
		glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)));
}

