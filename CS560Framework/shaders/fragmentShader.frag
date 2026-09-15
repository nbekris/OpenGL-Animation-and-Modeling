#version 330 core
out vec4 FragColor;

in vec2 texCoord;
in float floorX;
uniform float floorMinX;
uniform float floorMaxX;
uniform int useFloorGradient;
uniform vec3 lineColor;

void main()
{
	if (useFloorGradient != 0)
	{
		float t = clamp((floorX - floorMinX) / (floorMaxX - floorMinX), 0.0, 1.0);
		// Red on the left, yellow at center, and green on the right.
		vec3 rainbow = vec3(1.0 - t, sin(3.14159265 * t), 0.0);
		FragColor = vec4(rainbow, 1.0);
	}
	else
	{
		FragColor = vec4(lineColor, 1.0);
	}
}
