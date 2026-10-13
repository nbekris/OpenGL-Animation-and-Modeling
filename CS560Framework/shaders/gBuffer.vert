#version 330 core

uniform mat4 WorldView, WorldProj, ModelTr, MeshNodeTr;
layout(location = 0) in vec4 vertex;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec2 vertexTexture;
layout(location = 3) in vec3 vertexTangent;
layout(location = 4) in ivec4 boneIds;
layout(location = 5) in vec4 weight;

layout(std140) uniform SkinPalette
{
    mat4 gBones[128];
};

out vec3 normalVec;
out vec2 texCoord;
out vec4 worldPos;

void main()
{
    mat4 deformation = MeshNodeTr;
    float totalWeight = dot(weight, vec4(1.0));

    // Linear Blend Skinning logic
    if (totalWeight > 0.0)
    {
        deformation = mat4(0.0);
        for (int i = 0; i < 4; ++i)
        {
            if (weight[i] > 0.0)
            {
                deformation += (weight[i] / totalWeight) * gBones[boneIds[i]];
            }
        }
    }

    mat4 worldTransform = ModelTr * deformation;
    worldPos = worldTransform * vertex;
    gl_Position = WorldProj * WorldView * worldPos;

    mat3 linearTransform = mat3(worldTransform);
    // Opposing bone rotations can make an LBS blend singular.
    vec3 transformedNormal;
    if (abs(determinant(linearTransform)) > 1e-8)
    {
        transformedNormal = transpose(inverse(linearTransform)) * vertexNormal;
    }
    else
    {
        transformedNormal = mat3(ModelTr * MeshNodeTr) * vertexNormal;
    }

    float lengthSquared = dot(transformedNormal, transformedNormal);
    normalVec = lengthSquared > 1e-12
        ? transformedNormal * inversesqrt(lengthSquared) : vec3(0.0, 1.0, 0.0);
    texCoord = vertexTexture;
}
