
#version 330

uniform mat4 WorldView, WorldInverse, WorldProj, ModelTr, NormalTr;

in vec4 vertex;
in vec3 vertexNormal;
in vec2 vertexTexture;
in vec3 vertexTangent;

out vec2 texCoord;
out float floorX;

void main()
{
    vec4 worldPosition = ModelTr * vertex;
    gl_Position = WorldProj * WorldView * worldPosition;
    
    texCoord = vertexTexture; 
    floorX = worldPosition.x;
}
