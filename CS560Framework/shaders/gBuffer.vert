/////////////////////////////////////////////////////////////////////////
// Vertex shader for lighting
//
// Copyright 2013 DigiPen Institute of Technology
////////////////////////////////////////////////////////////////////////
#version 330

uniform mat4 WorldView, WorldProj, ModelTr, NormalTr;

layout(location = 0) in vec4 vertex;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec2 vertexTexture;
layout(location = 3) in vec3 vertexTangent;

out vec3 normalVec;
out vec2 texCoord;
out vec4 worldPos;

void main()
{
    gl_Position = WorldProj*WorldView*ModelTr*vertex;
    worldPos = ModelTr*vertex;
    normalVec = vertexNormal*mat3(NormalTr); 
    texCoord = vertexTexture; 
}
