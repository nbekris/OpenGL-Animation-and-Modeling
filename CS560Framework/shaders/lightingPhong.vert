/////////////////////////////////////////////////////////////////////////
// Vertex shader for lighting
//
// Copyright 2013 DigiPen Institute of Technology
////////////////////////////////////////////////////////////////////////
#version 330

uniform mat4 WorldView, WorldInverse, WorldProj, ModelTr, NormalTr;


layout(location = 0) in vec4 vertex;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec2 vertexTexture;
layout(location = 3) in vec3 vertexTangent;
layout(location = 4) in ivec4 boneIds;
layout(location = 5) in vec4 weight;

out vec2 texCoord;

void main()
{
    gl_Position = vertex;
    texCoord = vertexTexture;
    
}
