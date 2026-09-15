////////////////////////////////////////////////////////////////////////
// A small library of object shapes (ground plane, sphere, and the
// famous Utah teapot), each created as a Vertex Array Object (VAO).
// This is the most efficient way to get geometry into the OpenGL
// graphics pipeline.
//
// Each vertex is specified as four attributes which are made
// available in a vertex shader in the following attribute slots.
//
// position,        vec4,   attribute #0
// normal,          vec3,   attribute #1
// texture coord,   vec3,   attribute #2
// tangent,         vec3,   attribute #3
//
// An instance of any of these shapes is create with a single call:
//    unsigned int obj = CreateSphere(divisions, &quadCount);
// and drawn by:
//    glBindVertexArray(vaoID);
//    glDrawElements(GL_TRIANGLES, vertexcount, GL_UNSIGNED_INT, 0);
//    glBindVertexArray(0);
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <fstream>
#include <stdlib.h>

#include <glbinding/gl/gl.h>
#pragma warning(disable: 4251)
#include <glbinding/Binding.h>
#pragma warning(disable: 4251)


using namespace gl;

#pragma warning(disable: 4190)
#include <glu.h>                // For gluErrorString
#define CHECKERROR {GLenum err = glGetError(); if (err != GL_NO_ERROR) { fprintf(stderr, "OpenGL error (at line shapes.cpp:%d): %s\n", __LINE__, gluErrorString(err)); exit(-1);} }

#define GLM_FORCE_RADIANS
#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include "math.h"
#include "shapes.h"

const float PI = 3.14159f;
const float rad = PI / 180.0f;

void pushquad(std::vector<glm::ivec3>& Tri, int i, int j, int k, int l)
{
	Tri.push_back(glm::ivec3(i, j, k));
	Tri.push_back(glm::ivec3(i, k, l));
}

// Batch up all the data defining a shape to be drawn (example: the
// teapot) as a Vertex Array object (VAO) and send it to the graphics
// card.  Return an OpenGL identifier for the created VAO.
unsigned int VaoFromTris(std::vector<glm::vec4> Pnt,
	std::vector<glm::vec3> Nrm,
	std::vector<glm::vec2> Tex,
	std::vector<glm::vec3> Tan,
	std::vector<glm::ivec3> Tri)
{
	unsigned int vaoID;
	glGenVertexArrays(1, &vaoID);
	glBindVertexArray(vaoID);

	GLuint Pbuff;
	glGenBuffers(1, &Pbuff);
	glBindBuffer(GL_ARRAY_BUFFER, Pbuff);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * Pnt.size(),
		&Pnt[0][0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	if (Nrm.size() > 0) {
		GLuint Nbuff;
		glGenBuffers(1, &Nbuff);
		glBindBuffer(GL_ARRAY_BUFFER, Nbuff);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * Nrm.size(),
			&Nrm[0][0], GL_STATIC_DRAW);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	if (Tex.size() > 0) {
		GLuint Tbuff;
		glGenBuffers(1, &Tbuff);
		glBindBuffer(GL_ARRAY_BUFFER, Tbuff);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 2 * Tex.size(),
			&Tex[0][0], GL_STATIC_DRAW);
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	if (Tan.size() > 0) {
		GLuint Dbuff;
		glGenBuffers(1, &Dbuff);
		glBindBuffer(GL_ARRAY_BUFFER, Dbuff);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * Tan.size(),
			&Tan[0][0], GL_STATIC_DRAW);
		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	GLuint Ibuff;
	glGenBuffers(1, &Ibuff);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, Ibuff);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * 3 * Tri.size(), &Tri[0][0], GL_STATIC_DRAW);

	glBindVertexArray(0);

	return vaoID;
}

void Shape::ComputeSize()
{
	// Compute min/max
	minP = (Pnt[0]).xyz();
	maxP = (Pnt[0]).xyz();
	for (std::vector<glm::vec4>::iterator p = Pnt.begin(); p < Pnt.end(); p++)
		for (int c = 0; c < 3; c++) {
			minP[c] = std::min(minP[c], (*p)[c]);
			maxP[c] = std::max(maxP[c], (*p)[c]);
		}

	center = (maxP + minP) / 2.0f;
	size = 0.0;
	for (int c = 0; c < 3; c++)
		size = std::max(size, (maxP[c] - minP[c]) / 2.0f);

	float s = 1.0f / size;
	modelTr = Scale(s, s, s) * Translate(-center[0], -center[1], -center[2]);
}

void Shape::MakeVAO()
{
	vaoID = VaoFromTris(Pnt, Nrm, Tex, Tan, Tri);
	count = int(Tri.size());
}

void Shape::DrawVAO()
{
	CHECKERROR;
	glBindVertexArray(vaoID);
	CHECKERROR;
	glDrawElements(GL_TRIANGLES, 3 * count, GL_UNSIGNED_INT, 0);
	CHECKERROR;
	glBindVertexArray(0);
}

Quad::Quad(const int n)
{
	diffuseColor = glm::vec3(0.3, 0.2, 0.1);
	specularColor = glm::vec3(1.0, 1.0, 1.0);
	shininess = 120.0;

	float r = 1.0;
	for (int i = 0; i <= n; i++) {
		float s = i / float(n);
		for (int j = 0; j <= n; j++) {
			float t = j / float(n);
			Pnt.push_back(glm::vec4(s * 2.0 * r - r, t * 2.0 * r - r, 0.0, 1.0));
			Nrm.push_back(glm::vec3(0.0, 0.0, 1.0));
			Tex.push_back(glm::vec2(s, t));
			Tan.push_back(glm::vec3(1.0, 0.0, 0.0));
			if (i > 0 && j > 0) {
				pushquad(Tri,
					(i - 1) * (n + 1) + (j - 1),
					(i - 1) * (n + 1) + (j),
					(i) * (n + 1) + (j),
					(i) * (n + 1) + (j - 1));
			}
		}
	}

	vaoID = VaoFromTris(Pnt, Nrm, Tex, Tan, Tri);
	count = int(Tri.size());
}
