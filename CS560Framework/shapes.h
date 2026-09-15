
#ifndef _SHAPES
#define _SHAPES

#include "transform.h"

#include <vector>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>


class Shape
{
public:

    unsigned int vaoID;

    std::vector<glm::vec4> Pnt;
    std::vector<glm::vec3> Nrm;
    std::vector<glm::vec2> Tex;
    std::vector<glm::vec3> Tan;

    glm::vec3 diffuseColor, specularColor;
    float shininess;

    std::vector<glm::ivec3> Tri;
    unsigned int count;

    glm::vec3 minP, maxP;
    glm::vec3 center;
    float size;
    glm::mat4 modelTr;

    // Constructor and destructor
    Shape() {}
    virtual ~Shape() {}

    virtual void ComputeSize();
    virtual void MakeVAO();
    virtual void DrawVAO();
};

class Quad: public Shape
{
public:
    Quad(const int n=1);
};


#endif
