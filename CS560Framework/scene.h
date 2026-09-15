////////////////////////////////////////////////////////////////////////
// The scene class contains all the parameters needed to define and
// draw a simple scene, including:
//   * Geometry
//   * Light parameters
//   * Material properties
//   * Viewport size parameters
//   * Viewing transformation values
//   * others ...
//
// Some of these parameters are set when the scene is built, and
// others are set by the framework in response to user mouse/keyboard
// interactions.  All of them can be used to draw the scene.

#include "shapes.h"
#include "texture.h"
#include "fbo.h"
#include "gbuffer.h"
#include "Mesh.h"
#include "WireframeFloor.h"

class Shader;


class Scene
{
public:
    GLFWwindow* window;

    // Light parameters
    float lightSpin, lightTilt, lightDist;
    glm::vec3 lightPos = {-30.0f, -60.f, 90.0f};
    glm::vec3 lightCol = {1.0f, 1.0f, 1.0f};
    glm::vec3 lightDir = {1.0f, -1.0f, -1.0f};

    //buffer
    GBuffer gBuffer;
    unsigned int id, bindpoint;

    long double start;

    int key;
    float spin, tilt, speed, ry, front, back;
    glm::vec3 eye;
    glm::vec3 tr;
    float last_time;
    int mode; 

    
    // Viewport
    int width, height;

    // Transformations
    glm::mat4 WorldProj, WorldView, WorldInverse;
    glm::mat4 lightProj, lightView;

    Mesh* model;
    Shape* quad; //deferred shading
    WireframeFloor floor;

    //UI
    bool play = true;
    bool bone = false;
    bool mesh = true;

    // Shader programs
    ShaderProgram* lightingProgram;
    ShaderProgram* gBufferProgram;
    ShaderProgram* basicShader;

    void InitCamera();
    void CreateQuad();
    void DrawQuad();
    void InitializeScene();
    void BuildTransforms();
    void DrawMenu();
    void DrawScene();

};
