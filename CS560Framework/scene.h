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
#include "Path.h"
#include "PathDebugger.h"
#include "LocomotionController.h"
#include "LineRenderer.h"
#include <array>
#include <memory>

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

    // The target position is the center of the wireframe cube.
    struct TargetObject
    {
        glm::vec3 position{0.0f, 0.0f, 2.0f};
        float size = 0.5f;
    };

    TargetObject target;
    std::unique_ptr<LineRenderer> targetLines;

    float moveSpeed = 5.0f;

    //buffer
    GBuffer gBuffer;
    unsigned int id, bindpoint;

    int key;
    float spin, tilt, speed, ry, front, back;
    glm::vec3 eye;
    glm::vec3 tr;
    int mode; 

    
    // Viewport
    int width, height;

    // Transformations
    glm::mat4 WorldProj, WorldView, WorldInverse;
    glm::mat4 lightProj, lightView;

    // Non-owning pointer to the active cached model.
    Mesh* model = nullptr;
    std::array<std::unique_ptr<Mesh>, 2> loadedModels;
    Shape* quad; //deferred shading
    WireframeFloor floor;

	// Catmull-Rom Path that model follows
    Path travelPath;
    PathDebugger pathDebugger;
    LocomotionController locomotion;
    bool synchronizeLocomotion = true;
    // estimated from planted toe motion in the Run clip, Roman then Viking
    std::array<float, 2> strideWorld{ 11.7f, 12.6f };
    // both assets face -Y after the fixed rotation in GetModelTransform
    std::array<float, 2> assetForwardYawDegrees{ -90.0f, -90.0f };

    //UI
    bool bone = true;
    bool mesh = false;
    int selectedAnimationPreset = 0;
    int selectedModel = 0;
    std::string modelLoadError;

    // Shader programs
    ShaderProgram* lightingProgram;
    ShaderProgram* gBufferProgram;
    ShaderProgram* basicShader;

    void InitCamera();
    void CreateQuad();
    void DrawQuad();
    void InitializeScene();
    bool SelectModel(int index);
    void ReleaseModels();
    void BuildTransforms(double deltaSeconds);
    void DrawMenu();
    void DrawScene(double deltaSeconds);
    bool ConfigureLocomotion();
    void DrawLocomotionMenu();
    // Builds an open ground-plane route ending near the target cube.
    void BeginApproach(float stopDistance);
    float approachStopDistance = 3.0f;
    // Blends to standing during arrival and smoothly faces the target after stopping.
    void UpdateApproach(double deltaSeconds);
    bool approachActive = false;
    bool approachAlreadyNear = false;
    float standingWeight = 0.0f;
    float standingPoseTicks = 0.0f;
    float arrivalYawDegrees = 0.0f;

    // Target Object helper function
    // Builds the cube
    void BuildCube();

};
