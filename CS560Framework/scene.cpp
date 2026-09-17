
#include "math.h"
#include <iostream>
#include <stdlib.h>

#include <glbinding/gl/gl.h>
#pragma warning(disable: 4251)
#include <glbinding/Binding.h>
#pragma warning(disable: 4251)
using namespace gl;

#pragma warning(disable: 4190)
#include <glu.h>

#define GLM_FORCE_RADIANS
#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/ext.hpp>          

#include "framework.h"
//#include "shapes.h"
#include "texture.h"
#include "transform.h"

const float PI = 3.14159f;
const float rad = PI/180.0f;    

glm::mat4 Identity;

#define CHECKERROR {GLenum err = glGetError(); if (err != GL_NO_ERROR) { fprintf(stderr, "OpenGL error (at line scene.cpp:%d): %s\n", __LINE__, gluErrorString(err)); exit(-1);} }

void Scene::InitCamera()
{
    spin = 0.0;
    tilt = 15.0;
    eye = glm::vec3(0.0, -20.0, 10.0);
    speed = 300.0 / 30.0;
    last_time = (float)glfwGetTime();
    tr = glm::vec3(0.0, -3.0, 25.0);

    ry = 0.4f;
    front = 0.5;
    back = 5000.0;
}

void Scene::InitializeScene()
{
    std::cout << "INIT\n";
    glEnable(GL_DEPTH_TEST);
    CHECKERROR;

    glfwGetFramebufferSize(window, &width, &height);

    lightSpin = 90.0;
    lightTilt = -45.0;
    lightDist = 50.0;
    lightCol = { 3.0f, 3.0f, 3.0f };

    key = 0;
    InitCamera();
 
    glGenBuffers(1, &id);
    bindpoint = 1;
    glBindBufferBase(GL_UNIFORM_BUFFER, bindpoint, id);
    CHECKERROR;

    glEnable(GL_DEPTH_TEST);

    //FBO
    gBuffer.Init(width, height);
    CHECKERROR;

    lightingProgram = new ShaderProgram();
    lightingProgram->AddShader("shaders/lightingPhong.vert", GL_VERTEX_SHADER);
    lightingProgram->AddShader("shaders/lightingPhong.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(lightingProgram->programId, 0, "vertex");
    glBindAttribLocation(lightingProgram->programId, 1, "vertexNormal");
    glBindAttribLocation(lightingProgram->programId, 2, "vertexTexture");
    glBindAttribLocation(lightingProgram->programId, 3, "vertexTangent");
    lightingProgram->LinkProgram();

    gBufferProgram = new ShaderProgram();
    gBufferProgram->AddShader("shaders/gBuffer.vert", GL_VERTEX_SHADER);
    gBufferProgram->AddShader("shaders/gBuffer.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(gBufferProgram->programId, 0, "vertex");
    glBindAttribLocation(gBufferProgram->programId, 1, "vertexNormal");
    glBindAttribLocation(gBufferProgram->programId, 2, "vertexTexture");
    glBindAttribLocation(gBufferProgram->programId, 3, "vertexTangent");
    glBindAttribLocation(gBufferProgram->programId, 4, "boneIds");
    glBindAttribLocation(gBufferProgram->programId, 5, "weight");
    gBufferProgram->LinkProgram();

    basicShader = new ShaderProgram();
    basicShader->AddShader("shaders/vertexShader.vert", GL_VERTEX_SHADER);
    basicShader->AddShader("shaders/fragmentShader.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(basicShader->programId, 0, "vertex");
    glBindAttribLocation(basicShader->programId, 1, "vertexNormal");
    glBindAttribLocation(basicShader->programId, 2, "vertexTexture");
    glBindAttribLocation(basicShader->programId, 3, "vertexTangent");
    basicShader->LinkProgram();

    quad = new Quad();
    floor.Initialize();

    model = new Mesh();
    model->LoadMesh("fbx/fbx/roman_D.fbx");
    model->anim_num = 0;
    
    start = glfwGetTime();
}

void Scene::DrawMenu()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Sample UI Box");
    ImGui::Text("sample text");
    
    const char* models[] = { "Roman", "Viking" };
    const char* current_model = models[0];
    if (ImGui::BeginCombo("Model", models[0])) 
    {
        for (int n = 0; n < IM_ARRAYSIZE(models); n++)
        {
            bool is_selected = (current_model == models[n]); 
            if (ImGui::Selectable(models[n], is_selected))
                current_model = models[n];
            if (is_selected)
                ImGui::SetItemDefaultFocus();   
        }

        ImGui::EndCombo();
    }

    if (ImGui::Button((play ? "Stop" : "Play")))
    {
        play = !play;
    }

    ImGui::Checkbox("Draw Bone", &bone);
    ImGui::SameLine();
    ImGui::Checkbox("Draw Mesh", &mesh);

    const char* items[] = { "Cycle", "Idle", "Run", "Fall Down", "Attack" };
    const char* current_item = items[0];

    if (ImGui::BeginCombo("Animation", items[0]))
    {
        for (int n = 0; n < IM_ARRAYSIZE(items); n++)
        {
            bool is_selected = (current_item == items[n]); 
            if (ImGui::Selectable(items[n], is_selected))
                current_item = items[n];
            if (is_selected)
                ImGui::SetItemDefaultFocus();   
        }
        ImGui::EndCombo();
    }
    ImGui::End();


    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    
}

void Scene::BuildTransforms()
{
    float now = (float)glfwGetTime();
    float dist = (now-last_time)*speed;
    last_time = now;
    if (key == GLFW_KEY_KP_8)
        tr[1] -= dist;
    if (key == GLFW_KEY_KP_2)
        tr[1] += dist;
    if (key == GLFW_KEY_KP_6)
        tr[0] -= dist;
    if (key == GLFW_KEY_KP_4)
        tr[0] += dist;
    if (key == GLFW_KEY_KP_5)
        InitCamera();
    if (key == GLFW_KEY_KP_0)
        spin += dist*20.f;
    if (key == GLFW_KEY_KP_ADD)
        tr[2] = pow(tr[2], 1.0f / 1.01f);
    if (key == GLFW_KEY_KP_SUBTRACT)
        tr[2] = pow(tr[2], 1.01f);


    CHECKERROR;
    WorldView = Translate(tr[0], tr[1], -tr[2]) * Rotate(0, tilt - 90) * Rotate(2, spin);
    WorldProj = Perspective((ry*width)/height, ry, front, (mode==0) ? 1000 : back);

}

void Scene::DrawScene()
{
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    BuildTransforms();

    WorldInverse = glm::inverse(WorldView);
    

    int loc, programId;

    //Draw the mesh to the g buffer
    gBufferProgram->UseShader();
    programId = gBufferProgram->programId;

    gBuffer.BindFBO();
    GLenum DrawBuffers[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3 };
    glDrawBuffers(4, DrawBuffers);

    glViewport(0, 0, width, height);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    loc = glGetUniformLocation(programId, "WorldProj");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldProj));
    loc = glGetUniformLocation(programId, "WorldView");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldView));

    long double curr = glfwGetTime();
    long double dt = ((float)(curr - start));
    model->Draw(programId, dt);


    gBuffer.UnbindFBO();
    gBufferProgram->UnuseShader();

 
    glViewport(0, 0, width, height);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ////////////PHONG LIGHTING////////////////////
    lightingProgram->UseShader();
    programId = lightingProgram->programId;
    
    
    loc = glGetUniformLocation(programId, "WorldProj");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldProj));
    loc = glGetUniformLocation(programId, "WorldView");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldView));
    loc = glGetUniformLocation(programId, "WorldInverse");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldInverse));

    loc = glGetUniformLocation(programId, "lightPos");
    glUniform3fv(loc, 1, &(lightPos[0]));   
    loc = glGetUniformLocation(programId, "lightCol");
    glUniform3fv(loc, 1, &(lightCol[0]));

    loc = glGetUniformLocation(programId, "width");
    glUniform1i(loc, width);
    loc = glGetUniformLocation(programId, "height");
    glUniform1i(loc, height);
    loc = glGetUniformLocation(programId, "eyePos");
    glUniform3fv(loc, 1, &(eye[0]));
    
    CHECKERROR;

    gBuffer.BindTexture(0, lightingProgram->programId, "gBufferWorldPos", 0);
    gBuffer.BindTexture(1, lightingProgram->programId, "gBufferNormal", 1);
    gBuffer.BindTexture(2, lightingProgram->programId, "gBufferDiffuse", 2);
    gBuffer.BindTexture(3, lightingProgram->programId, "gBufferSpecular", 3);
    CHECKERROR;

    quad->DrawVAO();
    CHECKERROR; 

    lightingProgram->UnuseShader();

    glClear(GL_DEPTH_BUFFER_BIT);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, gBuffer.fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, width, height,
        0, 0, width, height,
        GL_DEPTH_BUFFER_BIT,
        GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    floor.Draw(*basicShader, WorldProj, WorldView);

    if (bone)
    {
        model->DrawBindPoseSkeleton(*basicShader, WorldProj, WorldView);
    }

}

