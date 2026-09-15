#include "framework.h"

Scene scene;

static void error_callback(int error, const char* msg)
{
    fputs(msg, stderr);
}

int main(int argc, char** argv)
{
    glfwSetErrorCallback(error_callback);


    // Initialize glfw open a window
    if (!glfwInit())  exit(EXIT_FAILURE);


    glfwWindowHint(GLFW_RESIZABLE, 1);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, 0);
    scene.window = glfwCreateWindow(750, 750, "Graphics Framework", NULL, NULL);
    if (!scene.window) { glfwTerminate();  exit(-1); }

    glfwMakeContextCurrent(scene.window);
    glfwSwapInterval(1);

    // Initialize the OpenGL bindings
    glbinding::Binding::initialize(glfwGetProcAddress);

    // Initialize interaction and the scene to be drawn.
    InitInteraction();
    scene.InitializeScene();

    ImGui::CreateContext();
    //ImGuiIO& io = ImGui::GetIO(); (void)io; 

    ImGui_ImplGlfw_InitForOpenGL(scene.window, true);
    ImGui_ImplOpenGL3_Init();
    printf("OpenGL Version: %s\n", glGetString(GL_VERSION));
    printf("GLSL Version: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
    printf("Rendered by: %s\n", glGetString(GL_RENDERER));
    fflush(stdout);

    // Enter the event loop.
    while (!glfwWindowShouldClose(scene.window)) {
        glfwPollEvents();

        scene.DrawScene();
        scene.DrawMenu();
        glfwSwapBuffers(scene.window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
}