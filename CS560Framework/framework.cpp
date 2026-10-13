#include "FrameLimiter.h"
#include "framework.h"
#include "TimingSettings.h"
#include "../tests/SkinningGpuSmoke.h"

Scene scene;

static void error_callback(int error, const char* msg)
{
    fputs(msg, stderr);
}

int main(int argc, char** argv)
{
    const bool skinningSmoke = argc == 2 && std::string(argv[1]) == "--skinning-smoke";
    glfwSetErrorCallback(error_callback);


    // Initialize glfw open a window
    if (!glfwInit())
    {
        exit(EXIT_FAILURE);
    }


    glfwWindowHint(GLFW_RESIZABLE, 1);
    if (skinningSmoke)
    {
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, 0);
    scene.window = glfwCreateWindow(750, 750, "Graphics Framework", NULL, NULL);
    if (!scene.window)
    {
        glfwTerminate();
        exit(-1);
    }

    glfwMakeContextCurrent(scene.window);
    // Use an explicit frame cap instead of the monitor's refresh rate.
    glfwSwapInterval(0);

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

    if (skinningSmoke)
    {
        int result = 0;
        try { RunSkinningGpuSmoke(scene); }
        catch (const std::exception& error)
        {
            fprintf(stderr, "Skinning smoke failed: %s\n", error.what());
            result = 1;
        }
        scene.ReleaseModels();
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(scene.window);
        glfwTerminate();
        return result;
    }

    // Enter the event loop.
    FrameLimiter frameLimiter(TimingSettings::TargetFramesPerSecond);
    while (!glfwWindowShouldClose(scene.window)) {
        const auto frameStart = FrameLimiter::Clock::now();
        glfwPollEvents();

        scene.DrawScene();
        scene.DrawMenu();
        glfwSwapBuffers(scene.window);
        frameLimiter.Wait(frameStart);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    // Mesh owns GL buffers; destroy them while the context is still current.
    scene.ReleaseModels();
    glfwTerminate();
}
