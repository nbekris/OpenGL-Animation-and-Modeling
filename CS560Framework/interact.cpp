////////////////////////////////////////////////////////////////////////
// All keyboard, mouse, and other interactions are implemented here.
// The single entry point, InitInteraction, sets up GLFW callbacks for
// various events that an interactive graphics program needs to
// handle.
//

#include "framework.h"

extern Scene scene;       // Declared in framework.cpp, but used here.

// Some globals used for mouse handling.
double mouseX, mouseY;
bool shifted = false;
bool leftDown = false;
bool middleDown = false;
bool rightDown = false;
bool control = false;

////////////////////////////////////////////////////////////////////////
// Function called to exit
void Quit(void* clientData)
{
    glfwSetWindowShouldClose(scene.window, 1);
}

std::string ACTION[3] = { "Release", "Press", "Repeat" };

////////////////////////////////////////////////////////////////////////
// Called for keyboard actions.

void Keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (ImGui::GetIO().WantCaptureKeyboard) return;

    if (action == GLFW_REPEAT) return; // Because keyboard autorepeat is evil.

    printf("Keyboard %c(%d);  S%d %s M%d\n", key, key, scancode, ACTION[action].c_str(), mods);
    fflush(stdout);

    // Track SHIFT/NO-SHIFT transitions. (The mods parameter should do this, but doesn't.)
    if (key == GLFW_KEY_LEFT_SHIFT || key == GLFW_KEY_RIGHT_SHIFT)
        shifted = !shifted;
    if (key == GLFW_KEY_LEFT_CONTROL || key == GLFW_KEY_RIGHT_CONTROL)
        control = !control;

    if (action == GLFW_PRESS) {
        switch (key) {
        
        case GLFW_KEY_KP_0: case GLFW_KEY_KP_2: case GLFW_KEY_KP_4: case GLFW_KEY_B:
            scene.key = key;
            break;
        case GLFW_KEY_KP_5: case GLFW_KEY_KP_6: case GLFW_KEY_KP_8:
            scene.key = key;
            break;
        case GLFW_KEY_KP_ADD: case GLFW_KEY_KP_SUBTRACT:
            scene.key = key;
            break;
        case GLFW_KEY_ESCAPE: case GLFW_KEY_Q: 
            exit(0);
        }
    }

    else if (action == GLFW_RELEASE) {

        scene.key = 0;
        fflush(stdout);
    }
}

////////////////////////////////////////////////////////////////////////
// Called when a mouse button changes state.
void MouseButton(GLFWwindow* window, int button, int action, int mods)
{

    if (ImGui::GetIO().WantCaptureMouse) return;

    glfwGetCursorPos(window, &mouseX, &mouseY);
    printf("MouseButton %d %d %d %f %f\n", button, action, mods, mouseX, mouseY);

    // @@ Catch any mouse button UP-or-DOWN-transitions you want here.
    // Record any change of state in variables in the scene object.

    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        leftDown = (action == GLFW_PRESS);
    }

    else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        middleDown = (action == GLFW_PRESS);
    }

    else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        rightDown = (action == GLFW_PRESS);
    }
}

////////////////////////////////////////////////////////////////////////
// Called by GLFW when a mouse moves (while a button is down)
void MouseMotion(GLFWwindow* window, double x, double y)
{
    if (ImGui::GetIO().WantCaptureMouse) return;

    // @@ Catch any mouse movement that occurs while any button is
    // down.  It is not reported here *which* button is down, but you
    // should have recorded the DOWN-transition in a previous call to
    // MouseButton.

    // @@ The x,y parameters to this function record the position of
    // the mouse.  Usually you want to know only how much it has moved
    // since the last call.  Hence the following two lines (and this
    // procedure's last two lines) calculate the change in the mouse
    // position.

    // Calculate the change in the mouse position
    double dx = x - mouseX;
    double dy = y - mouseY;

    if (leftDown) {
        scene.spin += (float)dx / 3.0f;
        scene.tilt += (float)dy / 3.0f;
    }

    if (middleDown) {}

    if (rightDown) {
        scene.tr[0] += (float)dx / 40.0f;
        scene.tr[1] -= (float)dy / 40.0f;
    }

    // Record this position
    mouseX = x;
    mouseY = y;
}


void Scroll(GLFWwindow* window, double x, double y)
{
    if (ImGui::GetIO().WantCaptureMouse)  return;

    printf("Scroll %f %f\n", x, y);

    // Figure out the mouse action, and handle accordingly
    // @@ Please don't disable this shifted-scroll-wheel code. 
    if (y > 0.0 && shifted) { // Scroll light in
        scene.lightDist = pow(scene.lightDist, 1.0f / 1.02f);
    }

    // @@ Please don't disable this shifted-scroll-wheel code. 
    else if (y < 0.0 && shifted) { // Scroll light out
        scene.lightDist = pow(scene.lightDist, 1.02f);
    }

    else if (control) { // Scroll light out
        float a = y < 0.0 ? 1.03f : 1.0f / 1.03f;
        scene.ry /= a;
        scene.tr[2] *= a;
        printf("ry: %f\n", scene.ry);
    }

    else if (y > 0.0) {
        scene.tr[2] = pow(scene.tr[2], 1.0f / 1.02f);
    }

    else if (y < 0.0) {
        scene.tr[2] = pow(scene.tr[2], 1.02f);
    }
}

void InitInteraction()
{
    glfwSetKeyCallback(scene.window, Keyboard);
    glfwSetMouseButtonCallback(scene.window, MouseButton);
    glfwSetCursorPosCallback(scene.window, MouseMotion);
    glfwSetScrollCallback(scene.window, Scroll);
}
