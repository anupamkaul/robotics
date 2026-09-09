#include <iostream>
#include <cmath>
#include <chrono>
#include "mujoco/mujoco.h"
#include "mujoco/mjrender.h" // Exposes mjFONTS to compiler
#include "GLFW/glfw3.h"

// Global MuJoCo simulation structures
mjModel* m = nullptr;
mjData* d = nullptr;

// Global visual rendering structures
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// Global GLFW window pointer 
GLFWwindow* window = nullptr;

// Mouse callback tracking states
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
int current_mods = 0;  // Keeps track of active modifier keys (like Shift)

// Mouse button interaction callback function
void mouse_button(GLFWwindow* window, int button, int action, int mods) {
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);
    current_mods = mods;  // Store modifier flags natively
    glfwGetCursorPos(window, &lastx, &lasty);
}

// Mouse movement interaction callback function (Handles rotation and zoom)
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    int action;
    if (button_right) {
        action = (current_mods & GLFW_MOD_SHIFT) ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = (current_mods & GLFW_MOD_SHIFT) ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    // FIXED: Removed the extra &scn parameter to match MuJoCo's 5-argument signature
    mjv_moveCamera(m, action, dx/height, dy/height, &cam);
}

// Scroll wheel callback function (Handles incremental zooming)
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // FIXED: Removed &scn parameter here as well
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &cam);
}

int main() {
    char error[1000] = "Could not load XML model";

    // 1. Load your 2-Joint link arm model structure
    m = mj_loadXML("../models/arm2d.xml", nullptr, error, 1000);
    if (!m) {
        std::cerr << "MuJoCo Load Error: " << error << std::endl;
        return 1;
    }
    d = mj_makeData(m);

    // 2. Initialize GLFW and create your native window framework
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW" << std::endl;
        return 1;
    }
    window = glfwCreateWindow(1200, 900, "2-Joint Arm Control Sandbox", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vertical sync for smooth visual frame pacing

    // 3. Register user interface mouse event listeners
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);

    // 4. Initialize abstract scene objects and link the local GPU rendering context
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(m, &scn, 2000);                    // Max 2000 visual geoms allowed
    mjr_makeContext(m, &con, mjFONTSCALE_150);       // Initialize virtual font textures

    // Automatically configure camera to look right at the moving pendulum center
    cam.type = mjCAMERA_FREE;
    cam.distance = 2.5;
    cam.lookat[0] = 0.0; cam.lookat[1] = 0.0; cam.lookat[2] = 0.8;
    cam.elevation = -15;

    std::cout << "Visual runtime initialized successfully. Enjoy the simulation!" << std::endl;

    // 5. Unified Step Physics + Step Graphics Pipeline Loop
    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        
        // Run physics steps forward to catch up with display real wall-clock timing frame updates
        while (d->time - simstart < 1.0/60.0) {
            // Apply structural sine/cosine position command inputs to track path waves
            double target_joint1 = 0.8 * std::sin(d->time * 3.0);
            double target_joint2 = 0.6 * std::cos(d->time * 4.0);

	    // controlling the actuator motions here: 
            d->ctrl[0] = target_joint1;
            d->ctrl[1] = target_joint2;

            mj_step(m, d);
        }

        // --- GRAPHICS DATA SYNC PASS ---
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // Update coordinate transforms inside the abstract geometry array
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        
        // Push geometric layout updates straight onto your macOS screen canvas layer
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents(); // Read trackpad inputs safely without interface stalling
    }

    // 6. Free all allocated system and GPU context buffers cleanly
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;
}

