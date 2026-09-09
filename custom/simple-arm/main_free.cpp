#include <iostream>
#include "mujoco/mujoco.h"
#include "mujoco/mjrender.h" 
#include "GLFW/glfw3.h"

mjModel* m = nullptr; mjData* d = nullptr;
mjvCamera cam; mjvOption opt; mjvScene scn; mjrContext con;
GLFWwindow* window = nullptr;
double lastx = 0; double lasty = 0;

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    double dx = xpos - lastx; double dy = ypos - lasty;
    lastx = xpos; lasty = ypos;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        mjv_moveCamera(m, mjMOUSE_ROTATE_V, dx/900.0, dy/900.0, &cam);
    }
}

int main() {
    char error[1000] = "Could not load XML model";
    m = mj_loadXML("../models/arm2d_free.xml", nullptr, error, 1000);
    if (!m) { std::cerr << error << std::endl; return 1; }
    d = mj_makeData(m);

    if (!glfwInit()) return 1;
    window = glfwCreateWindow(1200, 900, "Version 1: Free Double Pendulum", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, mouse_move);

    mjv_defaultCamera(&cam); mjv_defaultOption(&opt); mjv_defaultScene(&scn); mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);       

    cam.type = mjCAMERA_FREE; cam.distance = 2.0; cam.lookat[2] = 0.4;

    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {
            mj_step(m, d); // Natively lets gravity swing BOTH joints freely
        }
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window); glfwPollEvents();
    }
    mjv_freeScene(&scn); mjr_freeContext(&con); mj_deleteData(d); mj_deleteModel(m); glfwTerminate();
    return 0;
}
