// Dear ImGui: standalone example application for GLFW + OpenGL 3, using programmable pipeline
// (GLFW is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)
// If you are new to Dear ImGui, read documentation from the docs/ folder + read the top of imgui.cpp.
// Read online: https://github.com/ocornut/imgui/tree/master/docs

#include "../../lib/imgui/imgui.h"
#include "../../lib/imgui/backends/imgui_impl_glfw.h"
#include "../../lib/imgui/backends/imgui_impl_opengl3.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <mutex>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>

// Include LCM
#include <lcm/lcm-cpp.hpp>
#include "../../lib/lcm/FOC/motor_t.hpp"
#include <sys/select.h>
#include <math.h>

#include "../../lib/implot/implot.h"
#include "../../lib/implot/implot_internal.h"

#include "../../include/dyno.h"   // shared constants and MotorCmd enum
#include <unistd.h>
#include <iostream>

// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to maximize ease of testing and compatibility with old VS compilers.
// To link with VS2010-era libraries, VS2015+ requires linking with legacy_stdio_definitions.lib, which we do using this pragma.
// Your own project should not be affected, as you are likely to link with a newer binary of GLFW that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

// Motor constants are now in dyno.h — only PI is GUI-local
#define PI 3.14159265

std::string ctl_flag;
static int led = 0;
static int state = CMD_NONE;
static int counter = 0;
static double xdata = 0;
static double vq_cmd,iq_cmd,iq,vq;
static double position,velocity,position_cmd,velocity_cmd,torque,torque_cmd;
static double elec_power,mech_power;

// Protects the sensor globals above against concurrent access by
// updaterThread (Handler::handleMessage writes) and the main/render
// thread (FOC_RealtimePlots reads).
static std::mutex sensor_mutex;

lcm::LCM lcm2;
FOC::motor_t motor_data;

/*
class Handler {
  public:
    ~Handler() {}
    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const mypackage::Position *msg)
    {
        xdata = msg->x;
    }
};
*/

class Handler {
  public:
    ~Handler() {}
    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const FOC::motor_t *msg)
    {
        std::lock_guard<std::mutex> lock(sensor_mutex);
        position   = msg->position;
        velocity   = msg->velocity;
        vq         = msg->vq;
        iq         = msg->iq;
        torque     = msg->torque;
        elec_power = iq * vq;
        mech_power = torque * velocity;
    }
};
/*
class MotorHandler {
  public:
    ~MotorHandler() {}
    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const FOC::motor_t *msg)
    {
        vq_cmd = msg->vq_cmd;
        iq = msg->iq;
        vq = msg->vq;
        position = msg->position;
        velocity = msg->velocity;
        torque = msg->torque;
        power = iq*vq;
    }
};
*/


static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

// utility structure for realtime plot
struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000) {
        MaxSize = max_size;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
    }
};

// utility structure for realtime plot
struct RollingBuffer {
    float Span;
    ImVector<ImVec2> Data;
    RollingBuffer() {
        Span = 10.0f;
        Data.reserve(2000);
    }
    void AddPoint(float x, float y) {
        float xmod = fmodf(x, Span);
        if (!Data.empty() && xmod < Data.back().x)
            Data.shrink(0);
        Data.push_back(ImVec2(xmod, y));
    }
};

// Groups the 8 scrolling data buffers used by FOC_Plots().
struct PlotData {
    ScrollingBuffer pos, vel, torque, iq, vq, elec_power, vq_cmd, mech_power;
};

// Forward declarations
void FOC_ControlPanel(bool &paused);
void FOC_Plots(float t, bool paused, PlotData &pd);

// ============================================================
// FOC_ControlPanel — mode selector, buttons, sliders, LCM publish
// ============================================================
void FOC_ControlPanel(bool &paused)
{
    static int   clicked            = 0;
    static int   pause_clicked      = 0;
    static int   debug_clicked      = 0;
    static int   calibrate_clicked  = 0;
    static int   voltage_foc_clicked= 0;
    static int   off_clicked        = 0;
    static int   vel_ctl_clicked    = 0;
    static int   dyno_test_clicked  = 0;
    static float dq_      = 0.0f;
    static float q_       = 0.0f;
    static float kp_float = 0.0f;
    static float kd_float = 0.0f;
    static float voltage_ = 0.0f;
    static float current_ = 0.0f;
    static float torque_  = 0.0f;
    static int   e        = 0;      // selected control mode

    // ---- Mode selector ----
    ImGui::RadioButton("Position Control",         &e, 0); ImGui::SameLine();
    ImGui::RadioButton("Velocity Control",         &e, 1); ImGui::SameLine();
    ImGui::RadioButton("Open Loop Voltage Control",&e, 2);
    ImGui::RadioButton("Current Control",          &e, 3);
    ImGui::RadioButton("Torque Control",           &e, 4);

    // ---- Controller toggle ----
    if (ImGui::Button("Controller"))
    {
        clicked++;
        counter++;
    }
    if (clicked & 1)
    {
        ImGui::SameLine();
        ImGui::Text("Controller = ON");
        if (e == 0)
        {
            state = CMD_POSITION;
            motor_data.cmd_id       = state;
            motor_data.kp           = kp_float / GAIN;
            motor_data.kd           = kd_float / GAIN;
            motor_data.position_cmd = q_;
            lcm2.publish("MOTOR", &motor_data);
        }
        else if (e == 1)
        {
            state = CMD_VELOCITY;
            motor_data.cmd_id       = state;
            motor_data.kp           = kp_float / GAIN;
            motor_data.kd           = kd_float / GAIN;
            motor_data.velocity_cmd = dq_;
            lcm2.publish("MOTOR", &motor_data);
        }
        else if (e == 2)
        {
            state = CMD_VOLTAGE_FOC;
            motor_data.cmd_id  = state;
            motor_data.vq_cmd  = voltage_;
            lcm2.publish("MOTOR", &motor_data);
        }
        else if (e == 3)
        {
            state = CMD_CURRENT;
            motor_data.cmd_id  = state;
            motor_data.iq_cmd  = current_;
            lcm2.publish("MOTOR", &motor_data);
        }
        else if (e == 4)
        {
            state = CMD_TORQUE;
            motor_data.cmd_id    = state;
            motor_data.kp        = kp_float / GAIN;
            motor_data.kd        = kd_float / GAIN;
            motor_data.torque_cmd= torque_;
            lcm2.publish("MOTOR", &motor_data);
        }
        else
        {
            state = CMD_OFF;
            motor_data.cmd_id = state;
            lcm2.publish("MOTOR", &motor_data);
        }
        clicked = 1;
        if (counter % 2 != 0)
        {
            motor_data.cmd_id = state;
            lcm2.publish("MOTOR", &motor_data);
            counter++;
        }
    }
    else
    {
        ImGui::SameLine();
        ImGui::Text("Controller = OFF");
        if (counter % 2 != 0)
        {
            std::cout << "Controller Off\n";
            state = CMD_OFF;
            motor_data.cmd_id = state;
            lcm2.publish("MOTOR", &motor_data);
            counter++;
            state = CMD_NONE;
        }
    }

    // ---- Animate (pause/unpause plots) ----
    if (ImGui::Button("Animate"))
        pause_clicked++;
    if (pause_clicked & 1)
    {
        ImGui::SameLine();
        ImGui::Text("Running");
        paused = false;
    }
    else
    {
        ImGui::SameLine();
        ImGui::Text("Paused");
        paused = true;
    }

    // ---- Debug (toggle LED) ----
    if (ImGui::Button("Debug"))
        debug_clicked++;
    if (debug_clicked & 1)
    {
        led = (led == 0) ? 1 : 0;
        state = CMD_TOGGLE_LED;
        motor_data.cmd_id = state;
        motor_data.led    = led;
        lcm2.publish("MOTOR", &motor_data);
        state = CMD_NONE;
        debug_clicked = 0;
    }
    ImGui::SameLine();
    ImGui::Text(led == 0 ? "Toggle OFF" : "Toggle ON");

    // ---- Calibrate ----
    if (ImGui::Button("Calibrate"))
        calibrate_clicked++;
    if (calibrate_clicked & 1)
    {
        std::cout << "Calibrate\n";
        state = CMD_CALIBRATE;
        motor_data.cmd_id = state;
        lcm2.publish("MOTOR", &motor_data);
        state = CMD_NONE;
        calibrate_clicked = 0;
    }

    // ---- Dyno Test ----
    if (ImGui::Button("Dyno Test"))
        dyno_test_clicked++;
    if (dyno_test_clicked & 1)
    {
        std::cout << "Dyno Test\n";
        state = CMD_DYNO_TEST;
        dyno_test_clicked = 0;
        motor_data.cmd_id = state;
        motor_data.vq_cmd = 0;
        lcm2.publish("MOTOR", &motor_data);
    }

    // ---- Manual data logging toggle ----
    static bool log_active = false;
    if (log_active)
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.18f, 0.18f, 1.0f));
    if (ImGui::Button(log_active ? "Stop Logging" : "Start Logging")) {
        log_active = !log_active;
        motor_data.cmd_id = CMD_LOG_TOGGLE;
        lcm2.publish("MOTOR", &motor_data);
        motor_data.cmd_id = CMD_NONE;
    }
    if (log_active)
        ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::Text(log_active ? "Logging ON" : "Logging OFF");

    // ---- Sliders ----
    ImGui::SliderFloat("Angular Position (rad)",    &q_,       -ONE_REV * GR,     ONE_REV * GR);
    ImGui::SliderFloat("Angular Velocity (rad/sec)",&dq_,      -SPEED_MAX * 0.1f, SPEED_MAX * 0.1f);
    ImGui::SliderFloat("Q-Axis Voltage (V)",        &voltage_,  -V_MAX * 0.5f,     V_MAX * 0.5f);
    ImGui::SliderFloat("Q-Axis Current (A)",        &current_,  -I_MAX * 0.25f,    I_MAX * 0.25f);
    ImGui::SliderFloat("Torque (Nm)",               &torque_,   -0.15f,            0.15f);
    ImGui::SliderFloat("KP",                        &kp_float,   0.0f,             KP_MAX);
    ImGui::SliderFloat("KD",                        &kd_float,   0.0f,             0.1f);
}

// ============================================================
// FOC_Plots — history slider + 6-panel ImPlot subplot grid
// ============================================================
void FOC_Plots(float t, bool paused, PlotData &pd)
{
    static int   rows      = 3;
    static int   cols      = 2;
    static float rratios[] = {1, 1, 1, 1, 1, 1};
    static float cratios[] = {1, 1, 1, 1, 1, 1};
    static ImVec4 color    = ImVec4(1, 1, 0, 1);
    static float  thickness = 5;
    static float  history   = 10.0f;
    static ImPlotSubplotFlags flags = ImPlotSubplotFlags_None;

    ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");

    if (ImPlot::BeginSubplots("", rows, cols, ImVec2(600, 500), flags, rratios, cratios))
    {
        if (ImPlot::BeginPlot("##Digital"))
        {
            color = ImVec4(255, 0, 0, 1);
            ImPlot::SetNextLineStyle(color, thickness);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -ONE_REV * GR, ONE_REV * GR);
            ImPlot::SetupAxes("Time (sec)", "Position (rad)");
            ImPlot::PlotLine("Position",
                &pd.pos.Data[0].x, &pd.pos.Data[0].y,
                pd.pos.Data.size(), 0, pd.pos.Offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("##Digital"))
        {
            color = ImVec4(255, 0, 0, 1);
            ImPlot::SetNextLineStyle(color, thickness);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -2.0, 2.0);
            ImPlot::SetupAxes("Time (sec)", "VQ (V)");
            ImPlot::PlotLine("VQ CL",
                &pd.vq.Data[0].x, &pd.vq.Data[0].y,
                pd.vq.Data.size(), 0, pd.vq.Offset, 2 * sizeof(float));
            color = ImVec4(255, 0, 0, 0.5);
            ImPlot::SetNextLineStyle(color, thickness);
            ImPlot::PlotLine("VQ OL",
                &pd.vq_cmd.Data[0].x, &pd.vq_cmd.Data[0].y,
                pd.vq_cmd.Data.size(), 0, pd.vq_cmd.Offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("##Digital"))
        {
            color = ImVec4(0, 255, 0, 1);
            ImPlot::SetNextLineStyle(color, thickness);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -80.0, 80.0);
            ImPlot::SetupAxes("Time (sec)", "Velocity (rad/s)");
            ImPlot::PlotLine("Velocity",
                &pd.vel.Data[0].x, &pd.vel.Data[0].y,
                pd.vel.Data.size(), 0, pd.vel.Offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("##Digital"))
        {
            color = ImVec4(0, 255, 0, 1);
            ImPlot::SetNextLineStyle(color, thickness);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -2.5, 2.5);
            ImPlot::SetupAxes("Time (sec)", "IQ (A)");
            ImPlot::PlotLine("Current Data",
                &pd.iq.Data[0].x, &pd.iq.Data[0].y,
                pd.iq.Data.size(), 0, pd.iq.Offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("##Digital"))
        {
            color = ImVec4(0, 0, 255, 1);
            ImPlot::SetNextLineStyle(color, thickness);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -1.5, 1.5);
            ImPlot::SetupAxes("Time (sec)", "Torque (Nm)");
            ImPlot::PlotLine("Torque",
                &pd.torque.Data[0].x, &pd.torque.Data[0].y,
                pd.torque.Data.size(), 0, pd.torque.Offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("##Digital"))
        {
            color = ImVec4(0, 0, 255, 1);
            ImPlot::SetNextLineStyle(color, thickness);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -100, 100);
            ImPlot::SetupAxes("Time (sec)", "Power (W)");
            ImPlot::PlotLine("Elec Power",
                &pd.elec_power.Data[0].x, &pd.elec_power.Data[0].y,
                pd.elec_power.Data.size(), 0, pd.elec_power.Offset, 2 * sizeof(float));
            color = ImVec4(0, 0, 255, 0.5);
            ImPlot::SetNextLineStyle(color, thickness);
            ImPlot::PlotLine("Mech Power",
                &pd.mech_power.Data[0].x, &pd.mech_power.Data[0].y,
                pd.mech_power.Data.size(), 0, pd.mech_power.Offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }
        ImPlot::EndSubplots();
    }
}

// ============================================================
// FOC_RealtimePlots — orchestrator: owns timing and buffers,
//                     delegates to FOC_ControlPanel / FOC_Plots
// ============================================================
void FOC_RealtimePlots()
{
    ctl_flag = "1";

    static bool     paused = false;
    static float    t      = 0;
    static PlotData pd;

    ImGui::Begin("Motor Controller");

    // Snapshot sensor globals under lock, then feed buffers without holding the lock
    double s_position, s_velocity, s_torque, s_iq, s_vq, s_elec_power, s_vq_cmd, s_mech_power;
    {
        std::lock_guard<std::mutex> lock(sensor_mutex);
        s_position   = position;
        s_velocity   = velocity;
        s_torque     = torque;
        s_iq         = iq;
        s_vq         = vq;
        s_elec_power = elec_power;
        s_vq_cmd     = vq_cmd;
        s_mech_power = mech_power;
    }

    // Advance time and append sensor readings to scrolling buffers each frame
    if (!paused) {
        t += ImGui::GetIO().DeltaTime;
        pd.pos.AddPoint(t,        s_position);
        pd.vel.AddPoint(t,        s_velocity);
        pd.torque.AddPoint(t,     s_torque);
        pd.iq.AddPoint(t,         s_iq);
        pd.vq.AddPoint(t,         s_vq);
        pd.elec_power.AddPoint(t, s_elec_power);
        pd.vq_cmd.AddPoint(t,     s_vq_cmd);
        pd.mech_power.AddPoint(t, s_mech_power);
    }

    FOC_ControlPanel(paused);
    FOC_Plots(t, paused, pd);

    ImGui::End();
}

int main(int, char**)
{
    // Setup window
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // Decide GL+GLSL versions
    #if defined(IMGUI_IMPL_OPENGL_ES2)
        // GL ES 2.0 + GLSL 100
        const char* glsl_version = "#version 100";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
    #elif defined(__APPLE__)
        // GL 3.2 + GLSL 150
        const char* glsl_version = "#version 150";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
    #else
        // GL 3.0 + GLSL 130
        const char* glsl_version = "#version 130";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
        //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
    #endif

    lcm::LCM lcm;
    if (!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("GUI", &Handler::handleMessage, &handlerObject);

    // Start the ParameterUpdater in a separate thread
    std::thread updaterThread([&]()
    {
        while(0==lcm.handle())
        {
            ;
        }
    }
    );


    if (!lcm2.good())
        return 1;


    /*
    MotorHandler motor_handlerObject;
    lcm2.subscribe("MOTOR", &MotorHandler::handleMessage, &motor_handlerObject);
    // Start the ParameterUpdater in a separate thread
    std::thread motor_updaterThread([&]()
    {
        while(0==lcm2.handle())
        {
            ;
        }
    }
    );
    */
    //motor_data.position = 100;
    //lcm2.publish("MOTOR", &motor_data);

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(700, 1000, "Motor Control", NULL, NULL); // 1280,720
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    //ImGui::StyleColorsDark();
    ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return NULL. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != NULL);

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    ctl_flag = "0";

    // Modify style
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 12.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 12.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ScrollbarRounding, 12.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_GrabRounding, 12.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
    //ImGui::PushStyleVar(ImGuiStyleVar_TabBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(20.0f,20.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10.0f,4.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(5.0f,6.0f));

    //ImGuiIO& io = ImGui::GetIO();
    //io = ImGui::GetIO();
    ImFontConfig config;
    config.GlyphOffset.y = 0.0f;
    float size_pixels = 18.0f;
    //const char* filename = "/home/campo074/Documents/ImGuiApps/MyApplication/misc/fonts/ProggyClean.ttf"; // size=13;GlyphOffset.y=1
    //const char* filename = "/home/campo074/Documents/ImGuiApps/MyApplication/misc/fonts/ProggyTiny.ttf"; //size=10;GlyphOffset.y=1
    const char* filename = "/home/campo074/Documents/ImGuiApps/MyApplication/misc/fonts/DroidSans.ttf";
    //const char* filename = "/home/campo074/Documents/ImGuiApps/MyApplication/misc/fonts/Cousine_Regular.ttf";
    //const char* filename = "/home/campo074/Documents/ImGuiApps/MyApplication/misc/fonts/Karla-Regular.ttf";
    //const char* filename = "/home/campo074/Documents/ImGuiApps/MyApplication/misc/fonts/Roboto-Medium.ttf";
    ImFont* font2 = io.Fonts->AddFontFromFileTTF(filename, size_pixels, &config);

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.

        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // === My Code Goes Here ===


        //ImGui::PushStyleVar(ImGuiStyleVar_XXXX);
                //5. Implot Demo
        //lcm.handle(); // query the latest message

        // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).

        if (show_demo_window)
        {
            ImGui::ShowDemoWindow(&show_demo_window);
        }

        ImPlot::ShowDemoWindow(); // Put features below this line to have them on the same window
        FOC_RealtimePlots();

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

     // Join the updater thread
    updaterThread.join();
    //motor_updaterThread.join();

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
