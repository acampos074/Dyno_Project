// Dear ImGui: standalone example application for GLFW + OpenGL 3, using programmable pipeline
// (GLFW is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)
// If you are new to Dear ImGui, read documentation from the docs/ folder + read the top of imgui.cpp.
// Read online: https://github.com/ocornut/imgui/tree/master/docs

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

// Include LCM
#include <lcm/lcm-cpp.hpp>
#include "Position.hpp"
#include "motor_t.hpp"
#include <sys/select.h>
#include <math.h>

#include "implot.h"
#include "implot_internal.h"

//#include "RedisClient.h"
//#include "USB2CAN.h"
#include <unistd.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to maximize ease of testing and compatibility with old VS compilers.
// To link with VS2010-era libraries, VS2015+ requires linking with legacy_stdio_definitions.lib, which we do using this pragma.
// Your own project should not be affected, as you are likely to link with a newer binary of GLFW that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

#define PI 3.14159265

//auto redis_client = RedisClient();
// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY  = "q";
const std::string JOINT_CMD_KEY  = "q_cmd";
const std::string CONTROL_FLAG_KEY  = "flag";
const std::string JOINT_VELOCITIES_KEY = "dq";
const std::string JOINT_TORQUES_KEY = "u";
const std::string JOINT_TORQUES_CMD_KEY = "tau";
const std::string redis_ctrl_flag = "redis_ctrl_flag";
const std::string redis_term_flag = "redis_term_flag";
const std::string LED_KEY = "led";
const std::string GENERAL_COMMAND_KEY = "cmd";
const std::string JOINT_VELOCITY_CMD_KEY = "vel_cmd";
const std::string KP_KEY = "kp";
const std::string KD_KEY = "kd";
Eigen::VectorXd v_q(1);
Eigen::VectorXd v_dq(1);
Eigen::VectorXd v_u(1);
Eigen::VectorXd v_tau_cmd(1);
Eigen::VectorXd v_kp(1);
Eigen::VectorXd v_kd(1);
std::string ctl_flag;
std::string led_flag;
static int gen_cmd;
static int counter = 0;
//static std::string dq_cmd;
Eigen::VectorXd dq_cmd(1);

static double xdata = 0;
static double vq_cmd;
lcm::LCM lcm2;
FOC::motor_t motor_data;

class Handler {
  public:
    ~Handler() {}
    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const mypackage::Position *msg)
    {
        xdata = msg->x;
    }
}; 

class MotorHandler {
  public:
    ~MotorHandler() {}
    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const FOC::motor_t *msg)
    {
        vq_cmd = msg->vq_cmd;
    }
}; 


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


void FOC_RealtimePlots() {

    // ===== My Code =====

    v_u[0] = 0;
    /*
    redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY,v_q);
    redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY,v_dq); 
    redis_client.getEigenMatrixDerivedString(JOINT_TORQUES_CMD_KEY,v_tau_cmd);
    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_KEY,v_u);
    */
    ctl_flag = "1";
    /*
    redis_client.setCommandIs(CONTROL_FLAG_KEY,ctl_flag);
    redis_client.setCommandIs(LED_KEY,led_flag);
    */
    
    static int clicked = 0;
    static int pause_clicked = 0;
    static int debug_clicked = 0;
    static int calibrate_clicked = 0;
    static int voltage_foc_clicked = 0;
    static int off_clicked = 0;
    static int vel_ctl_clicked = 0;
    static int dyno_test_clicked = 0;
    static float f = 0.0f;
    static float kp_float = 0.05f;
    static float kd_float = 0.012f;

    // ===============

    ImGui::Begin("Motor Controller");
    //ImGui::BulletText("Move your mouse to change the data!");
    //ImGui::BulletText("This example assumes 60 FPS. Higher FPS requires larger buffer size.");
    static bool paused = false;
    //static ScrollingBuffer sdata1, sdata2;
    static ScrollingBuffer   rdata1, rdata2, rdata3, rdata4, rdata5;
    ImVec2 mouse = ImGui::GetMousePos();
    static float t = 0;
    float gear_ratio = 12.0f;
    if (!paused) {
        t += ImGui::GetIO().DeltaTime;
        //rdata1.AddPoint(t, v_q[0]/gear_ratio);
        
        rdata1.AddPoint(t,xdata);
        //rdata1.AddPoint(t,2*sin(2*PI*t));
        rdata2.AddPoint(t, v_dq[0]);
        rdata3.AddPoint(t, v_tau_cmd[0]);
        rdata4.AddPoint(t,v_tau_cmd[0]*2);
        //vq_cmd_internal = vq_cmd;
        rdata5.AddPoint(t,vq_cmd);
    }

    static ImPlotSubplotFlags flags = ImPlotSubplotFlags_None;

    // === Buttons ====
    static int e = 0;
    ImGui::RadioButton("Position Control", &e, 0); ImGui::SameLine();
    ImGui::RadioButton("Velocity Control", &e, 1); ImGui::SameLine();
    ImGui::RadioButton("Open Loop Voltage Control", &e, 2);

    // gen_cmd KEY :
    // 1: Calibration
    // 2: Controller off
    // 3: voltage FOC
    // 4: Position control
    // 5: Velocity control
    // 6: dyno test

    if (ImGui::Button("Controller"))
    {
        clicked++;
        counter++;
    }                            // Buttons return true when clicked (most widgets return true when edited/activated)
        
    if(clicked & 1)
    {
        ImGui::SameLine();
        ImGui::Text("Controller = ON");
        if(e == 0)
            gen_cmd = 4;
        else if(e == 1)
            gen_cmd = 5;
        else if(e == 2)
            gen_cmd = 3;
        else
            gen_cmd = 2; // turn off controller by default
        clicked = 1;
        if(counter%2 != 0)
        {
            motor_data.cmd_id = gen_cmd;
            lcm2.publish("MOTOR", &motor_data);
            counter++;
        }
    }
    else
    {
        ImGui::SameLine();
        ImGui::Text("Controller = OFF");
        gen_cmd = 2;
        //clicked = 0;
        if(counter%2 != 0)
        {
            motor_data.cmd_id = gen_cmd;
            lcm2.publish("MOTOR", &motor_data);
            counter++;
        }
    }
    // -------- Animate button --------
    if (ImGui::Button("Animate"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
        pause_clicked++;
    if(pause_clicked & 1)
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
    // -------- Debug button --------
    if (ImGui::Button("Debug"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
        debug_clicked++;
    if(debug_clicked & 1 )
    {
        ImGui::SameLine();
        ImGui::Text("Toggle ON");
        led_flag = "1";
        //motor_data.cmd_it = gen_cmd;
        //lcm2.publish("MOTOR", &motor_data);
    }
    else
    {
        ImGui::SameLine();
        ImGui::Text("Toggle OFF");
        led_flag = "0";
        //motor_data.cmd_it = gen_cmd;
        //lcm2.publish("MOTOR", &motor_data);
    }
    // -------- Calibrate --------
    if (ImGui::Button("Calibrate"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
        calibrate_clicked++;
    if(calibrate_clicked & 1 )
    {
        //ImGui::SameLine();
        //ImGui::Text("Toggle ON");
        gen_cmd = 1;
        calibrate_clicked = 0;
        motor_data.cmd_id = gen_cmd;
        lcm2.publish("MOTOR", &motor_data);
    }
    

    // -------------------
    // TODO DO: 
    // Implement test sequence for dyno test
    // -------------------
    // -------- Dyno Test --------
    if (ImGui::Button("Dyno Test"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
        dyno_test_clicked++;
    if(dyno_test_clicked & 1 )
    {
        //ImGui::SameLine();
        //ImGui::Text("Running Test");
        gen_cmd = 6;
        dyno_test_clicked = 0;
        motor_data.cmd_id = gen_cmd;
        motor_data.vq_cmd = 0; // start sequence with zero volts
        motor_data.dyno_test_flag = 1; // turn-on test sequence
        lcm2.publish("MOTOR", &motor_data);
    }

    
    // -------------------
    // TODO DO: 
    // Add button to store csv data for post-proccesing & analysis
    // -------------------

    // ================
    ImGui::SliderFloat("Angular Position (rad)", &f, -10.0f, 10.0f);// Edit 1 float using a slider from 1.0f to 65.0f
    ImGui::SliderFloat("Angular Velocity (rad/sec)", &f, -65.0f, 65.0f);// Edit 1 float using a slider from 1.0f to 65.0f
    ImGui::SliderFloat("KP", &kp_float, 0.0f, 0.1f);
    ImGui::SliderFloat("KD", &kd_float, 0.0f, 0.1f);
    //char buf[100];    
    //gcvt(f,5,buf);
    //itoa(f,buf,10);
    dq_cmd << f;
    v_kp << kp_float;
    v_kd << kd_float;
    /*
    redis_client.setEigenMatrixDerivedString(JOINT_VELOCITY_CMD_KEY,dq_cmd);
    redis_client.setEigenMatrixDerivedString(KP_KEY,v_kp);
    redis_client.setEigenMatrixDerivedString(KD_KEY,v_kd);
    redis_client.setCommandIs(GENERAL_COMMAND_KEY,gen_cmd);
    */

    // Reset the command ID to zero
    //gen_cmd = 0;
    //motor_data.cmd_id = gen_cmd;
    //lcm2.publish("MOTOR", &motor_data);
    

    static int rows = 3;
    static int cols = 2;
    //ImGui::SliderInt("Rows",&rows,1,5);
    //ImGui::SliderInt("Cols",&cols,1,5);
    static float rratios[] = {1,1,1,1,1,1};
    static float cratios[] = {1,1,1,1,1,1};
    static ImVec4 color     = ImVec4(1,1,0,1);
    static float  thickness = 5;
    
    static float history = 10.0f;
    ImGui::SliderFloat("History",&history,1,30,"%.1f s");

    //ImGui::DragScalarN("Row Ratios",ImGuiDataType_Float,rratios,rows,0.01f,0);
    //ImGui::DragScalarN("Col Ratios",ImGuiDataType_Float,cratios,cols,0.01f,0);
    //if (ImPlot::BeginSubplots("My Subplots", rows, cols, ImVec2(-1,300), flags, rratios, cratios)) 
    if (ImPlot::BeginSubplots("", rows, cols, ImVec2(600,500), flags, rratios, cratios)) 
    {
        //for (int i = 0; i < rows*cols; ++i) {
            //if (ImPlot::BeginPlot("",ImVec2(),ImPlotFlags_NoLegend)) 
            if (ImPlot::BeginPlot("##Digital")) 
            {
/*                 ImPlot::SetupAxes(NULL,NULL,ImPlotAxisFlags_NoDecorations,ImPlotAxisFlags_NoDecorations);
                float fi = 0.01f * (i+1);
                ImPlot::SetNextLineStyle(SampleColormap((float)i/(float)(rows*cols-1),ImPlotColormap_Jet));
                ImPlot::PlotLineG("data",SinewaveGetter,&fi,1000); */
                color = ImVec4(255,0,0,1); // R,G,B,Opaque
                ImPlot::SetNextLineStyle(color, thickness);
                ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -7, 7);
                ImPlot::SetupAxes("Time (sec)","Position (rad)");
                ImPlot::PlotLine("Position", &rdata1.Data[0].x, &rdata1.Data[0].y, rdata1.Data.size(), 0, rdata1.Offset, 2 * sizeof(float));
                ImPlot::EndPlot();
            }
            if (ImPlot::BeginPlot("##Digital")) 
            {
                color = ImVec4(255,0,0,1); // R,G,B,Opaque
                ImPlot::SetNextLineStyle(color, thickness);
                ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 24);
                ImPlot::SetupAxes("Time (sec)","VQ (V)");
                ImPlot::PlotLine("Voltage", &rdata5.Data[0].x, &rdata5.Data[0].y, rdata5.Data.size(), 0, rdata5.Offset, 2 * sizeof(float));
                ImPlot::EndPlot();
            } 
            if (ImPlot::BeginPlot("##Digital")) 
            {
                color = ImVec4(0,255,0,1); // R,G,B,Opaque
                ImPlot::SetNextLineStyle(color, thickness);
                ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -70, 70);
                ImPlot::SetupAxes("Time (sec)","Velocity (rad/s)");
                ImPlot::PlotLine("Velocity", &rdata2.Data[0].x, &rdata2.Data[0].y, rdata2.Data.size(), 0, rdata2.Offset, 2 * sizeof(float));
                ImPlot::EndPlot();
            } 
            if (ImPlot::BeginPlot("##Digital")) 
            {
                color = ImVec4(0,255,0,1); // R,G,B,Opaque
                ImPlot::SetNextLineStyle(color, thickness);
                ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -24, 24);
                ImPlot::SetupAxes("Time (sec)","IQ (A)");
                ImPlot::PlotLine("Current", &rdata2.Data[0].x, &rdata2.Data[0].y, rdata2.Data.size(), 0, rdata2.Offset, 2 * sizeof(float));
                ImPlot::EndPlot();
            } 
            if (ImPlot::BeginPlot("##Digital")) 
            {
                color = ImVec4(0,0,255,1); // R,G,B,Opaque
                ImPlot::SetNextLineStyle(color, thickness);
                ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -1.5, 1.5);
                ImPlot::SetupAxes("Time (sec)","Torque (Nm)");
                //ImPlot::SetupAxis(ImAxis_Y2, "Current (A)",ImPlotAxisFlags_AuxDefault);
                //mPlot::SetupAxisLimits(ImAxis_Y2, -3, 3);
                ImPlot::PlotLine("Torque", &rdata3.Data[0].x, &rdata3.Data[0].y, rdata3.Data.size(), 0, rdata3.Offset, 2 * sizeof(float));
                //ImPlot::PlotLine("Current", &rdata4.Data[0].x, &rdata4.Data[0].y, rdata3.Data.size(), 0, rdata3.Offset, 2 * sizeof(float));
                ImPlot::EndPlot();
            } 
            if (ImPlot::BeginPlot("##Digital")) 
            {
                color = ImVec4(0,0,255,1); // R,G,B,Opaque
                ImPlot::SetNextLineStyle(color, thickness);
                ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -24, 24);
                ImPlot::SetupAxes("Time (sec)","Power (W)");
                ImPlot::PlotLine("Power", &rdata2.Data[0].x, &rdata2.Data[0].y, rdata2.Data.size(), 0, rdata2.Offset, 2 * sizeof(float));
                ImPlot::EndPlot();
            } 
        //}
        ImPlot::EndSubplots();
    }

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
    lcm.subscribe("EXAMPLE", &Handler::handleMessage, &handlerObject); 
    
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
    //motor_data.position = 100;
    //lcm2.publish("MOTOR", &motor_data);

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(1400, 1000, "Motor Control", NULL, NULL); // 1280,720
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

    v_q << 0;
    v_dq << 0;
    v_u << 0;
    v_tau_cmd << 0;
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
    motor_updaterThread.join();
    
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}



