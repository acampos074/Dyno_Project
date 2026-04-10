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

#include <math.h>

// Include LCM
#include <lcm/lcm-cpp.hpp>
#include "Position.hpp"
#include "mypackage_Position.h"
#include <sys/select.h>

//#include "/home/campo074/vcpkg/packages/implot_x64-linux/include/implot.h"
#include "implot.h"
#include "implot_internal.h"

//#include "RedisClient.h"
//#include "USB2CAN.h"
#include <unistd.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
//#include "/home/campo074/Documents/ImGuiApps/MyApplication/implot-master/implot.h"
//#include "/home/campo074/Documents/ImGuiApps/MyApplication/implot-master/implot_internal.h"

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
static std::string gen_cmd;
//static std::string dq_cmd;
Eigen::VectorXd dq_cmd(1);

//mypackage::Position position;
double xdata = 0;


 class Handler {
  public:
    ~Handler() {}
    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const mypackage::Position *msg)
    {
        xdata = msg->x;
    }
}; 

/* static void my_handler(const lcm_recv_buf_t *rbuf, const char *channel, const mypackage_Position *msg,
                       void *user)
{
    xdata = msg->x;
} */


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
    static float f = 0.0f;
    static float kp_float = 0.05f;
    static float kd_float = 0.012f;

    // ===============

    ImGui::Begin("Motor Controller");
    //ImGui::BulletText("Move your mouse to change the data!");
    //ImGui::BulletText("This example assumes 60 FPS. Higher FPS requires larger buffer size.");
    static bool paused = false;
    //static ScrollingBuffer sdata1, sdata2;
    static ScrollingBuffer   rdata1, rdata2, rdata3;
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
    }


    //sdata1.AddPoint(t, mouse.x * 0.0005f);
    //rdata1.AddPoint(t, mouse.x * 0.0005f);
    //sdata2.AddPoint(t, mouse.y * 0.0005f);
    //rdata2.AddPoint(t, mouse.y * 0.0005f);

    

    //static float history = 10.0f;
    //ImGui::SliderFloat("History",&history,1,30,"%.1f s");
    //rdata1.Span = history;
    //rdata2.Span = history;

    //static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

    //if (ImPlot::BeginPlot("##Rolling", ImVec2(-1,150))) 
/*     if (ImPlot::BeginPlot("##Digital")) 
    {
        ImPlot::SetupAxisLimits(ImAxis_X1, t - 10.0, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -1, 7);
        ImPlot::SetupAxes("Time (sec)","Rotor Position (rad)");
        ImPlot::PlotLine("Position", &rdata1.Data[0].x, &rdata1.Data[0].y, rdata1.Data.size(), 0, rdata1.Offset, 2 * sizeof(float));
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("##Digital")) 
    {
        ImPlot::SetupAxisLimits(ImAxis_X1, t - 10.0, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -7, 7);
        ImPlot::SetupAxes("Time (sec)","Rotor Speed (rad/sec)");
        ImPlot::PlotLine("Speed", &rdata2.Data[0].x, &rdata2.Data[0].y, rdata2.Data.size(), 0, rdata2.Offset, 2 * sizeof(float));
        ImPlot::EndPlot();
    } */


    static ImPlotSubplotFlags flags = ImPlotSubplotFlags_None;
    //ImGui::CheckboxFlags("ImPlotSubplotFlags_NoResize", (unsigned int*)&flags, ImPlotSubplotFlags_NoResize);
    //ImGui::CheckboxFlags("ImPlotSubplotFlags_NoTitle", (unsigned int*)&flags, ImPlotSubplotFlags_NoTitle);

    // === Buttons ====
    // -------- Controller On/Off button --------
    if (ImGui::Button("Position Controller ON"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
        clicked++;
    
    if(clicked & 1)
    {
        ImGui::SameLine();
        ImGui::Text("State = ON");
        gen_cmd = "4";
        clicked = 0;
    }
    if (ImGui::Button("Velocity Controller ON"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
        vel_ctl_clicked++;
    
    if(vel_ctl_clicked & 1)
    {
        ImGui::SameLine();
        ImGui::Text("State = ON");
        gen_cmd = "5";
        vel_ctl_clicked = 0;
    }
    if (ImGui::Button("Controller OFF"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
        off_clicked++;
    if(off_clicked & 1)
    {
        ImGui::SameLine();
        ImGui::Text("State = OFF");
        gen_cmd = "2";
        off_clicked = 0;
    }
    // -------- Pause button --------
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
    }
    else
    {
        ImGui::SameLine();
        ImGui::Text("Toggle OFF");
        led_flag = "0";
    }
    // -------- Calibrate --------
    if (ImGui::Button("Calibrate"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
        calibrate_clicked++;
    
    if(calibrate_clicked & 1 )
    {
        ImGui::SameLine();
        //ImGui::Text("Toggle ON");
        gen_cmd = "1";
        calibrate_clicked = 0;
    }
    // -------- Voltage FOC --------
    if (ImGui::Button("Voltage FOC"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
    voltage_foc_clicked++;
    
    if(voltage_foc_clicked & 1 )
    {
        ImGui::SameLine();
        //ImGui::Text("Toggle ON");
        gen_cmd = "3";
        voltage_foc_clicked = 0;
    }

    
    
    // ================
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
    gen_cmd = "0";

    static int rows = 3;
    static int cols = 1;
    //ImGui::SliderInt("Rows",&rows,1,5);
    //ImGui::SliderInt("Cols",&cols,1,5);
    static float rratios[] = {1,1,1,1,1,1};
    static float cratios[] = {1,1,1,1,1,1};
    static ImVec4 color     = ImVec4(1,1,0,1);
    static float  thickness = 5;
    
    
    //ImGui::DragScalarN("Row Ratios",ImGuiDataType_Float,rratios,rows,0.01f,0);
    //ImGui::DragScalarN("Col Ratios",ImGuiDataType_Float,cratios,cols,0.01f,0);
    //if (ImPlot::BeginSubplots("My Subplots", rows, cols, ImVec2(-1,300), flags, rratios, cratios)) 
    if (ImPlot::BeginSubplots("My Subplots", rows, cols, ImVec2(600,700), flags, rratios, cratios)) 
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
                ImPlot::SetupAxisLimits(ImAxis_X1, t - 10.0, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -7, 7);
                ImPlot::SetupAxes("Time (sec)","Position (rad)");
                ImPlot::PlotLine("Position", &rdata1.Data[0].x, &rdata1.Data[0].y, rdata1.Data.size(), 0, rdata1.Offset, 2 * sizeof(float));
                ImPlot::EndPlot();
            }
            if (ImPlot::BeginPlot("##Digital")) 
            {
                color = ImVec4(0,255,0,1); // R,G,B,Opaque
                ImPlot::SetNextLineStyle(color, thickness);
                ImPlot::SetupAxisLimits(ImAxis_X1, t - 10.0, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -70, 70);
                ImPlot::SetupAxes("Time (sec)","Speed (rad/s)");
                ImPlot::PlotLine("Speed", &rdata2.Data[0].x, &rdata2.Data[0].y, rdata2.Data.size(), 0, rdata2.Offset, 2 * sizeof(float));
                ImPlot::EndPlot();
            } 
            if (ImPlot::BeginPlot("##Digital")) 
            {
                color = ImVec4(0,0,255,1); // R,G,B,Opaque
                ImPlot::SetNextLineStyle(color, thickness);
                ImPlot::SetupAxisLimits(ImAxis_X1, t - 10.0, t, paused ? ImGuiCond_Once : ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -1.5, 1.5);
                ImPlot::SetupAxes("Time (sec)","Torque (Amps)");
                ImPlot::PlotLine("Torque", &rdata3.Data[0].x, &rdata3.Data[0].y, rdata3.Data.size(), 0, rdata3.Offset, 2 * sizeof(float));
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
    std::thread updaterThread([&]() {
        while(0==lcm.handle())
        {
            ;
        } // query the latest message
    }); 

/*
    lcm_t *lcm;

    lcm = lcm_create(NULL);
    if (!lcm)
        return 1;

    mypackage_Position_subscription_t *sub =
        mypackage_Position_subscribe(lcm, "EXAMPLE", &my_handler, NULL);


    std::thread updaterThread([&]() {
        while(1)
        {
            // setup the LCM file descriptor for waiting.
            int lcm_fd = lcm_get_fileno(lcm);
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(lcm_fd, &fds);

            // wait a limited amount of time for an incoming message
            struct timeval timeout = {
                1,  // seconds
                0   // microseconds
            };
            int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);

            if (0 == status) {
                // no messages
                printf("waiting for message\n");
            } else if (FD_ISSET(lcm_fd, &fds)) {
                // LCM has events ready to be processed.
                lcm_handle(lcm);
            }
        } // query the latest message
    }); */

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

    // Make sure redis-server is running at localhost with default port 6379
	// start redis client
    /*
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	//auto redis_client = RedisClient();
    //redis_client = RedisClient();
	redis_client.serverIs(info);
    */    

    // Do other tasks in the main thread
    
    // Use the Position message
    
    //position.x = 10.0;
    //position.y = 5.0;

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
            
        /*
        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        {
            static float f = 0.0f;
            static int counter = 0;

            ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

            ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
            ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
            ImGui::Checkbox("Another Window", &show_another_window);

            ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

            if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
                counter++;
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            
            // Generate samples and plot them (sine wave)
            float samples[100];
            float cos_samples[100];
            for (int n = 0; n < 100; n++)
            {
                samples[n] = sinf(n * 0.2f + ImGui::GetTime() * 1.5f);
                cos_samples[n] = cosf(n*0.2f+ImGui::GetTime()*1.5f);
            }
            ImGui::PlotLines("Sine", samples, 100);
            ImGui::PlotLines("Cosine", cos_samples, 100);
            
            ImGui::End();
        }
        */
        /*
        // 3. Show another simple window.
        if (show_another_window)
        {
            ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
            ImGui::Text("Hello from another window!");
            if (ImGui::Button("Close Me"))
                show_another_window = false;
            ImGui::End();
        }
        */
        /*
        // 4. Implot Demo
        {
            v_u[0] = 0;
            redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY,v_q);
            redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY,v_dq); 
            redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_KEY,v_u);
            ctl_flag = "1";
            redis_client.setCommandIs(CONTROL_FLAG_KEY,ctl_flag);
            //int   bar_data[11];
            float x_data[100];
            //float y_data[100];
            //x_data[n] = v_q[0]; // v_dq[0]
            
            for (int n = 0; n < 100; n++)
            {
                x_data[n] = sinf(n * 0.2f + ImGui::GetTime() * 1.5f);
                //x_data[n] = v_q[0]; // v_dq[0]
                //cos_samples[n] = cosf(n*0.2f+ImGui::GetTime()*1.5f);
            }
            
            ImGui::Begin("My Window");
            if (ImPlot::BeginPlot("Joint Speed")) {
                //ImPlot::PlotBars("My Bar Plot", bar_data, 11);
                //ImPlot::PlotLine("My Line Plot", x_data, y_data, 1000);
                ImPlot::PlotLine("Joint Speed", x_data, 100);
                ImPlot::EndPlot();
            }
            ImGui::End();
        }
        */

        
        
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

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}



