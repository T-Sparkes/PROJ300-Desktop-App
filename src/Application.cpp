
#include "Application.hpp"
#include <Eigen/Dense>
#include <algorithm>

Eigen::Vector2d wheelVelFromGoal(double x, double y, double theta, double targetX, double targetY); 

Application::Application() : m_WorldGrid({0, 0}, {10, 10}), m_FpsBuffer(FPS_BUFFER_SIZE)
{
    m_SerialMonitor = new SerialMonitor(&m_SerialComm);
}

Application::~Application() 
{
    delete m_SerialMonitor;
}

void Application::OnEvent(SDL_Event* event) 
{
    if (ImGui::IsKeyDown(ImGuiKey_Enter) && ImGui::IsKeyDown(ImGuiKey_LeftAlt))
    {
        appState = RESTART;
    }

    if (ImGui::IsKeyDown(ImGuiKey_Escape))
    {
        appState = QUIT;
    }
}   

void Application::Update()
{
    ImGuiIO& io = ImGui::GetIO();
    m_FpsBuffer.addData({(double)SDL_GetTicks() / 1000.0, io.Framerate});

    EncoderDataPacket encoderData = m_SerialMonitor->EncPacket;
    odom.update(encoderData.encA, encoderData.encB);

    Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * GetViewPortMousePos();
    Eigen::Vector2d wheelVels = wheelVelFromGoal(odom.getState().x(), odom.getState().y(), odom.getState().z(), mousePosWorld.x(), mousePosWorld.y());

    m_SerialComm.SetCommandVel((float)wheelVels.x(), (float)wheelVels.y());

    fpsWindow();
    GraphWindow();
    ConfigWindow();
    ViewPortWindow();
    m_SerialMonitor->OnNewFrame();
    MotorTestWindow();
}

void Application::MotorTestWindow()
{
    ImGui::Begin("Motor DEBUG");

    static float motorSpeedA = 0.0f;
    static float motorSpeedB = 0.0f;

    ImGui::SliderFloat("Motor Speed A", &motorSpeedA, 0.0f, 7.0f);
    ImGui::SliderFloat("Motor Speed B", &motorSpeedB, 0.0f, 7.0f);

    //m_SerialComm.SetCommandVel(motorSpeedA, motorSpeedB);
    ImGui::End();
}

void Application::ViewPortWindow()
{
    viewPort.ViewPortBegin();
    {
        Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * GetViewPortMousePos();

        if (ImGui::IsWindowHovered())
        {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) viewPort.GetCamera().setPanStart(mousePosWorld);
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) viewPort.GetCamera().panCamera(mousePosWorld);
            if (abs(ImGui::GetIO().MouseWheel) > 0) viewPort.GetCamera().updateZoom(mousePosWorld, ImGui::GetIO().MouseWheel);
        }
    }
    viewPort.ViewPortEnd();
}

inline void Application::GraphWindow()
{
    ImGui::Begin("Graphs");
    {
        if (ImGui::CollapsingHeader("Fps Graph##Header", ImGuiTreeNodeFlags_DefaultOpen) && ImPlot::BeginPlot("Fps##graph")) 
        {
            ImPlot::SetupAxes("Time (s)", "FPS", ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxisLimits(ImAxis_Y1, m_AverageFps - 10.0f, m_AverageFps + 10.0f, ImPlotCond_Always);
            ImPlot::PlotLine("##fpsline", &m_FpsBuffer.data()[0][0], &m_FpsBuffer.data()[0][1], (int)m_FpsBuffer.size(), 0, 0, sizeof(m_FpsBuffer.data()[0]));
            ImPlot::EndPlot();
        }
    }
    ImGui::End();
}

void Application::fpsWindow()
{
    ImGuiIO& io = ImGui::GetIO();
    ViewPort& viewPort = ViewPort::GetInstance();
    Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * GetViewPortMousePos();

    m_AverageFps = 0;
    for (int i = 0; i < m_FpsBuffer.size(); i++)
    {
        m_AverageFps += m_FpsBuffer.data()[i][1];
    }
    m_AverageFps /= m_FpsBuffer.size();

    ImGui::Begin("##FPS");
    { 
        ImGui::Text("FPS: %d ", (int)m_AverageFps);

        ImGui::SameLine(); 
        ImGui::Text("| Camera Pos: %f, %f ", viewPort.GetCamera().getPosition().x(), viewPort.GetCamera().getPosition().y());

        ImGui::SameLine();
        ImGui::Text("| Camera Scale: %d", viewPort.GetCamera().getScale());

        ImGui::SameLine();
        ImGui::Text("| Mouse World Pos: %f, %f ", mousePosWorld.x(), mousePosWorld.y());

        ImGui::SameLine();
        ImGui::Text("| Robot Connetion Status: ");

        ImGui::SameLine();
        static StatusPacket status;
        m_SerialComm.getPacket(&status);

        if (status.connected)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0, 255, 0, 255));
            ImGui::Text("Connected");
            ImGui::PopStyleColor();
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(255, 0, 0, 255));
            ImGui::Text("No Connection");
            ImGui::PopStyleColor();
        }        
    }
    ImGui::End();
}

void Application::ConfigWindow()
{
    ViewPort& viewPort = ViewPort::GetInstance();
    Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * GetViewPortMousePos();

    ImGui::Begin("Config");
    {
        if (ImGui::CollapsingHeader("World Grid", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Checkbox("Enable Grid", &m_WorldGrid.bRender);
            int gridSizeInt[2] = {static_cast<int>(m_WorldGrid.gridSize.x()), static_cast<int>(m_WorldGrid.gridSize.y())};
            ImGui::InputInt2("Grid Size", gridSizeInt);
            m_WorldGrid.gridSize = Eigen::Vector2d(gridSizeInt[0], gridSizeInt[1]);
            ImGui::InputFloat("Cell Size", &m_WorldGrid.gridStep, 0.1f, 1.0f);
        }

        if (ImGui::CollapsingHeader("Anchor Options", ImGuiTreeNodeFlags_DefaultOpen))
        {
            // Todo
        }

        if (ImGui::CollapsingHeader("Kalman Filter", ImGuiTreeNodeFlags_DefaultOpen))
        {  
            // Todo
        }
    }
    ImGui::End();
}

Eigen::Vector2d wheelVelFromGoal(double x, double y, double theta, double targetX, double targetY) 
{

    const double width = 0.173; 

    double targetTheta = atan2(targetY - y, targetX - x);
    double error = targetTheta - theta;

    while (error > M_PI) error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;

    double omega = 1.0 * error;

    double vForwards = 0.05;  
    double vL = vForwards - (width / 2.0) * omega;
    double vR = vForwards + (width / 2.0) * omega;

    double omegaL = vL / 0.03;
    double omegaR = vR / 0.03;

    return {omegaL, omegaR}; 
}