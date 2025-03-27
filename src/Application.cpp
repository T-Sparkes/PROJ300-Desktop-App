
#include "Application.hpp"

Application& Application::GetInstance()
{
    static Application instance;
    return instance;
}

Application::Application() : 
    m_WorldGrid({0, 0}, {DEFAULT_GRID_SIZE, DEFAULT_GRID_SIZE}), 
    m_biLat(DEFAULT_ANCHOR_A_POS, DEFAULT_ANCHOR_B_POS), 
    m_KalmanFilter(KF_DEFAULT_POS, KF_DEFAULT_Q, KF_DEFAULT_R)
{
    m_ConfigWindow = std::make_shared<ConfigWindow>(m_WorldGrid, m_biLat, m_KalmanFilter);
    m_infoBar = std::make_shared<InfoBar>(m_RobotSerial, m_AvgFrameTime);
    m_SerialMonitor = std::make_shared<SerialMonitor>(m_RobotSerial);
    m_ControlPanel = std::make_shared<BotControlWindow>();

    m_UIwindows.push_back(m_ConfigWindow);
    m_UIwindows.push_back(m_SerialMonitor);
    m_UIwindows.push_back(m_ControlPanel);
    m_UIwindows.push_back(m_infoBar);

    viewPort.GetCamera().setScale(DEFAULT_VIEWPORT_ZOOM);
    m_FrameTBuffer = std::make_unique<Buffer<ImPlotPoint>>(FPS_BUFFER_SIZE);
    m_KalmanFilter.setAnchors(DEFAULT_ANCHOR_A_POS, DEFAULT_ANCHOR_B_POS);
}

void Application::OnEvent(SDL_Event* event) 
{
    if (ImGui::IsKeyDown(ImGuiKey_Enter) && ImGui::IsKeyDown(ImGuiKey_LeftAlt))
    {
        appState = RESTART; // Doesnt work anymore :(
    }

    if (ImGui::IsKeyDown(ImGuiKey_Escape))
    {
        appState = QUIT;
    }

    if (ImGui::IsKeyDown(ImGuiKey_F11))
    {
        static bool bFullscreen = false;
        bFullscreen = !bFullscreen;
        SDL_SetWindowFullscreen(RendererBackend::GetInstance().GetSdlWindow(), bFullscreen);
    }
}   

void Application::Update()
{
    // Get data from robot
    StatusPacket statusData;
    AnchorRangePacket rangeData;
    EncoderDataPacket encoderData;

    //m_RobotSerial.getPacket(&statusData);
    //m_RobotSerial.getPacket(&rangeData);
    //m_RobotSerial.getPacket(&encoderData);

    // Temp odom stuff
    //m_Odom.update(encoderData.encA, encoderData.encB);
    //Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * ViewPort::GetInstance().GetViewPortMousePos();
//
    //Eigen::Vector2d wheelVels = wheelVelFromGoal(
    //    m_Odom.getState().x(), 
    //    m_Odom.getState().y(), 
    //    m_Odom.getState().z(), 
    //    mousePosWorld.x(), 
    //    mousePosWorld.y()
    //); // Temp, do a better job of this
//
    //m_RobotSerial.SetCommandVel((float)wheelVels.x(), (float)wheelVels.y());

    // Update bilateration visualisation
    if (m_SerialMonitor->AncPacket.anchorID == 'A')
    {
        m_biLat.updateRange(m_SerialMonitor->AncPacket.range, m_biLat.rangeB);
    }

    else if (m_SerialMonitor->AncPacket.anchorID == 'B')
    {
        m_biLat.updateRange(m_biLat.rangeA, m_SerialMonitor->AncPacket.range);
    }

    // Update Kalman filter
    m_KalmanFilter.predict({0, 0}, 0.1);
    m_KalmanFilter.update({m_biLat.getAnchorRange('A') , m_biLat.getAnchorRange('B')}, 0.1);

    // Update UI
    GraphWindow();
    ViewPortWindow();

    for (auto& window : m_UIwindows)
    {
        window->OnUpdate();
    }

    // Self explanatory really
    CalcFrameTime();
}

void Application::ViewPortWindow()
{
    viewPort.ViewPortBegin();
    {
        Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * ViewPort::GetInstance().GetViewPortMousePos();

        if (ImGui::IsWindowHovered())
        {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) viewPort.GetCamera().setPanStart(mousePosWorld);
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) viewPort.GetCamera().panCamera(mousePosWorld);
            if (abs(ImGui::GetIO().MouseWheel) > 0) viewPort.GetCamera().updateZoom(mousePosWorld, ImGui::GetIO().MouseWheel);
        }
    }
    viewPort.ViewPortEnd();
}

void Application::GraphWindow()
{
    ImGui::Begin("Graphs");
    {
        if (ImGui::CollapsingHeader("Frametime Graph##Header", ImGuiTreeNodeFlags_DefaultOpen) && ImPlot::BeginPlot("Frametime##graph")) 
        {
            ImPlot::SetupAxes("Time (s)", "Frametime (ms)", ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxisLimits(ImAxis_Y1, m_AvgFrameTime - 10.0f, m_AvgFrameTime + 10.0f, ImPlotCond_Always);
            ImPlot::PlotLine("##fpsline", &m_FrameTBuffer->data()[0][0], &m_FrameTBuffer->data()[0][1], (int)m_FrameTBuffer->size(), 0, 0, sizeof(m_FrameTBuffer->data()[0]));
            ImPlot::EndPlot();
        }
    }
    ImGui::End();
}

void Application::CalcFrameTime()
{
    m_AvgFrameTime = 0;
    m_FrameTBuffer->addData({(float)SDL_GetTicks() / 1000.0f, ImGui::GetIO().DeltaTime * 1000.0f});
    
    for (int i = 0; i < m_FrameTBuffer->size(); i++)
    {
        m_AvgFrameTime += m_FrameTBuffer->data()[i][1];
    }
    m_AvgFrameTime /= m_FrameTBuffer->size();
}


