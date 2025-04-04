
#include "Application.hpp"

Application& Application::GetInstance()
{
    static Application instance;
    return instance;
}

Application::Application() : 
    m_WorldGrid({0, 0}, {DEFAULT_GRID_SIZE, DEFAULT_GRID_SIZE}), 
    m_Landmarks(DEFAULT_LANDMARK_A_POS, DEFAULT_LANDMARK_B_POS), 
    m_KalmanFilter(KF_DEFAULT_POS, KF_DEFAULT_Q, KF_DEFAULT_R),
    m_FrameTBuffer(FPS_BUFFER_SIZE)
{
    m_infoBar = std::make_shared<InfoBar>(m_RobotSerial, m_AvgFrameTime);
    m_SerialMonitor = std::make_shared<SerialMonitor>(m_RobotSerial);
    m_ControlPanel = std::make_shared<BotControlWindow>(m_RobotSerial);
    m_GraphWindow = std::make_shared<GraphWindow>(m_FrameTBuffer, m_AvgFrameTime);
    m_ConfigWindow = std::make_shared<ConfigWindow>(m_WorldGrid, m_Landmarks, m_KalmanFilter);

    m_UIwindows.push_back(m_ConfigWindow);
    m_UIwindows.push_back(m_SerialMonitor);
    m_UIwindows.push_back(m_ControlPanel);
    m_UIwindows.push_back(m_infoBar);
    m_UIwindows.push_back(m_GraphWindow);

    viewPort.GetCamera().setScale(DEFAULT_VIEWPORT_ZOOM);
    m_KalmanFilter.setAnchors(DEFAULT_LANDMARK_A_POS, DEFAULT_LANDMARK_B_POS);
}

void Application::OnEvent(SDL_Event* event) 
{
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
    Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * ViewPort::GetInstance().GetViewPortMousePos();
    static Eigen::Vector2d goal1 = {-0.65, -0.75};
    static Eigen::Vector2d goal2 = {-0.5, -2};
    static Eigen::Vector2d goal3 = {0.65, -0.75};
    static Eigen::Vector2d goal4 = {1.5, -0.75};
    static Eigen::Vector2d currentGoal = goal1; 

    if ((m_KalmanFilter.x.head(2) - currentGoal).norm() < 0.1)
    {
        if (currentGoal == goal1) currentGoal = goal2;
        else if (currentGoal == goal2) currentGoal = goal3;
        else if (currentGoal == goal3) currentGoal = goal4;
        else if (currentGoal == goal4) currentGoal = goal1;              
    }

    // Update bilateration visualisation
    if ((m_SerialMonitor->AncPacket.anchorID == 'A') && (m_SerialMonitor->AncPacket.range) < 10.0f && (m_SerialMonitor->AncPacket.range) > 0.0f)
    {   
        // Calculate horizontal range if anchor is 75cm above the ground
        float correctedRange = sqrtf(powf(m_SerialMonitor->AncPacket.range * LANDMARK_A_CALIBRATION, 2) - powf(0.75f, 2));
        if (correctedRange > 0 && correctedRange < 10)
        {
            m_Landmarks.updateRange(correctedRange, m_Landmarks.rangeB);
            
        }
    }

    else if ((m_SerialMonitor->AncPacket.anchorID == 'B'))
    {   
        // Calculate horizontal range if anchor is 75cm above the ground
        float correctedRange = sqrtf(powf(m_SerialMonitor->AncPacket.range * LANDMARK_B_CALIBRATION, 2) - powf(0.75f, 2));
        if (correctedRange > 0 && correctedRange < 10)
        {
            m_Landmarks.updateRange(m_Landmarks.rangeA, correctedRange);
        }
    }

    // Update Kalman filter
    m_KalmanFilter.setAnchors(m_Landmarks.getLandmarkPos('A'), m_Landmarks.getLandmarkPos('B'));
    m_KalmanFilter.predict({m_SerialMonitor->EncPacket.encA, m_SerialMonitor->EncPacket.encB}, ImGui::GetIO().DeltaTime);

    static double lastRangeA = 0;    
    static double lastRangeB = 0;

    if ((m_Landmarks.getLandmarkRange('A') != lastRangeA) && (m_Landmarks.getLandmarkRange('B') != lastRangeB))
    {
        m_KalmanFilter.update({m_Landmarks.getLandmarkRange('A') , m_Landmarks.getLandmarkRange('B')}, ImGui::GetIO().DeltaTime);
        lastRangeA = m_Landmarks.getLandmarkRange('A');    
        lastRangeB = m_Landmarks.getLandmarkRange('B');    
    }

    Eigen::Vector2d wheelVels = wheelVelFromGoal(m_KalmanFilter.x.x(), m_KalmanFilter.x.y(), m_KalmanFilter.x.z(), currentGoal.x(), currentGoal.y());

    if (m_ControlPanel->controlMode == WAYPOINT)
    {
        m_RobotSerial.SetCommandVel(static_cast<float>(wheelVels[0]), static_cast<float>(wheelVels[1]));
    }

    static Uint64 lastGraphSample = SDL_GetTicks();
    if ((SDL_GetTicks() - lastGraphSample) > 20)
    {
        double kRangeA = (m_KalmanFilter.x.head(2) - m_Landmarks.getLandmarkPos('A')).norm();
        double kRangeB = (m_KalmanFilter.x.head(2) - m_Landmarks.getLandmarkPos('B')).norm();

        m_GraphWindow->addRangeData({m_Landmarks.getLandmarkRange('A'), m_Landmarks.getLandmarkRange('B')}, {kRangeA, kRangeB});
        m_GraphWindow->addKalmanData(m_KalmanFilter.K, m_KalmanFilter.P);

        CalcFrameTime();
        lastGraphSample = SDL_GetTicks();
    }

    static Uint64 lastTrailDraw = SDL_GetTicks();
    static std::vector<Eigen::Vector2d> posBuffer;

    // Update UI
    ViewPortWindow();
    
    for (auto& window : m_UIwindows)
    {
        window->OnUpdate();
    }
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

            if (ImGui::IsKeyDown(ImGuiKey_LeftShift) && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            {
                m_KalmanFilter.setPoseEstimate({mousePosWorld.x(), mousePosWorld.y(), 0});
            }
            
        }
    }
    viewPort.ViewPortEnd();
}

void Application::CalcFrameTime()
{
    m_AvgFrameTime = 0;
    m_FrameTBuffer.addData({(float)SDL_GetTicks() / 1000.0f, ImGui::GetIO().DeltaTime * 1000.0f});
    
    for (int i = 0; i < m_FrameTBuffer.size(); i++)
    {
        m_AvgFrameTime += m_FrameTBuffer.data()[i][1];
    }
    m_AvgFrameTime /= m_FrameTBuffer.size();
}


