#include "Application.hpp"

// Constructor: Initializes the application, UI windows, and default settings
Application::Application() : 
    m_WorldGrid({0, 0}, {DEFAULT_GRID_SIZE, DEFAULT_GRID_SIZE}), 
    m_Landmarks(DEFAULT_LANDMARK_A_POS, DEFAULT_LANDMARK_B_POS), 
    m_KalmanFilter(KF_DEFAULT_POS, KF_DEFAULT_Q, KF_DEFAULT_R),
    m_FrameTBuffer(FPS_BUFFER_SIZE)
{
    // Initialize UI windows
    m_infoBar = std::make_shared<InfoBar>(m_RobotSerial, m_AvgFrameTime);
    m_SerialMonitor = std::make_shared<SerialMonitor>(m_RobotSerial);
    m_ControlPanel = std::make_shared<BotControlWindow>(m_RobotSerial);
    m_GraphWindow = std::make_shared<GraphWindow>(m_FrameTBuffer, m_AvgFrameTime);
    m_ConfigWindow = std::make_shared<ConfigWindow>(m_WorldGrid, m_Landmarks, m_KalmanFilter, m_PathController);

    // Add UI windows to the rendering order
    m_UIwindows.push_back(m_ConfigWindow);
    m_UIwindows.push_back(m_SerialMonitor);
    m_UIwindows.push_back(m_ControlPanel);
    m_UIwindows.push_back(m_infoBar);
    m_UIwindows.push_back(m_GraphWindow);

    // Set default viewport zoom and Kalman filter anchors
    m_ViewPort.GetCamera().setScale(DEFAULT_VIEWPORT_ZOOM);
    m_KalmanFilter.setAnchors(DEFAULT_LANDMARK_A_POS, DEFAULT_LANDMARK_B_POS);

    SDL_LogVerbose(SDL_LOG_CATEGORY_APPLICATION, "APP INFO: Application initialized\n");
}

// Handles SDL events and processes custom user events
void Application::OnEvent(SDL_Event* event) 
{
    // Handle serial status event
    if (event->type == SDL_EVENT_USER && event->user.code == SERIAL_STATUS_EVENT)
    {
        StatusPacket statusData = *reinterpret_cast<StatusPacket*>(event->user.data1);
        delete event->user.data1; // Free the memory allocated for the packet

        m_SerialMonitor->OnNewStatusPacket(&statusData); 
    }
    // Handle serial landmark event
    else if (event->type == SDL_EVENT_USER && event->user.code == SERIAL_LANDMARK_EVENT)
    {
        LandmarkPacket landmarkData = *reinterpret_cast<LandmarkPacket*>(event->user.data1);
        delete event->user.data1; // Free the memory allocated for the packet

        // Landmark Container processes the landmark data
        m_Landmarks.OnNewPacket(&landmarkData);
        m_SerialMonitor->OnNewLandmarkPacket(&landmarkData);
    
        // Update the Kalman filter with the corrected landmark data
        m_KalmanFilter.updateLandmark(
            landmarkData.LandmarkID, 
            m_Landmarks.getLandmarkPos(landmarkData.LandmarkID), 
            m_Landmarks.getLandmarkRange(landmarkData.LandmarkID)
        );
    }
    // Handle serial encoder event
    else if (event->type == SDL_EVENT_USER && event->user.code == SERIAL_ENCODER_EVENT)
    {
        EncoderDataPacket encoderData = *reinterpret_cast<EncoderDataPacket*>(event->user.data1);
        delete event->user.data1; // Free the memory allocated for the packet

        m_SerialMonitor->OnNewEncoderPacket(&encoderData);
        m_KalmanFilter.predict({encoderData.encA, encoderData.encB}, ImGui::GetIO().DeltaTime);
    }
}

// Main update loop for the application
void Application::Update()
{
    // Handle application state changes
    if (ImGui::IsKeyPressed(ImGuiKey_Escape))
    {
        appState = RESTART;
    }

    // Toggle fullscreen mode
    if (ImGui::IsKeyPressed(ImGuiKey_F11, false))
    {
        static bool bFullscreen = false;
        bFullscreen = !bFullscreen;
        SDL_SetWindowFullscreen(RendererBackend::GetInstance().GetSdlWindow(), bFullscreen);
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "APP INFO: Fullscreen: %s\n", bFullscreen ? "Enabled" : "Disabled");
    }

    // Toggle VSync
    if (ImGui::IsKeyPressed(ImGuiKey_F10, false))
    {
        static bool bVsync = true;
        bVsync = !bVsync;
        SDL_SetRenderVSync(RendererBackend::GetInstance().GetSdlRenderer(), bVsync);
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "APP INFO: VSync: %s\n", bVsync ? "Enabled" : "Disabled");
    }

    // Handle viewport input (camera and interaction)
    m_HandleViewportInput();

    // Update robot control logic
    Eigen::Vector2d mousePosWorld = m_ViewPort.GetCamera().transform.inverse() * m_ViewPort.GetViewPortMousePos();
    static Eigen::Vector2d currentGoal = {0, -0.5}; 
    static bool bStopped = false;

    // Check if the robot has reached the current goal
    if ((m_KalmanFilter.x.head(2) - currentGoal).norm() < 0.05)
    {
        currentGoal = m_PathController.getNextWaypoint();    
    }

    // Toggle robot movement
    if (ImGui::IsKeyPressed(ImGuiKey_Space, false))
    {
        bStopped = !bStopped;
    }

    // Update robot serial commands
    static Uint64 lastControl = SDL_GetTicks();
    if (((SDL_GetTicks() - lastControl) > 1000 / CONTROL_FREQ_HZ) && !bStopped)
    {
        if (m_ControlPanel->controlMode == WAYPOINT)
        {
            Eigen::Vector2d wheelVels = m_PathController.wheelVelFromGoal(m_KalmanFilter.x, currentGoal);
            m_RobotSerial.SetCommandVel(static_cast<float>(wheelVels[0]), static_cast<float>(wheelVels[1]));
        }
        lastControl = SDL_GetTicks();
    }
    
    else if (bStopped)
    {
        m_RobotSerial.SetCommandVel(0, 0);
    }

    // Update graphs with Kalman filter data
    static Uint64 lastGraphSample = SDL_GetTicks();
    if ((SDL_GetTicks() - lastGraphSample) > 1000 / GRAPH_FREQ_HZ)
    {
        double kRangeA = (m_KalmanFilter.x.head(2) - m_Landmarks.getLandmarkPos('A')).norm();
        double kRangeB = (m_KalmanFilter.x.head(2) - m_Landmarks.getLandmarkPos('B')).norm();

        m_GraphWindow->addRangeData({m_Landmarks.getLandmarkRange('A'), m_Landmarks.getLandmarkRange('B')}, {kRangeA, kRangeB});
        m_GraphWindow->addKalmanData(m_KalmanFilter.K, m_KalmanFilter.P);

        m_CalcFrameTime();
        lastGraphSample = SDL_GetTicks();
    }

    // Update all UI windows
    for (auto& window : m_UIwindows)
    {
        window->OnUpdate();
    }
}

// Handles viewport input for camera controls and waypoint editing
void Application::m_HandleViewportInput()
{
    m_ViewPort.ViewPortBegin();
    {
        Eigen::Vector2d mousePosWorld = m_ViewPort.GetCamera().transform.inverse() * m_ViewPort.GetViewPortMousePos();

        if (ImGui::IsWindowHovered())
        {
            // Camera controls
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) 
            {
                m_ViewPort.GetCamera().setPanStart(mousePosWorld);
            }

            else if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) 
            {
                m_ViewPort.GetCamera().panCamera(mousePosWorld);
            }

            if (abs(ImGui::GetIO().MouseWheel) > 0) 
            {
                m_ViewPort.GetCamera().updateZoom(mousePosWorld, ImGui::GetIO().MouseWheel);
            }

            // Kalman filter controls
            if (ImGui::IsKeyDown(ImGuiKey_LeftShift) && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
            {
                m_KalmanFilter.setPoseEstimate({mousePosWorld.x(), mousePosWorld.y(), 0});
                m_KalmanFilter.P = Eigen::Matrix3d::Identity();
            }
            
            // Waypoint editing 
            if (ImGui::IsKeyDown(ImGuiKey_LeftShift) && ImGui::IsMouseClicked(ImGuiMouseButton_Left, true))
            {
                m_PathController.addWaypoint(mousePosWorld);
            }

            else if (ImGui::IsKeyDown(ImGuiKey_LeftCtrl) & ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            {
                m_PathController.removeWaypointNear(mousePosWorld);
            }

            // Move nearest waypoint to mouse position
            else if (ImGui::IsKeyDown(ImGuiKey_LeftAlt) && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
            {
                for (int i = 0; i < m_PathController.waypoints.size(); i++)
                {
                    if (m_PathController.isMouseOverWaypoint(m_PathController.waypoints[i]))
                    {
                        m_PathController.moveWaypoint(i, mousePosWorld);
                        break;
                    }
                }
            }
        }
    }
    m_ViewPort.ViewPortEnd();
}

// Calculates the average frame time for performance monitoring
void Application::m_CalcFrameTime()
{
    m_AvgFrameTime = 0;
    m_FrameTBuffer.addData({(float)SDL_GetTicks() / 1000.0f, ImGui::GetIO().DeltaTime * 1000.0f});
    
    for (int i = 0; i < m_FrameTBuffer.size(); i++)
    {
        m_AvgFrameTime += m_FrameTBuffer.data()[i][1];
    }
    m_AvgFrameTime /= m_FrameTBuffer.size();
}


