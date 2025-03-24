
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
    m_infoBar = std::make_shared<InfoBar>(m_RobotSerial, m_AverageFps);
    m_SerialMonitor = std::make_shared<SerialMonitor>(m_RobotSerial);
    m_ControlPanel = std::make_shared<BotControlWindow>();

    m_UIwindows.push_back(m_ConfigWindow);
    m_UIwindows.push_back(m_SerialMonitor);
    m_UIwindows.push_back(m_ControlPanel);
    m_UIwindows.push_back(m_infoBar);

    m_FpsBuffer = std::make_unique<Buffer<ImPlotPoint>>(FPS_BUFFER_SIZE);
    viewPort.GetCamera().setScale(DEFAULT_VIEWPORT_ZOOM);
}

Application::~Application() 
{

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
}   

void Application::Update()
{
    ImGuiIO& io = ImGui::GetIO();
    m_FpsBuffer->addData({(float)SDL_GetTicks() / 1000.0f, io.DeltaTime * 1000.0f});

    m_AverageFps = 0;
    for (int i = 0; i < m_FpsBuffer->size(); i++)
    {
        m_AverageFps += m_FpsBuffer->data()[i][1];
    }
    m_AverageFps /= m_FpsBuffer->size();
    m_AverageFps;

    EncoderDataPacket encoderData = m_SerialMonitor->EncPacket;
    m_Odom.update(encoderData.encA, encoderData.encB);

    Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * ViewPort::GetInstance().GetViewPortMousePos();
    Eigen::Vector2d wheelVels = wheelVelFromGoal(m_Odom.getState().x(), m_Odom.getState().y(), m_Odom.getState().z(), mousePosWorld.x(), mousePosWorld.y());

    m_RobotSerial.SetCommandVel((float)wheelVels.x(), (float)wheelVels.y());

    if (m_SerialMonitor->AncPacket.anchorID == 'A')
    {
        m_biLat.updateRange(m_SerialMonitor->AncPacket.range, m_biLat.rangeB);
    }

    else if (m_SerialMonitor->AncPacket.anchorID == 'B')
    {
        m_biLat.updateRange(m_biLat.rangeA, m_SerialMonitor->AncPacket.range);
    }

    m_KalmanFilter.setAnchors(m_biLat.getAnchorPos('A'), m_biLat.getAnchorPos('B'));
    m_KalmanFilter.predict({0, 0}, 0.1);
    m_KalmanFilter.update({m_biLat.getAnchorRange('A') , m_biLat.getAnchorRange('B')}, 0.1);

    GraphWindow();
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
        }
    }
    viewPort.ViewPortEnd();
}

void Application::GraphWindow()
{
    ImGui::Begin("Graphs");
    {
        if (ImGui::CollapsingHeader("Fps Graph##Header", ImGuiTreeNodeFlags_DefaultOpen) && ImPlot::BeginPlot("Fps##graph")) 
        {
            ImPlot::SetupAxes("Time (s)", "FPS", ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxisLimits(ImAxis_Y1, m_AverageFps - 10.0f, m_AverageFps + 10.0f, ImPlotCond_Always);
            ImPlot::PlotLine("##fpsline", &m_FpsBuffer->data()[0][0], &m_FpsBuffer->data()[0][1], (int)m_FpsBuffer->size(), 0, 0, sizeof(m_FpsBuffer->data()[0]));
            ImPlot::EndPlot();
        }
    }
    ImGui::End();
}



