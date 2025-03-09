
#include "Application.hpp"

Application::Application() : worldGrid({0, 0}, {10, 10}), fpsBuffer(FPS_BUFFER_SIZE)
{

}

Application::~Application() 
{

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
    fpsBuffer.addData({(double)SDL_GetTicks() / 1000.0, io.Framerate});

    fpsWindow();
    GraphWindow();
    ConfigWindow();
    ViewPortWindow();
    SerialMonitor();
    MotorTestWindow();
}

void Application::MotorTestWindow()
{
    ImGui::Begin("Motor DEBUG");
    static float motorSpeed = 0.0f;
    ImGui::SliderFloat("Motor Speed", &motorSpeed, 0.0f, 7.0f);
    serial.SetCommandVel(motorSpeed, motorSpeed);
    ImGui::End();
}

void Application::ViewPortWindow()
{
    viewPort.ViewPortBegin();

    Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * GetViewPortMousePos();

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) viewPort.GetCamera().setPanStart(mousePosWorld);
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) viewPort.GetCamera().panCamera(mousePosWorld);
    if (abs(ImGui::GetIO().MouseWheel) > 0) viewPort.GetCamera().updateZoom(mousePosWorld, ImGui::GetIO().MouseWheel);

    viewPort.ViewPortEnd();
}

inline void Application::GraphWindow()
{
    ImGui::Begin("Graphs");

    if (ImGui::CollapsingHeader("Fps Graph##Header", ImGuiTreeNodeFlags_DefaultOpen) && ImPlot::BeginPlot("Fps##graph")) 
    {
        ImPlot::SetupAxes("Time (s)", "FPS", ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisLimits(ImAxis_Y1, averageFps - 10.0f, averageFps + 10.0f, ImPlotCond_Always);
        ImPlot::PlotLine("##fpsline", &fpsBuffer.data()[0][0], &fpsBuffer.data()[0][1], (int)fpsBuffer.size(), 0, 0, sizeof(fpsBuffer.data()[0]));
        ImPlot::EndPlot();
    }

    ImGui::End();
}

void Application::fpsWindow()
{
    ImGuiIO& io = ImGui::GetIO();
    ViewPort& viewPort = ViewPort::GetInstance();
    Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * GetViewPortMousePos();

    averageFps = 0;
    for (int i = 0; i < fpsBuffer.size(); i++)
    {
        averageFps += fpsBuffer.data()[i][1];
    }
    averageFps /= fpsBuffer.size();

    ImGui::Begin("##FPS"); 
    ImGui::Text("FPS: %d ", (int)averageFps);
    ImGui::SameLine(); 
    ImGui::Text("| Camera Pos: %f, %f ", viewPort.GetCamera().getPosition().x(), viewPort.GetCamera().getPosition().y());
    ImGui::SameLine();
    ImGui::Text("| Camera Scale: %d", viewPort.GetCamera().getScale());
    ImGui::SameLine();
    ImGui::Text("| Mouse World Pos: %f, %f ", mousePosWorld.x(), mousePosWorld.y());
    ImGui::End();
}

void Application::ConfigWindow()
{
    ViewPort& viewPort = ViewPort::GetInstance();
    Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * GetViewPortMousePos();

    ImGui::Begin("Config");
    if (ImGui::CollapsingHeader("World Grid", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Enable Grid", &worldGrid.bRender);
        int gridSizeInt[2] = {static_cast<int>(worldGrid.gridSize.x()), static_cast<int>(worldGrid.gridSize.y())};
        ImGui::InputInt2("Grid Size", gridSizeInt);
        worldGrid.gridSize = Eigen::Vector2d(gridSizeInt[0], gridSizeInt[1]);
        ImGui::InputFloat("Cell Size", &worldGrid.gridStep, 0.1f, 1.0f);
    }

    if (ImGui::CollapsingHeader("Anchor Options", ImGuiTreeNodeFlags_DefaultOpen))
    {

    }

    if (ImGui::CollapsingHeader("Kalman Filter", ImGuiTreeNodeFlags_DefaultOpen))
    {  

    }

    ImGui::End();
}

void Application::SerialMonitor()
{
    static EncoderDataPacket rxPacket;
    static char portBuff[8] = DEFAULT_PORT;
    static int baudInput = DEFAULT_BAUDRATE;

    static std::vector<std::string> historyBuffer;

    ImGui::Begin("Serial Console");

    ImGui::InputText("Port", portBuff, sizeof(portBuff));
    ImGui::InputInt("BaudRate", &baudInput, 0, 0);

    if (ImGui::Button("Connect")) serial.OpenPort(portBuff, baudInput);
    ImGui::SameLine();
    if (ImGui::Button("Disconnect")) serial.ClosePort();

    if (serial.getPacket(&rxPacket))
    {
        static char lineBuffer[SERIAL_LINE_SIZE_BYTES];
        sprintf_s(lineBuffer, sizeof(lineBuffer), "PACKET: 0x%02X | %f | %f | %f | %f |\n", rxPacket.header, rxPacket.encA, rxPacket.encB, rxPacket.velA, rxPacket.velB);
        historyBuffer.push_back(lineBuffer);
    }

    ImGui::BeginChild("Console");
    {
        if (historyBuffer.size() > SERIAL_HISTORY_SIZE_LINES)
        {
            historyBuffer.erase(historyBuffer.begin());
        }

        for (auto line : historyBuffer)
        {
            ImGui::TextWrapped("%s", line.c_str());
        }

        ImGui::SetScrollHereY(0.0f);
    }
    
    ImGui::EndChild();
    ImGui::End();
}