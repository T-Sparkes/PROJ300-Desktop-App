
#include "Application.hpp"

Application::Application() : worldGrid({0, 0}, {10, 10}), fpsBuffer(100)
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
}   

void Application::Update()
{
    ImGuiIO& io = ImGui::GetIO();
    fpsBuffer.addData({(double)SDL_GetTicks() / 1000.0, io.Framerate});

    Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * GetViewPortMousePos();
    infoBar.OnNewFrame(mousePosWorld, fpsBuffer);

    ViewPortWindow();
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