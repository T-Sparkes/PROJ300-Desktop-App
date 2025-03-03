#include "BaseApplication.hpp"
#include "WorldGrid.hpp"

class Application : public BaseApplication
{
private:
    /* data */
public:
    Application();
    ~Application();
    void OnEvent(SDL_Event *event) override;
    void Update() override;

    void ViewPortWindow();

    GridRenderer worldGrid;
    GridRenderer worldGridSmall;
};

Application::Application() : worldGrid({0, 0}, {10, 10})
{
    worldGrid.gridStep = 1;
    worldGrid.bRender = true;
}

Application::~Application()
{
}

inline void Application::OnEvent(SDL_Event* event) 
{
    if (ImGui::IsKeyDown(ImGuiKey_Enter) && ImGui::IsKeyDown(ImGuiKey_LeftAlt))
    {
        appState = RESTART;
    }
}

inline void Application::Update()
{
    ImGui::ShowDemoWindow();
    ViewPortWindow();
}

inline void Application::ViewPortWindow()
{
    viewPort.ViewPortBegin();

    Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * GetViewPortMousePos();

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) viewPort.GetCamera().setPanStart(mousePosWorld);
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) viewPort.GetCamera().panCamera(mousePosWorld);
    if (abs(ImGui::GetIO().MouseWheel) > 0) viewPort.GetCamera().updateZoom(mousePosWorld, ImGui::GetIO().MouseWheel);

    viewPort.ViewPortEnd();
}