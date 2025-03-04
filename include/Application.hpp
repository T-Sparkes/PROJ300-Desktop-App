#include "BaseApplication.hpp"
#include "WorldGrid.hpp"
#include "InfoBar.hpp"
#include "Buffer.hpp"

class Application : public BaseApplication
{
public:
    Application();
    ~Application();
    void OnEvent(SDL_Event *event) override;
    void Update() override;

    void ViewPortWindow();

private:
    GridRenderer worldGrid;
    InfoBar infoBar;
    Buffer<ImPlotPoint> fpsBuffer;
};

