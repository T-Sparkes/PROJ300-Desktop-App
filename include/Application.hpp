#include "BaseApplication.hpp"
#include "WorldGrid.hpp"
#include "Buffer.hpp"

#define FPS_BUFFER_SIZE 500

class Application : public BaseApplication
{
public:
    Application();
    ~Application();
    void OnEvent(SDL_Event *event) override;
    void Update() override;

    void ViewPortWindow();
    void GraphWindow();
    void fpsWindow();
    void ConfigWindow();

private:
    double averageFps;

    GridRenderer worldGrid;
    Buffer<ImPlotPoint> fpsBuffer;
};

