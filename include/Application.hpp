#pragma once
#include "BaseApplication.hpp"
#include "WorldGrid.hpp"
#include "Buffer.hpp"
#include "SerialInterface.hpp"
#include "SerialMonitor.hpp"
#include "DiffDriveOdom.hpp"

#define FPS_BUFFER_SIZE 500

class Application : public BaseApplication
{
public:
    Application();
    ~Application();
    void OnEvent(SDL_Event *event) override;
    void Update() override;

private:
    double m_AverageFps;
    GridRenderer m_WorldGrid;
    Buffer<ImPlotPoint> m_FpsBuffer;
    SerialInterface m_SerialComm;
    SerialMonitor* m_SerialMonitor;
    DiffDriveOdom odom;

    void MotorTestWindow();
    void ViewPortWindow();
    void GraphWindow();
    void fpsWindow();
    void ConfigWindow();
};

