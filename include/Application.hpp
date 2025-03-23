#pragma once
#include "Core/BaseApplication.hpp"
#include "UI/InfoBar.hpp"
#include "WorldGrid.hpp"
#include "Buffer.hpp"
#include "SerialInterface.hpp"
#include "SerialMonitor.hpp"
#include "DiffDriveOdom.hpp"
#include "BotControlWindow.hpp"
#include "Bilateration.hpp"
#include "ConstPosKalmanFilter.hpp"

#define FPS_BUFFER_SIZE 500

class Application : public BaseApplication
{
public:
    static Application &GetInstance();
    void OnEvent(SDL_Event *event) override;
    void Update() override;
    ~Application();

private:
    double m_AverageFps;

    std::unique_ptr<SerialMonitor> m_SerialMonitor;
    std::unique_ptr<BotControlWindow> m_ControlPanel;
    std::unique_ptr<InfoBar> m_infoBar;
    std::unique_ptr<Buffer<ImPlotPoint>> m_FpsBuffer;

    GridRenderer m_WorldGrid;
    SerialInterface m_SerialComm;
    DiffDriveOdom m_Odom;
    Bilateration biLat;
    ConstPosKalmanFilter m_KalmanFilter;

    Application();
    void MotorTestWindow();
    void ViewPortWindow();
    void GraphWindow();
    void ConfigWindow();
};

