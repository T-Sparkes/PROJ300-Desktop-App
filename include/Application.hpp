#pragma once
#include "Core/BaseApplication.hpp"
#include "WorldGrid.hpp"
#include "Buffer.hpp"
#include "SerialInterface.hpp"
#include "SerialMonitor.hpp"
#include "DiffDriveOdom.hpp"
#include "BotControlWindow.hpp"
#include "Bilateration.hpp"
#include "ConstPosKalmanFilter.hpp"
#include "UI/InfoBar.hpp"

#define FPS_BUFFER_SIZE 500

class Application : public BaseApplication
{
public:
    static Application &GetInstance();
    ~Application();
    void OnEvent(SDL_Event *event) override;
    void Update() override;

private:
    double m_AverageFps;
    GridRenderer m_WorldGrid;
    Buffer<ImPlotPoint>* m_FpsBuffer;
    SerialInterface m_SerialComm;
    SerialMonitor* m_SerialMonitor;
    BotControlWindow* m_ControlPanel;
    DiffDriveOdom m_Odom;
    Bilateration biLat;
    ConstPosKalmanFilter m_KalmanFilter;
    InfoBar* m_infoBar;

    Application();
    void MotorTestWindow();
    void ViewPortWindow();
    void GraphWindow();
    void ConfigWindow();
};

