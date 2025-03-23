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
#include "UI/UIwindow.hpp"

#define FPS_BUFFER_SIZE 500

class Application : public BaseApplication
{
public:
    static Application &GetInstance();
    ~Application();
    
private:
    double m_AverageFps;

    GridRenderer m_WorldGrid;
    SerialInterface m_SerialComm;
    DiffDriveOdom m_Odom;
    Bilateration biLat;
    ConstPosKalmanFilter m_KalmanFilter;

    std::shared_ptr<InfoBar> m_infoBar;
    std::shared_ptr<Buffer<ImPlotPoint>> m_FpsBuffer;
    std::shared_ptr<SerialMonitor> m_SerialMonitor;
    std::shared_ptr<BotControlWindow> m_ControlPanel;

    std::vector<std::shared_ptr<UIwindow>> m_UIwindows;

    Application();
    void OnEvent(SDL_Event *event) override;
    void Update() override;
    void MotorTestWindow();
    void ViewPortWindow();
    void GraphWindow();
    void ConfigWindow();
};

