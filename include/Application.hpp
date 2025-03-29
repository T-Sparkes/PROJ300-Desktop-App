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
#include "UI/ConfigWindow.hpp"
#include "OdomKalmanFilter.hpp"

#define FPS_BUFFER_SIZE 500

#define DEFAULT_GRID_SIZE 4
#define DEFAULT_VIEWPORT_ZOOM 250

#define DEFAULT_ANCHOR_A_POS {-1, 0}
#define DEFAULT_ANCHOR_B_POS {1, 0}

#define KF_DEFAULT_POS {0, -1}
#define KF_DEFAULT_Q 1e-6
#define KF_DEFAULT_R 0.1

/*
    ~~~ NOTES ~~~
    1. Corrupt serial data containing large numbers will freeze program
    2. Need to add kalman filter updates only when new Data is received
*/

class Application : public BaseApplication
{
public:
    static Application &GetInstance();
    
private:
    double m_AvgFrameTime;
    std::unique_ptr<Buffer<ImPlotPoint>> m_FrameTBuffer;

    GridRenderer m_WorldGrid;
    SerialInterface m_RobotSerial;
    DiffDriveOdom m_Odom;
    Bilateration m_biLat;
    ConstPosKalmanFilter m_KalmanFilter;
    OdomKalmanFilter m_OdomKalmanFilter;

    std::shared_ptr<InfoBar> m_infoBar;
    std::shared_ptr<SerialMonitor> m_SerialMonitor;
    std::shared_ptr<BotControlWindow> m_ControlPanel;
    std::shared_ptr<ConfigWindow> m_ConfigWindow;

    std::vector<std::shared_ptr<UIwindow>> m_UIwindows;

    Application();
    void OnEvent(SDL_Event *event) override;
    void Update() override;
    void ViewPortWindow();
    void GraphWindow();
    void CalcFrameTime();
};
