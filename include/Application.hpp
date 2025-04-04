#pragma once
#include "Core/BaseApplication.hpp"
#include "UI/InfoBar.hpp"
#include "WorldGrid.hpp"
#include "Buffer.hpp"
#include "SerialInterface.hpp"
#include "SerialMonitor.hpp"
#include "DiffDriveOdom.hpp"
#include "BotControlWindow.hpp"
#include "LandmarkContainer.hpp"
#include "ConstPosKalmanFilter.hpp"
#include "UI/UIwindow.hpp"
#include "UI/ConfigWindow.hpp"
#include "OdomKalmanFilter.hpp"
#include "UI/GraphWindow.hpp"

#define FPS_BUFFER_SIZE 250

#define DEFAULT_GRID_SIZE 4
#define DEFAULT_VIEWPORT_ZOOM 250

#define DEFAULT_LANDMARK_A_POS {-0.65, 0}
#define DEFAULT_LANDMARK_B_POS {0.65, 0}

#define KF_DEFAULT_POS {0, 0, 0}
#define KF_DEFAULT_Q 100e-12 //10e-12 //100e-12
#define KF_DEFAULT_R 0.1

/*
    ~~~ NOTES ~~~
    1. Corrupt serial data containing large numbers will freeze program
    2. Need to add kalman filter updates only when new Data is received
    3. Need to add proper waypoint navigation
    4. Serial interface needs observer pattern to update UI windows
*/

class Application : public BaseApplication 
{
public:
    static Application &GetInstance();
    
private:
    double m_AvgFrameTime;
    Buffer<ImPlotPoint> m_FrameTBuffer;

    GridRenderer m_WorldGrid;
    SerialInterface m_RobotSerial;
    LandmarkContainer m_Landmarks;
    OdomKalmanFilter m_KalmanFilter;

    std::shared_ptr<InfoBar> m_infoBar;
    std::shared_ptr<SerialMonitor> m_SerialMonitor;
    std::shared_ptr<BotControlWindow> m_ControlPanel;
    std::shared_ptr<ConfigWindow> m_ConfigWindow;
    std::shared_ptr<GraphWindow> m_GraphWindow;

    std::vector<std::shared_ptr<UIwindow>> m_UIwindows;

    Application();
    void OnEvent(SDL_Event *event) override;
    void Update() override;
    void m_ViewPortWindow();
    void m_CalcFrameTime();
    void m_NewLandmarkData(AnchorRangePacket *packet);
};
