#pragma once

#include "Core/BaseApplication.hpp"

#include "UI/InfoBar.hpp"
#include "UI/BotControlWindow.hpp"
#include "UI/UIwindow.hpp"
#include "UI/ConfigWindow.hpp"
#include "UI/GraphWindow.hpp"
#include "UI/SerialMonitor.hpp"

#include "Localization/DiffDriveOdom.hpp"
#include "Localization/LandmarkContainer.hpp"
#include "Localization/ConstPosKalmanFilter.hpp"
#include "Localization/OdomKalmanFilter.hpp"
#include "Localization/PathController.hpp"

#include "WorldGrid.hpp"
#include "Buffer.hpp"
#include "SerialInterface.hpp"


#define FPS_BUFFER_SIZE 250

#define GRAPH_FREQ_HZ 50
#define CONTROL_FREQ_HZ 50

#define DEFAULT_GRID_SIZE 4
#define DEFAULT_VIEWPORT_ZOOM 250

#define DEFAULT_LANDMARK_A_POS {-0.65, 0}
#define DEFAULT_LANDMARK_B_POS {0.65, 0}

#define KF_DEFAULT_POS {0, 0, 0}
#define KF_DEFAULT_Q 0.5e-6 //10e-12 //100e-12
#define KF_DEFAULT_R 0.1

/*
    ~~~ NOTES ~~~
    1. Corrupt serial data containing large numbers will freeze program
    2. Need to add kalman filter updates only when new Data is received, DONE, using SDL events
    3. Need to add proper waypoint navigation
    4. Serial interface needs observer pattern to update UI windows: DONE, used sdl events instead
*/

class Application : public BaseApplication 
{
public:
    Application();
    ~Application() = default;

private:
    double m_AvgFrameTime;
    Buffer<ImPlotPoint> m_FrameTBuffer;

    // These render in the order they are declared
    GridRenderer m_WorldGrid;
    SerialInterface m_RobotSerial;
    LandmarkContainer m_Landmarks;
    PathController m_PathController;
    OdomKalmanFilter m_KalmanFilter;

    // UI windows
    std::shared_ptr<InfoBar> m_infoBar;
    std::shared_ptr<SerialMonitor> m_SerialMonitor;
    std::shared_ptr<BotControlWindow> m_ControlPanel;
    std::shared_ptr<ConfigWindow> m_ConfigWindow;
    std::shared_ptr<GraphWindow> m_GraphWindow;

    std::vector<std::shared_ptr<UIwindow>> m_UIwindows;

    void OnEvent(SDL_Event *event) override;
    void Update() override;
    void m_HandleViewportInput();
    void m_CalcFrameTime();
};
