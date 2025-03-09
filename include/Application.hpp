#pragma once
#include "BaseApplication.hpp"
#include "WorldGrid.hpp"
#include "Buffer.hpp"
#include "SerialInterface.hpp"

#define FPS_BUFFER_SIZE 500

#define SERIAL_LINE_SIZE_BYTES 64
#define SERIAL_HISTORY_SIZE_LINES 100
#define DEFAULT_PORT "COM3"
#define DEFAULT_BAUDRATE 115200

class Application : public BaseApplication
{
public:
    Application();
    ~Application();
    void OnEvent(SDL_Event *event) override;
    void Update() override;

    void MotorTestWindow();
    void ViewPortWindow();
    void GraphWindow();
    void fpsWindow();
    void ConfigWindow();
    void SerialMonitor();

    double averageFps;
    GridRenderer worldGrid;
    Buffer<ImPlotPoint> fpsBuffer;
    SerialInterface serial;
};

