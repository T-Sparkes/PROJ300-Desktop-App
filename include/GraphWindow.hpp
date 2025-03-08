#include "imgui.h"
#include "implot.h"

class GraphWindow
{
private:
    /* data */
public:
    GraphWindow(/* args */);
    ~GraphWindow();
};

GraphWindow::GraphWindow(/* args */)
{
}

GraphWindow::~GraphWindow()
{
}

GraphWindow::OnNewFrame(Buffer<ImPlotPoint>& fpsBuffer)
{
    ImGui::Begin("Graphs");

    if (ImGui::CollapsingHeader("Fps Graph##Header", ImGuiTreeNodeFlags_DefaultOpen) && ImPlot::BeginPlot("Fps##graph")) 
    {
        ImPlot::SetupAxes("Time (s)", "FPS", ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisLimits(ImAxis_Y1, averageFps - 10.0f, averageFps + 10.0f, ImPlotCond_Always);
        ImPlot::PlotLine("##fpsline", &fpsBuffer.data()[0][0], &fpsBuffer.data()[0][1], (int)fpsBuffer.size(), 0, 0, sizeof(fpsBuffer.data()[0]));
        ImPlot::EndPlot();
    }

    ImGui::End();   
}
