#include "imgui.h"
#include "implot.h"
#include "UI/UIwindow.hpp"
#include "Buffer.hpp"

#define GRAPH_BUFFER_SIZE 250

class GraphWindow : public UIwindow
{
private:
    struct kData
    {
        double time;
        Eigen::Matrix<double, 3, 2> K;
    };

    struct pData
    {
        double time;
        Eigen::Matrix3d P;
    };

    double& m_AvgFrameTime;

    Buffer<ImPlotPoint>& m_FrameTBuffer;
    Buffer<ImPlotPoint> rangeBufferA;
    Buffer<ImPlotPoint> rangeBufferB;
    Buffer<ImPlotPoint> kalmanRangeA;
    Buffer<ImPlotPoint> kalmanRangeB;

    Buffer<kData> kBuffer;
    Buffer<pData> pBuffer;

public:

    GraphWindow(Buffer<ImPlotPoint>& FrameTBuffer, double& AvgFrameTime) : 
        m_FrameTBuffer(FrameTBuffer), 
        m_AvgFrameTime(AvgFrameTime), 
        rangeBufferA(GRAPH_BUFFER_SIZE), 
        rangeBufferB(GRAPH_BUFFER_SIZE),
        kalmanRangeA(GRAPH_BUFFER_SIZE), 
        kalmanRangeB(GRAPH_BUFFER_SIZE),
        kBuffer(GRAPH_BUFFER_SIZE),
        pBuffer(GRAPH_BUFFER_SIZE)
    {}

    void GraphWindow::OnUpdate() override
    {
        ImGui::Begin("Graphs");
        {
            if (ImGui::CollapsingHeader("Frametime Graph##Header", ImGuiTreeNodeFlags_DefaultOpen) && ImPlot::BeginPlot("Frametime##graph")) 
            {
                ImPlot::SetupAxes("Time (s)", "Frametime (ms)", ImPlotAxisFlags_AutoFit);
                ImPlot::SetupAxisLimits(ImAxis_Y1, m_AvgFrameTime - 10.0f, m_AvgFrameTime + 10.0f, ImPlotCond_Always);
                ImPlot::PlotLine("##fpsline", &m_FrameTBuffer.data()[0][0], &m_FrameTBuffer.data()[0][1], (int)m_FrameTBuffer.size(), 0, 0, sizeof(m_FrameTBuffer.data()[0]));
                ImPlot::EndPlot();
            }

            if (ImGui::CollapsingHeader("Anchor Ranges##Header", ImGuiTreeNodeFlags_DefaultOpen) && ImPlot::BeginPlot("Anchor Ranges")) 
            {
                ImPlot::SetupAxes("Time (s)", "Range (m)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                ImPlot::PlotLine("rangeA"  , &rangeBufferA.data()[0][0], &rangeBufferA.data()[0][1], (int)rangeBufferA.size(), 0, 0, sizeof(rangeBufferA.data()[0]));
                ImPlot::PlotLine("rangeB"  , &rangeBufferB.data()[0][0], &rangeBufferB.data()[0][1], (int)rangeBufferB.size(), 0, 0, sizeof(rangeBufferB.data()[0]));
                ImPlot::PlotLine("Kalman A", &kalmanRangeA.data()[0][0], &kalmanRangeA.data()[0][1], (int)kalmanRangeA.size(), 0, 0, sizeof(kalmanRangeA.data()[0]));
                ImPlot::PlotLine("Kalman B", &kalmanRangeB.data()[0][0], &kalmanRangeB.data()[0][1], (int)kalmanRangeB.size(), 0, 0, sizeof(kalmanRangeB.data()[0]));
                ImPlot::EndPlot();
            }

            if (ImGui::CollapsingHeader("K Matrix Graph##Header", ImGuiTreeNodeFlags_DefaultOpen) && ImPlot::BeginPlot("K Matrix ##graph"))
            {
                ImPlot::SetupAxes("Time (s)", "Gain", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                // Row 0: x state gains
                ImPlot::PlotLine("K(x, r_a)", &kBuffer.data()[0].time, &kBuffer.data()[0].K(0, 0), (int)kBuffer.size(), 0, 0, sizeof(kBuffer.data()[0]));
                ImPlot::PlotLine("K(x, r_b)", &kBuffer.data()[0].time, &kBuffer.data()[0].K(0, 1), (int)kBuffer.size(), 0, 0, sizeof(kBuffer.data()[0]));
                // Row 1: y state gains
                ImPlot::PlotLine("K(y, r_a)", &kBuffer.data()[0].time, &kBuffer.data()[0].K(1, 0), (int)kBuffer.size(), 0, 0, sizeof(kBuffer.data()[0]));
                ImPlot::PlotLine("K(y, r_b)", &kBuffer.data()[0].time, &kBuffer.data()[0].K(1, 1), (int)kBuffer.size(), 0, 0, sizeof(kBuffer.data()[0]));
                // Row 2: theta state gains
                ImPlot::PlotLine("K(θ, r_a)", &kBuffer.data()[0].time, &kBuffer.data()[0].K(2, 0), (int)kBuffer.size(), 0, 0, sizeof(kBuffer.data()[0]));
                ImPlot::PlotLine("K(θ, r_b)", &kBuffer.data()[0].time, &kBuffer.data()[0].K(2, 1), (int)kBuffer.size(), 0, 0, sizeof(kBuffer.data()[0]));
                ImPlot::EndPlot();
            }
        
            if (ImGui::CollapsingHeader("P Matrix Graph##Header", ImGuiTreeNodeFlags_DefaultOpen) && ImPlot::BeginPlot("P Matrix ##graph"))
            {
                ImPlot::SetupAxes("Time (s)", "Variance", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                // Row 0: x state variances
                ImPlot::PlotLine("P(x, x)", &pBuffer.data()[0].time, &pBuffer.data()[0].P(0, 0), (int)pBuffer.size(), 0, 0, sizeof(pBuffer.data()[0]));
                ImPlot::PlotLine("P(x, y)", &pBuffer.data()[0].time, &pBuffer.data()[0].P(0, 1), (int)pBuffer.size(), 0, 0, sizeof(pBuffer.data()[0]));
                ImPlot::PlotLine("P(x, θ)", &pBuffer.data()[0].time, &pBuffer.data()[0].P(0, 2), (int)pBuffer.size(), 0, 0, sizeof(pBuffer.data()[0]));
                // Row 1: y state variances
                ImPlot::PlotLine("P(y, x)", &pBuffer.data()[0].time, &pBuffer.data()[0].P(1, 0), (int)pBuffer.size(), 0, 0, sizeof(pBuffer.data()[0]));
                ImPlot::PlotLine("P(y, y)", &pBuffer.data()[0].time, &pBuffer.data()[0].P(1, 1), (int)pBuffer.size(), 0, 0, sizeof(pBuffer.data()[0]));
                ImPlot::PlotLine("P(y, θ)", &pBuffer.data()[0].time, &pBuffer.data()[0].P(1, 2), (int)pBuffer.size(), 0, 0, sizeof(pBuffer.data()[0]));
                // Row 2: theta state variances
                ImPlot::PlotLine("P(θ, x)", &pBuffer.data()[0].time, &pBuffer.data()[0].P(2, 0), (int)pBuffer.size(), 0, 0, sizeof(pBuffer.data()[0]));
                ImPlot::PlotLine("P(θ, y)", &pBuffer.data()[0].time, &pBuffer.data()[0].P(2, 1), (int)pBuffer.size(), 0, 0, sizeof(pBuffer.data()[0]));
                ImPlot::PlotLine("P(θ, θ)", &pBuffer.data()[0].time, &pBuffer.data()[0].P(2, 2), (int)pBuffer.size(), 0, 0, sizeof(pBuffer.data()[0]));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();
    }

    void addRangeData(Eigen::Vector2d rangeData, Eigen::Vector2d KalmanData)
    {
        rangeBufferA.addData({SDL_GetTicks() / 1000.0, rangeData(0)});
        rangeBufferB.addData({SDL_GetTicks() / 1000.0, rangeData(1)});
        kalmanRangeA.addData({SDL_GetTicks() / 1000.0, KalmanData(0)});
        kalmanRangeB.addData({SDL_GetTicks() / 1000.0, KalmanData(1)});
    }

    void addKalmanData(Eigen::Matrix<double, 3, 2> K, Eigen::Matrix3d P)
    {
        kBuffer.addData({SDL_GetTicks() / 1000.0, K});
        pBuffer.addData({SDL_GetTicks() / 1000.0, P});
    }
};

