
#include "Buffer.hpp"
#include "imgui.h"

class InfoBar
{
private:
    /* data */
public:
    InfoBar() = default;
    ~InfoBar() = default;

    void OnNewFrame(Eigen::Vector2d& mousePosWorld, Buffer<ImPlotPoint>& fpsBuffer) 
    {
        double averageFps = 0;
        for (int i = 0; i < fpsBuffer.size(); i++)
        {
            averageFps += fpsBuffer.data()[i][1];
        }
        averageFps /= fpsBuffer.size();
    
        ImGui::Begin("##FPS"); 
        ImGui::Text("FPS: %d ", (int)averageFps);
        ImGui::SameLine(); 
        ImGui::Text("| Camera Pos: %f, %f ", ViewPort::GetInstance().GetCamera().getPosition().x(), ViewPort::GetInstance().GetCamera().getPosition().y());
        ImGui::SameLine();
        ImGui::Text("| Camera Scale: %d", ViewPort::GetInstance().GetCamera().getScale());
        ImGui::SameLine();
        ImGui::Text("| Mouse World Pos: %f, %f ", mousePosWorld.x(), mousePosWorld.y());
        ImGui::End();
    }
};

