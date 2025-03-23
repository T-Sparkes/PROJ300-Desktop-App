#pragma once
#include <imgui.h>
#include <Eigen/Dense>
#include "ViewPort.hpp"
#include "Application.hpp"

class InfoBar
{
private:
    SerialInterface* m_SerialComm;
    double& m_AverageFps;

public:
    InfoBar(SerialInterface* SerialComm, double& AverageFps);
    ~InfoBar();
    void onUpdate();
};

inline InfoBar::InfoBar(SerialInterface* SerialComm, double& AverageFps) : m_AverageFps(AverageFps)
{
    m_SerialComm = SerialComm;
}

inline InfoBar::~InfoBar()
{
    
}

inline void InfoBar::onUpdate()
{
    ImGuiIO& io = ImGui::GetIO();
    ViewPort& viewPort = ViewPort::GetInstance();
    Eigen::Vector2d mousePosWorld = viewPort.GetCamera().transform.inverse() * viewPort.GetViewPortMousePos();

    ImGui::Begin("##FPS");
    { 
        ImGui::Text("FPS: %d ", (int)m_AverageFps);

        ImGui::SameLine(); 
        ImGui::Text("| Camera Pos: %f, %f ", viewPort.GetCamera().getPosition().x(), viewPort.GetCamera().getPosition().y());

        ImGui::SameLine();
        ImGui::Text("| Camera Scale: %d", viewPort.GetCamera().getScale());

        ImGui::SameLine();
        ImGui::Text("| Mouse World Pos: %f, %f ", mousePosWorld.x(), mousePosWorld.y());

        ImGui::SameLine();
        ImGui::Text("| Robot Connetion Status: ");

        ImGui::SameLine();
        static StatusPacket status;
        m_SerialComm->getPacket(&status);

        if (status.connected)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0, 255, 0, 255));
            ImGui::Text("Connected");
            ImGui::PopStyleColor();
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(255, 0, 0, 255));
            ImGui::Text("No Connection");
            ImGui::PopStyleColor();
        }        
    }
    ImGui::End();
}