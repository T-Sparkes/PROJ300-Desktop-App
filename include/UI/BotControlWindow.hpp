#include <imgui.h>
#include "UI/UIwindow.hpp"

class BotControlWindow : public UIwindow
{
private:
    /* data */
public:
    BotControlWindow() = default;
    ~BotControlWindow() = default;

    void OnUpdate() override
    {
        ImGui::Begin("Robot Controls");
        {
            
        }
        ImGui::End();
    }
};