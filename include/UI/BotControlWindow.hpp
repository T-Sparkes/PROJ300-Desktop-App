#include <imgui.h>

class BotControlWindow
{
private:
    /* data */
public:
    BotControlWindow() = default;
    ~BotControlWindow() = default;

    void onUpdate()
    {
        ImGui::Begin("Robot Controls");
        {
            
        }
        ImGui::End();
    }
};