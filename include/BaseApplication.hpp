#pragma once

#include "ViewPort.hpp"
#include "RendererBackend.hpp"

typedef enum {INIT, RUN, QUIT, RESTART} appState_t;

class BaseApplication
{
public:
    appState_t Run()
    {
        while(appState == RUN)
        {
            this->ProcessEvents();
            m_RendererBackend.StartFrame();
            this->Update();
            viewPort.RenderViewport();
            m_RendererBackend.EndFrame();
        }

        return appState;
    }

protected:
    appState_t appState = INIT;
    ViewPort viewPort;

    virtual void Update() = 0;
    virtual void OnEvent(SDL_Event* event) = 0;

    BaseApplication::BaseApplication() : m_RendererBackend(RendererBackend::GetInstance()), viewPort()
    {
        ImGuiIO& Io = ImGui::GetIO();
        Io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
        appState = RUN; 
    }

    void ProcessEvents()
    {
        static SDL_Event event;
        static unsigned int eventCount = 0;

        while (SDL_PollEvent(&event) != 0)
        {
            if (event.type == SDL_EVENT_QUIT) appState = QUIT;
            ImGui_ImplSDL3_ProcessEvent(&event);
            m_RendererBackend.ProcessEvent(&event);
            this->OnEvent(&event);
            eventCount++;
        }
    }

    Eigen::Vector2d GetViewPortMousePos()
    {
        viewPort.ViewPortBegin();

        ImVec2 mousePos = ImGui::GetMousePos();          // Global mouse position (screen coordinates)
        ImVec2 cursorPos = ImGui::GetCursorScreenPos();  // Top-left corner of the current ImGui content region
        ImVec2 contentMousePos = ImVec2(mousePos.x - cursorPos.x, mousePos.y - cursorPos.y);
        
        viewPort.ViewPortEnd();

        return ImToEigen(contentMousePos);
    }

    Eigen::Vector2d ImToEigen(ImVec2 imVec)
    {
        return Eigen::Vector2d(imVec.x, imVec.y);
    }

    ImVec2 EigenToIm(Eigen::Vector2d eigenVec)
    {
        return ImVec2((float)eigenVec.x(), (float)eigenVec.y());
    }
    
private:
    RendererBackend& m_RendererBackend;
};

