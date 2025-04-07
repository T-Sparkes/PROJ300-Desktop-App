#pragma once

#include "ViewPort.hpp"
#include "RendererBackend.hpp"

typedef enum {INIT, RUN, QUIT, RESTART} appState_t;

class BaseApplication
{
public:
    appState_t Run()
    {
        SDL_LogVerbose(SDL_LOG_CATEGORY_APPLICATION, "APP INFO: Application started.");
        while(appState == RUN)
        {
            m_RendererBackend.StartFrame();
            this->ProcessEvents();
            this->Update();
            m_ViewPort.RenderViewport();
            m_RendererBackend.EndFrame();
        }
        return appState;
    }

protected:
    appState_t appState = INIT;
    ViewPort& m_ViewPort;

    virtual void Update() = 0;
    virtual void OnEvent(SDL_Event* event) = 0;

    BaseApplication::BaseApplication() : m_RendererBackend(RendererBackend::GetInstance()), m_ViewPort(ViewPort::GetInstance())
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
        event.type = SDL_EVENT_LAST; // Dummy event to trigger ImGui event processing
        this->OnEvent(&event); // Final event for ImGui Io to update
        eventCount = 0;
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

