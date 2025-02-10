#include "RendererBackend.hpp"

RendererBackend::RendererBackend()
{
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();

    ImGuiIO& Io = ImGui::GetIO();
    Io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    SDL_Init(SDL_INIT_VIDEO);
    if (!SDL_CreateWindowAndRenderer(nullptr, DEFAULT_WINDOW_SIZE_X, DEFAULT_WINDOW_SIZE_Y, SDL_WINDOW_RESIZABLE, &m_Window, &m_sdlRenderer))
    {
        SDL_Log("%S", SDL_GetError());
    }

    ImGui_ImplSDL3_InitForSDLRenderer(m_Window, m_sdlRenderer);
    ImGui_ImplSDLRenderer3_Init(m_sdlRenderer);

    SDL_SetRenderVSync(m_sdlRenderer, 1);
    m_DpiScale = SDL_GetWindowDisplayScale(m_Window);
}

/// @brief Start the new ImGui frame, windows must be called / created after this
void RendererBackend::StartFrame()
{
    ImGui_ImplSDLRenderer3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
    ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);    
}

/// @brief Ends the ImGui frame and renders it using SDL3
void RendererBackend::EndFrame()
{
    ImGui::Render();
    SDL_SetRenderDrawColor(m_sdlRenderer, BLACK, 255);
    SDL_RenderClear(m_sdlRenderer);
    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), m_sdlRenderer);
    SDL_RenderPresent(m_sdlRenderer);    
}

/// @brief Handles SDL events related to renderering, Dpi changes for example
/// @param event SDL event object 
void RendererBackend::ProcessEvent(SDL_Event* event)
{
    ImGuiIO& io = ImGui::GetIO();
    ImGuiStyle& style = ImGui::GetStyle();
    
    switch (event->type)
    {
    case SDL_EVENT_WINDOW_DISPLAY_SCALE_CHANGED: // Scales Gui when dpi changes
        style.ScaleAllSizes(1.0f / m_DpiScale);
        m_DpiScale = SDL_GetWindowDisplayScale(m_Window);
        style.ScaleAllSizes(m_DpiScale);
        io.FontGlobalScale = m_DpiScale;
        break;

    case SDL_EVENT_WINDOW_RESIZED:
        break;
    
    default:
        break;
    }
}

SDL_Renderer *RendererBackend::GetSdlRenderer()
{
    return m_sdlRenderer;
}

RendererBackend::~RendererBackend()
{
    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    ImPlot::DestroyContext();

    SDL_DestroyRenderer(m_sdlRenderer);
    SDL_DestroyWindow(m_Window);
    SDL_Quit();
}