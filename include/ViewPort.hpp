#include "SDL3/SDL.h"
#include "imgui.h"
#include "RendererBackend.hpp"

#define SDL_VIEWPORT "##sdl_viewport"

class ViewPort
{
public:
    ViewPort(RendererBackend* rendererBackend);
    ~ViewPort();
    void ViewPortBegin();
    void ViewPortEnd();
    void RenderViewport();

private:
    RendererBackend* m_rendererBackend;
    SDL_Texture* m_renderTexture;
};

ViewPort::ViewPort(RendererBackend* rendererBackend) : m_rendererBackend(rendererBackend)
{

}

ViewPort::~ViewPort()
{
    if(m_renderTexture) SDL_DestroyTexture(m_renderTexture);
}

inline void ViewPort::ViewPortBegin()
{
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0)); // Remove padding
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
    ImGui::Begin(SDL_VIEWPORT, NULL, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse); // Display the texture in an ImGui windo
}

inline void ViewPort::ViewPortEnd()
{
    ImGui::End();
    ImGui::PopStyleVar(2);
}

inline void ViewPort::RenderViewport()
{
    if(m_renderTexture) SDL_DestroyTexture(m_renderTexture);

    ViewPortBegin();

    ImVec2 textureSize = ImGui::GetContentRegionAvail();
    m_renderTexture = SDL_CreateTexture(m_rendererBackend->GetSdlRenderer(), SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_TARGET, (int)textureSize.x, (int)textureSize.y);
    //m_camera.setScreenSize({textureSize.x, textureSize.y });

    SDL_SetRenderTarget(m_rendererBackend->GetSdlRenderer(), m_renderTexture);
    SDL_SetRenderDrawColor(m_rendererBackend->GetSdlRenderer(), DARK_GREY, 255);
    SDL_RenderClear(m_rendererBackend->GetSdlRenderer());   // Render stuff after this

    //RenderAll();

    SDL_SetRenderTarget(m_rendererBackend->GetSdlRenderer(), nullptr);  // And before this
    ImTextureID textureID = (ImTextureID)m_renderTexture; // Cast SDL_Texture* to ImTextureID
    ImGui::Image(textureID, textureSize, ImVec2(0, 0), ImVec2(1, 1));

    ViewPortEnd(); 
}