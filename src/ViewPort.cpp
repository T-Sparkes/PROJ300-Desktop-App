
#include "ViewPort.hpp"
#include "ViewPortRenderable.hpp"
#include <algorithm>

ViewPort::ViewPort(RendererBackend* rendererBackend) : m_rendererBackend(rendererBackend)
{

}

ViewPort::~ViewPort()
{
    if(m_renderTexture) SDL_DestroyTexture(m_renderTexture);
}

void ViewPort::AddRenderable(ViewPortRenderable* renderable)
{
    m_Renderables.push_back(renderable);
    SDL_Log("Added 1 Renderable - Total: %d\n", m_Renderables.size());
}

void ViewPort::RemoveRenderable(ViewPortRenderable* renderable)
{
    m_Renderables.erase(std::find(m_Renderables.begin(), m_Renderables.end(), renderable));
    SDL_Log("Removed 1 Renderable - Total: %d\n", m_Renderables.size());
}

void ViewPort::ViewPortBegin()
{
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0)); // Remove padding
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
    ImGui::Begin(SDL_VIEWPORT, NULL, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse); // Display the texture in an ImGui windo
}

void ViewPort::ViewPortEnd()
{
    ImGui::End();
    ImGui::PopStyleVar(2);
}

void ViewPort::RenderViewport()
{
    if(m_renderTexture) SDL_DestroyTexture(m_renderTexture);

    ViewPortBegin();

    ImVec2 textureSize = ImGui::GetContentRegionAvail();
    m_renderTexture = SDL_CreateTexture(m_rendererBackend->GetSdlRenderer(), SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_TARGET, (int)textureSize.x, (int)textureSize.y);
    //m_camera.setScreenSize({textureSize.x, textureSize.y });

    SDL_SetRenderTarget(m_rendererBackend->GetSdlRenderer(), m_renderTexture);
    SDL_SetRenderDrawColor(m_rendererBackend->GetSdlRenderer(), DARK_GREY, 255);
    SDL_RenderClear(m_rendererBackend->GetSdlRenderer());   // Render stuff after this

    for (auto renderable : m_Renderables)
    {
        renderable->render();
    }

    SDL_SetRenderTarget(m_rendererBackend->GetSdlRenderer(), nullptr);  // And before this
    ImTextureID textureID = (ImTextureID)m_renderTexture; // Cast SDL_Texture* to ImTextureID
    ImGui::Image(textureID, textureSize, ImVec2(0, 0), ImVec2(1, 1));

    ViewPortEnd(); 
}