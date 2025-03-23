
#include "Core/ViewPort.hpp"
#include "Core/ViewPortRenderable.hpp"
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ViewPort& ViewPort::GetInstance()
{
    static ViewPort instance;
    return instance;
}

ViewPort::ViewPort() : m_camera({0, 0}, {0, 0}, 100), m_rendererBackend(RendererBackend::GetInstance())
{
    circleTexture = LoadTexture("textures\\circle.bmp");
    robotTexture = LoadTexture("textures\\robot.bmp");
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

Eigen::Vector2d ViewPort::GetViewPortMousePos()
{
    ViewPortBegin();

    ImVec2 mousePos = ImGui::GetMousePos();          // Global mouse position (screen coordinates)
    ImVec2 cursorPos = ImGui::GetCursorScreenPos();  // Top-left corner of the current ImGui content region
    ImVec2 contentMousePos = ImVec2(mousePos.x - cursorPos.x, mousePos.y - cursorPos.y);
    
    ViewPortEnd();

    return Eigen::Vector2d(contentMousePos.x, contentMousePos.y);;
}


void ViewPort::RenderViewport()
{
    if(m_renderTexture) SDL_DestroyTexture(m_renderTexture);

    ViewPortBegin();

    ImVec2 textureSize = ImGui::GetContentRegionAvail();
    m_renderTexture = SDL_CreateTexture(m_rendererBackend.GetSdlRenderer(), SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_TARGET, (int)textureSize.x, (int)textureSize.y);
    m_camera.setScreenSize({textureSize.x, textureSize.y});

    SDL_SetRenderTarget(m_rendererBackend.GetSdlRenderer(), m_renderTexture);
    SDL_SetRenderDrawColor(m_rendererBackend.GetSdlRenderer(), DARK_GREY, 255);
    SDL_RenderClear(m_rendererBackend.GetSdlRenderer());   // Render stuff after this

    for (auto renderable : m_Renderables)
    {
        renderable->render();
    }

    SDL_SetRenderTarget(m_rendererBackend.GetSdlRenderer(), nullptr);  // And before this
    ImTextureID textureID = (ImTextureID)m_renderTexture; // Cast SDL_Texture* to ImTextureID
    ImGui::Image(textureID, textureSize, ImVec2(0, 0), ImVec2(1, 1));

    ViewPortEnd(); 
}

void ViewPort::RenderTexture( SDL_Texture* texture, Eigen::Vector2d pos, Eigen::Vector2d size, double angle, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    //Render a texture at a specific location, with a specific size, rotation, colour and alpa, x,y is center of texture

    pos = m_camera.transform * pos;

    SDL_FRect destRect = 
    {
        (float)(pos.x() - (size.x() * m_camera.getScale()) / 2.0), 
        (float)(pos.y() - (size.y() * m_camera.getScale()) / 2.0), 
        (float)(size.x() * m_camera.getScale()), 
        (float)(size.y() * m_camera.getScale())
    };  //Create a rect to render the texture to

    SDL_SetTextureColorMod(texture, r, g, b);  //Set the colour of the texture
    SDL_SetTextureAlphaMod(texture, a);  //Set the alpha of the texture
    SDL_RenderTextureRotated(m_rendererBackend.GetSdlRenderer(), texture, NULL, &destRect, ((angle + m_camera.rotation) * (180.0 / M_PI)), NULL, SDL_FLIP_NONE);
}

SDL_Texture* ViewPort::LoadTexture(std::string path)
{
    std::string sfilePath = SDL_GetBasePath();

    SDL_Surface* tempSerface = SDL_LoadBMP((sfilePath + path).c_str());
    if (tempSerface == NULL) SDL_Log("Could not load surface: %s\n", SDL_GetError());

    SDL_Texture* texture = SDL_CreateTextureFromSurface(m_rendererBackend.GetSdlRenderer(), tempSerface);
    if (texture == NULL) SDL_Log("Could not load texture: %s\n", SDL_GetError());

    SDL_DestroySurface(tempSerface);

    return texture;
}