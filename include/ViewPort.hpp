#pragma once

#include<vector>
#include "SDL3/SDL.h"
#include "imgui.h"
#include "RendererBackend.hpp"

#define SDL_VIEWPORT "##sdl_viewport"

class ViewPortRenderable;

class ViewPort
{
public:
    ViewPort(RendererBackend* rendererBackend);
    ~ViewPort();
    void AddRenderable(ViewPortRenderable *renderable);
    void RemoveRenderable(ViewPortRenderable *renderable);
    void ViewPortBegin();
    void ViewPortEnd();
    void RenderViewport();

private:
    RendererBackend* m_rendererBackend;
    SDL_Texture* m_renderTexture;
    std::vector<ViewPortRenderable*> m_Renderables;
};