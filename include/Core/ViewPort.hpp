#pragma once

#include<vector>
#include "SDL3/SDL.h"
#include "imgui.h"
#include "RendererBackend.hpp"
#include "Camera2D.hpp"

#define SDL_VIEWPORT "##sdl_viewport"

class ViewPortRenderable;

class ViewPort
{
public:
    static ViewPort &GetInstance();
    ~ViewPort();
    void AddRenderable(ViewPortRenderable *renderable);
    void RemoveRenderable(ViewPortRenderable *renderable);
    void ViewPortBegin();
    void ViewPortEnd();
    Eigen::Vector2d GetViewPortMousePos();
    void RenderViewport();

    void RenderTexture(
        SDL_Texture *texture, 
        Eigen::Vector2d pos, 
        Eigen::Vector2d size, 
        double angle, 
        uint8_t r, uint8_t g, uint8_t b, 
        uint8_t a
    );

    void ViewPort::RenderLineTexture(
        Eigen::Vector2d start, 
        Eigen::Vector2d end,
        double width,  
        uint8_t r, uint8_t g, uint8_t b, 
        uint8_t a
    );
    
    SDL_Texture *LoadTexture(std::string path);

    SDL_Texture* squareTexture = nullptr;
    SDL_Texture* circleTexture = nullptr;
    SDL_Texture* robotTexture = nullptr;

    inline SDL_Renderer* GetSdlRenderer() { return m_rendererBackend.GetSdlRenderer(); }
    inline Camera2D& GetCamera() { return m_camera; }

private:
    ViewPort();
    RendererBackend& m_rendererBackend;
    SDL_Texture* m_renderTexture;
    std::vector<ViewPortRenderable*> m_Renderables;
    Camera2D m_camera;
};


