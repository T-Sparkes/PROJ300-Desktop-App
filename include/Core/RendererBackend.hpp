#pragma once

#define SDL_MAIN_HANDLED

#include "SDL3/SDL.h"
#include "imgui.h"
#include "implot.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_sdlrenderer3.h"

#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL3/SDL_opengles2.h>
#else
#include <SDL3/SDL_opengl.h>
#endif

#include <string>

#define BLUE 63, 136, 197
#define DARK_BLUE 03, 86, 147
#define YELLOW 217, 189, 48
#define GREEN 136, 148, 76
#define RED 221, 28, 26
#define WHITE 240, 240, 240
#define DARK_GREY 52, 52, 52
#define GREY 77, 77, 77
#define BLACK 0, 0, 0

#define DEFAULT_WINDOW_SIZE_X 1920
#define DEFAULT_WINDOW_SIZE_Y 1080

class RendererBackend
{
public:
    static RendererBackend& GetInstance(unsigned int windowSizeX = DEFAULT_WINDOW_SIZE_X, unsigned int windowSizeY = DEFAULT_WINDOW_SIZE_Y)
    {
        static RendererBackend instance(windowSizeX, windowSizeY);
        return instance;
    }

    RendererBackend(const RendererBackend&) = delete;
    RendererBackend& operator=(const RendererBackend&) = delete;

    void StartFrame();
    void EndFrame();
    void ProcessEvent(SDL_Event* event);
    SDL_Renderer* GetSdlRenderer();
    SDL_Window* GetSdlWindow();

private:
    RendererBackend(unsigned int windowSizeX, unsigned int windowSizeY);
    ~RendererBackend();

    SDL_Window* m_Window = nullptr; 
    SDL_Renderer* m_sdlRenderer = nullptr;
    float m_DpiScale;
};
