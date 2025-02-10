#include "RendererBackend.hpp"

int main(int argc, char const *argv[])
{
    bool quit = false;
    SDL_Event event;
    RendererBackend rendererBackend;

    while (!quit)
    {
        while (SDL_PollEvent(&event) != 0)
        {
            if (event.type == SDL_EVENT_QUIT) quit = true;
            ImGui_ImplSDL3_ProcessEvent(&event);
            rendererBackend.ProcessEvent(&event);
        }

        rendererBackend.StartFrame();
        ImGui::ShowDemoWindow();
        rendererBackend.EndFrame();
    }
    
    return 0;
}
