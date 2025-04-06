#include "Application.hpp"

int main(int argc, char const *argv[])
{
    appState_t returnState = INIT;
    Application* PROJ300;

    while (returnState != QUIT)
    {
        PROJ300 = new Application();
        returnState = PROJ300->Run();
        delete PROJ300;
    }
    return 0;
}
