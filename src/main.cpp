#include "Application.hpp"

int main(int argc, char const *argv[])
{
    appState_t returnState = INIT;

    while(returnState != QUIT)
    {
        Application& PROJ300 = Application::GetInstance();
        returnState = PROJ300.Run();
    }
    return 0;
}
