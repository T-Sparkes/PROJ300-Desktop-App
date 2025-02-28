#include "Application.hpp"

int main(int argc, char const *argv[])
{
    appState_t returnState = INIT;

    while(returnState != QUIT)
    {
        Application testApp;
        returnState = testApp.Run();
    }
    return 0;
}
