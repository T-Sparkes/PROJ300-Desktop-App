#include "Application.hpp"

int main(int argc, char const *argv[])
{
    Application& PROJ300 = Application::GetInstance();
    PROJ300.Run();
    return 0;
}
