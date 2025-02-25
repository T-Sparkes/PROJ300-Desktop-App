
#include "ViewPortRenderable.hpp"

class testRenderable : public ViewPortRenderable
{
private:
    Eigen::Vector2d m_pos;

public:
    testRenderable(ViewPort* targetViewPort, Eigen::Vector2d pos = {0, 0}) : m_pos(pos), ViewPortRenderable(targetViewPort) {}

    void render() override
    {
        m_targetViewPort->RenderTexture(m_targetViewPort->circleTexture, m_pos, {1, 1}, 0.0, RED, SDL_ALPHA_OPAQUE);
    }
};


