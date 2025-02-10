#pragma once

#include "ViewPort.hpp"

class ViewPortRenderable
{
private:
ViewPort* m_targetViewPort;    

public:
    ViewPortRenderable(ViewPort* targetViewPort)
    {
        m_targetViewPort = targetViewPort;
        m_targetViewPort->AddRenderable(this);
    }

    ~ViewPortRenderable()
    {
        m_targetViewPort->RemoveRenderable(this);
    }

    virtual void render() = 0;
};

