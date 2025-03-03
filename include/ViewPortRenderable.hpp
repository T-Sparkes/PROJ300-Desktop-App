#pragma once

#include "ViewPort.hpp"

class ViewPortRenderable
{
public:
    ViewPortRenderable()
    {
        ViewPort::GetInstance().AddRenderable(this);
    }

    ~ViewPortRenderable()
    {
        ViewPort::GetInstance().RemoveRenderable(this);
    }

    virtual void render() = 0;
};

