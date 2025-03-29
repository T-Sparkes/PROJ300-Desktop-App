#pragma once

class UIwindow
{
public:
    UIwindow() = default;
    ~UIwindow() = default;
    virtual void OnUpdate() = 0;
};
