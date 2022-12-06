#pragma once

/**
 * @file IKGLCanvas.h
 *
 * @brief Canvas class for drawing OpenGL scenes.
 *
 * Nom:
 * Code permanent :
 * Email :
 *
 */

#include <nanogui/window.h>
#include <nanogui/canvas.h>
#include <nanogui/shader.h>

class IKApplication;

// An OpenGL canvas class to draw point clouds
//
class IKGLCanvas : public nanogui::Canvas
{
public:
    IKGLCanvas(IKApplication* _app);

    virtual void draw_contents() override;

    virtual bool mouse_drag_event(const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button, int modifiers) override;

    virtual bool scroll_event(const  nanogui::Vector2i& p, const  nanogui::Vector2f& rel) override;

    void resetCamera();

private:

    nanogui::ref<nanogui::Shader> m_shader;
    nanogui::ref<nanogui::Shader> m_lineShader;
    IKApplication* m_app;

    // Camera parameters
    float phi[2];
    float dist;

};
