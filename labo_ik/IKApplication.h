#pragma once

/**
 * @file IKApplication.h
 *
 * @brief Application class for labo 2.
 *
 * Nom: Mathias Glorieux
 * Code permanent : AR53050
 * Email : mathias.glorieux.1@ens.etsmtl.ca
 *
 */

#include <nanogui/window.h>
#include <nanogui/textbox.h>
#include <nanogui/label.h>
#include <nanogui/layout.h>
#include <nanogui/slider.h>
#include <nanogui/screen.h>
#include <nanogui/vscrollpanel.h>

#include "Armature.h"
#include "LinkUI.h"
#include "TargetUI.h"
#include "IKSolver.h"
#include "Math3D.h"

class IKGLCanvas;

// Iterative closest point algorithm
//
class IKApplication : public nanogui::Screen
{
public:
	IKApplication();

    virtual bool keyboard_event(int key, int scancode, int action, int modifiers) override;
    
    virtual void draw(NVGcontext *ctx) override;

    nanogui::Window* getWindow() { return m_window.get(); }

    gti320::Armature* getArmature() const { return m_armature.get(); }

    const gti320::Vector3f& getTargetPos() const { return m_targetPos; }

private:

    // Reset the source transform and the initial transform.
    // 
    void reset();
    void ikSolve();
    void initializeArmature(); 
    void initializeTarget();

    std::unique_ptr<gti320::IKSolver> m_ikSolver;
    std::unique_ptr<IKGLCanvas> m_canvas;
    nanogui::ref<nanogui::Window> m_window;

    std::vector<LinkUI*> m_linkUIArr;
    std::unique_ptr<TargetUI> m_targetUI;
    std::unique_ptr<gti320::Armature> m_armature;
    gti320::Vector3f m_targetPos;
};