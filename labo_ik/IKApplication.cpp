#include "IKApplication.h"
#include "IKGLCanvas.h"

#include <nanogui/window.h>
#include <nanogui/formhelper.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/button.h>
#include <nanogui/toolbutton.h>
#include <nanogui/popupbutton.h>
#include <nanogui/combobox.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/graph.h>
#include <nanogui/tabwidget.h>
#include <GLFW/glfw3.h>
#include <memory>

#define _USE_MATH_DEFINES
#include <cmath>

#include "IKSolver.h"

using namespace nanogui;

namespace
{
    // Random number generator.
    //  Returns a random value such that _min <= val <= _max
    //
    template<typename _Scalar>
    static inline _Scalar rand(_Scalar _min, _Scalar _max)
    {
        static std::random_device rdev;
        static std::default_random_engine re(rdev());

        // Using uniform distribution (recommended since C++11)
        typedef typename std::conditional<
            std::is_floating_point<_Scalar>::value,
            std::uniform_real_distribution<_Scalar>,
            std::uniform_int_distribution<_Scalar>>::type dist_type;
        dist_type uni(_min, _max);
        return static_cast<_Scalar>(uni(re));
    }

}

IKApplication::IKApplication() : Screen(Vector2i(1280, 720), "GTI320 Labo sur la cinematique inverse"),
m_targetPos()
{
    m_armature = std::make_unique<gti320::Armature>();
    m_ikSolver = std::make_unique<gti320::IKSolver>(m_armature.get(), m_targetPos);

    initializeTarget();
    initializeArmature();

    m_window = new Window(this, "Cinematique inverse");
    m_window->set_position(Vector2i(8, 8));
    m_window->set_layout(new GroupLayout());

    m_canvas = std::make_unique<IKGLCanvas>(this);
    m_canvas->set_background_color({ 255, 255, 255, 255 });
    m_canvas->set_size({ 1080, 600 });

    Window* controls = new Window(this, "Controls");
    controls->set_position(Vector2i(1020, 10));
    controls->set_layout(new GroupLayout());

    Widget* tools = new Widget(controls);
    tools->set_layout(new BoxLayout(Orientation::Vertical, Alignment::Middle, 0, 5));

    m_targetUI = std::make_unique<TargetUI>(tools, m_targetPos);

    const int numLinks = m_armature->links.size();
    for (int i = 0; i < numLinks; ++i)
    {
        gti320::Link* link = m_armature->links[i];
        if (!link->isEndEffector())
        {
            m_linkUIArr.push_back(new LinkUI(m_armature->links[i], i, tools));
        }
    }

    Button* solveButton = new Button(tools, "IK solve");
    solveButton->set_callback([this]
        {
            ikSolve();
        });

    Button* resetButton = new Button(tools, "Reset");
    resetButton->set_callback([this]
        {
            reset();
        });

    perform_layout();
    reset();
}

bool IKApplication::keyboard_event(int key, int scancode, int action, int modifiers)
{
    if (Screen::keyboard_event(key, scancode, action, modifiers))
        return true;
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        set_visible(false);
        return true;
    }
    return false;
}

void IKApplication::draw(NVGcontext* ctx)
{
    assert(m_armature->root != nullptr);

    m_armature->updateKinematics();

    // Draw the user interface
    Screen::draw(ctx);
}

void IKApplication::reset()
{
    // Reset all joints to zero angle
    //
    for (gti320::Link* l : m_armature->links)
    {
        l->eulerAng.setZero();
    }

    // Update the armature
    //
    m_armature->updateKinematics();

    // Update UI
    //
    for (LinkUI* ui : m_linkUIArr)
    {
        ui->onArmatureChanged();
    }
}

void IKApplication::ikSolve()
{
    m_ikSolver->solve();
    for (LinkUI* ui : m_linkUIArr)
    {
        ui->onArmatureChanged();
    }
}

void IKApplication::initializeArmature()
{
    // Initialize the armature
    //
    gti320::Vector3f angs;
    angs.setZero();
    gti320::Vector3f t;

    // Root
    t.setZero();
    gti320::Link* link0 = new gti320::Link(nullptr, angs, t);

    // First joint
    t(1) = 1.5f;
    gti320::Link* link1 = new gti320::Link(link0, angs, t);

    // Second joint
    t(1) = 0.75f;
    gti320::Link* link2 = new gti320::Link(link1, angs, t);

    // End-effector
    t(1) = 0.25f;
    gti320::Link* link3 = new gti320::Link(link2, angs, t);

    m_armature->links.push_back(link0);
    m_armature->links.push_back(link1);
    m_armature->links.push_back(link2);
    m_armature->links.push_back(link3);
    m_armature->root = link0;
}

void IKApplication::initializeTarget()
{
    m_targetPos(0) = 1.0f;
    m_targetPos(1) = 1.0f;
    m_targetPos(2) = 0.0f;
}
