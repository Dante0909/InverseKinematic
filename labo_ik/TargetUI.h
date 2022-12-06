#pragma once

/**
 * @file TargetUI.h
 *
 * @brief User interface for end-effector target.
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
#include "Math3D.h"


 // Target UI
 //
class TargetUI
{
public:

    TargetUI(nanogui::Widget* _parent, gti320::Vector3f& _target);

private:

    void init();

    void onSlidersChanged();

    nanogui::ref<nanogui::Widget> panel, panelTranslateX, panelTranslateY, panelTranslateZ;
    nanogui::ref<nanogui::TextBox> textboxTranslateX, textboxTranslateY, textboxTranslateZ;
    nanogui::ref<nanogui::Label> labelTranslateX, labelTranslateY, labelTranslateZ;
    nanogui::ref<nanogui::Slider> sliderTranslateX, sliderTranslateY, sliderTranslateZ;
    gti320::Vector3f& target;
};
