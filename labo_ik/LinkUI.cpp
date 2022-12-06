#include "LinkUI.h"

LinkUI::LinkUI(gti320::Link* _link, int _id, nanogui::Widget* _parent) : link(_link), id(_id)
{
    panel = new nanogui::Widget(_parent);
    panel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Middle, 0, 5));

    init();
}

void LinkUI::init()
{
    static char buf[64];
    static const auto rotateMinMax = std::make_pair<float, float>(-1.57f, 1.57f);
    static const std::string baseName = "link ";

    // Rotate X
    panelRotateX = new nanogui::Widget(panel);
    panelRotateX->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 20));
    labelRotateX = new nanogui::Label(panelRotateX, baseName + itoa(id, buf, 10) + " (X): ");
    sliderRotateX = new nanogui::Slider(panelRotateX);
    sliderRotateX->set_range(rotateMinMax);
    textboxRotateX = new nanogui::TextBox(panelRotateX);
    sliderRotateX->set_callback([this](float value)
        {
            link->eulerAng(0) = value;
            onSlidersChanged();
        });

    // Rotate Y
    panelRotateY = new nanogui::Widget(panel);
    panelRotateY->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 20));
    labelRotateY = new nanogui::Label(panelRotateY, baseName + itoa(id, buf, 10) + " (Y): ");
    sliderRotateY = new nanogui::Slider(panelRotateY);
    sliderRotateY->set_range(rotateMinMax);
    textboxRotateY = new nanogui::TextBox(panelRotateY);
    sliderRotateY->set_callback([this](float value)
        {
            link->eulerAng(1) = value;
            onSlidersChanged();
        });

    // Rotate Z
    panelRotateZ = new nanogui::Widget(panel);
    panelRotateZ->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 20));
    labelRotateZ = new nanogui::Label(panelRotateZ, baseName + itoa(id, buf, 10) + " (Z): ");
    sliderRotateZ = new nanogui::Slider(panelRotateZ);
    sliderRotateZ->set_range(rotateMinMax);
    textboxRotateZ = new nanogui::TextBox(panelRotateZ);
    sliderRotateZ->set_callback([this](float value)
        {
            link->eulerAng(2) = value;
            onSlidersChanged();
        });

    onSlidersChanged();
}

void LinkUI::onSlidersChanged()
{
    static char buf[64];
    snprintf(buf, sizeof(buf), "%3.3f", link->eulerAng(0));
    textboxRotateX->set_value(buf);
    snprintf(buf, sizeof(buf), "%3.3f", link->eulerAng(1));
    textboxRotateY->set_value(buf);
    snprintf(buf, sizeof(buf), "%3.3f", link->eulerAng(2));
    textboxRotateZ->set_value(buf);
}

void LinkUI::onArmatureChanged()
{
    sliderRotateX->set_value(link->eulerAng(0));
    sliderRotateY->set_value(link->eulerAng(1));
    sliderRotateZ->set_value(link->eulerAng(2));

    onSlidersChanged();
}
