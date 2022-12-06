#include "TargetUI.h"

TargetUI::TargetUI(nanogui::Widget* _parent, gti320::Vector3f& _target) : target(_target)
{
	panel = new nanogui::Widget(_parent);
	panel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Middle, 0, 5));

	init();
}

void TargetUI::init()
{
	static const auto translateMinMax = std::make_pair<float, float>(-4.0f, 4.0f);

	// Translate X
	panelTranslateX = new nanogui::Widget(panel);
	panelTranslateX->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 20));
	labelTranslateX = new nanogui::Label(panelTranslateX, "Target X : ");
	sliderTranslateX = new nanogui::Slider(panelTranslateX);
	sliderTranslateX->set_range(translateMinMax);
	textboxTranslateX = new nanogui::TextBox(panelTranslateX);
	sliderTranslateX->set_callback([this](float value)
		{
			target(0) = value;
			onSlidersChanged();
		});

	// Rotate Y
	panelTranslateY = new nanogui::Widget(panel);
	panelTranslateY->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 20));
	labelTranslateY = new nanogui::Label(panelTranslateY, "Target Y : ");
	sliderTranslateY = new nanogui::Slider(panelTranslateY);
	sliderTranslateY->set_range(translateMinMax);
	textboxTranslateY = new nanogui::TextBox(panelTranslateY);
	sliderTranslateY->set_callback([this](float value)
		{
			target(1) = value;
			onSlidersChanged();
		});

	// Rotate Z
	panelTranslateZ = new nanogui::Widget(panel);
	panelTranslateZ->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 20));
	labelTranslateZ = new nanogui::Label(panelTranslateZ, "Target Z : ");
	sliderTranslateZ = new nanogui::Slider(panelTranslateZ);
	sliderTranslateZ->set_range(translateMinMax);
	textboxTranslateZ = new nanogui::TextBox(panelTranslateZ);
	sliderTranslateZ->set_callback([this](float value)
		{
			target(2) = value;
			onSlidersChanged();
		});

	onSlidersChanged();
}

void TargetUI::onSlidersChanged()
{
	static char buf[64];
	snprintf(buf, sizeof(buf), "%3.3f", target(0));
	textboxTranslateX->set_value(buf);
	snprintf(buf, sizeof(buf), "%3.3f", target(1));
	textboxTranslateY->set_value(buf);
	snprintf(buf, sizeof(buf), "%3.3f", target(2));
	textboxTranslateZ->set_value(buf);
}
