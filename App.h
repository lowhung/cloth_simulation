#pragma once
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/button.h>
#include <nanogui/combobox.h>
#include <nanogui/slider.h>
#include <nanogui/textbox.h>

#include <iostream>
#include <string>
#include <memory>
#include <functional>

#include "scenarios/Scenario.h"

class cClothScenario;

class cApp : public nanogui::Screen {
public:
	
	cApp(int w, int h, const std::string& title);
	virtual ~cApp();

	virtual void Init();

	virtual bool keyboardEvent(int key, int scancode, int action, int modifiers);
	virtual bool mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers);
	virtual bool mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers);
	virtual bool scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel);

	virtual void draw(NVGcontext *ctx);
	virtual void drawContents();
	
	/// Set window size
	virtual bool resizeEvent(const Eigen::Vector2i& size);

	virtual double GetFPS() const;

protected:
	
	typedef std::function<void(int)> tComboCallback;
	typedef std::function<void()> tButtonCallback;
	typedef std::function<void(bool)> tToggleCallback;
	typedef std::function<void(double)> tSliderCallback;

	std::unique_ptr<cScenario> mScenario;
	nanogui::Window* mGUIWindow;
	nanogui::ComboBox* mIntegratorCombo;
	nanogui::Button* mPlayButton;
	nanogui::Slider* mWindSlider;
	nanogui::TextBox* mStepSizeTextBox;
	nanogui::TextBox* mStiffnessTextBox;
	nanogui::Label* mFPSLabel;

	double mPrevTime;
	bool mEnableAnimation;

	virtual void BuildScenario();
	virtual void UpdateScenario(double time);
	virtual void StepScenario(double time_elapsed);
	virtual cClothScenario* GetClothScene();

	virtual void Update();
	virtual void DrawScenario();

	virtual bool EnableAnimation() const;

	virtual void ClearGUI();
	virtual void BuildGUI();
	virtual void UpdateGUI();
	virtual void RefreshGUI();

	virtual void StepForwardCallback();
	virtual void TogglePlayCallback(bool pushed);
	virtual void WindSliderCallback(double val);
	virtual void IntegratorComboCallback(int i);
	virtual void DecSimStepsize();
	virtual void IncSimStepsize();
	virtual void SetSimStepsize(double stepsize);
	virtual void DecClothStiffness();
	virtual void IncClothStiffness();
	virtual void SetClothStiffness(double stiffness);

	virtual void Reload();

	virtual void BuildShortFileNames(const std::vector<std::string>& files, std::vector<std::string>& out_names) const;
};