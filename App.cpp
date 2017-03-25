#include "App.h"
#include <memory>
#include <nanogui/label.h>
#include <nanogui/layout.h>

#include "scenarios/ClothScenario.h"

const double gFPS = 60;
const double gMaxWindStrength = 50;

cApp::cApp(int w, int h, const std::string& title) : nanogui::Screen(Eigen::Vector2i(w, h), title)
{
	mGUIWindow = nullptr;
	mPlayButton = nullptr;
	mWindSlider = nullptr;
	mPrevTime = 0;
	mEnableAnimation = true;
}

cApp::~cApp() 
{
	mScenario.reset();
}

void cApp::Init()
{
	cDrawUtil::InitDrawUtil();
	BuildScenario();
	BuildGUI();
}

bool cApp::keyboardEvent(int key, int scancode, int action, int modifiers) {
	if (Screen::keyboardEvent(key, scancode, action, modifiers))
		return true;
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		setVisible(false);
		return true;
	}
	return false;
}

bool cApp::mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers)
{
	if (nanogui::Screen::mouseButtonEvent(p, button, down, modifiers))
	{
		return true;
	}
	if (mScenario != nullptr)
	{
		return mScenario->MouseButtonEvent(p, button, down, modifiers);
	}
	return false;
}

bool cApp::mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers)
{
	if (nanogui::Screen::mouseMotionEvent(p, rel, button, modifiers))
	{
		return true;
	}
	if (mScenario != nullptr)
	{
		return mScenario->MouseMotionEvent(p, rel, button, modifiers);
	}
	return false;
}

bool cApp::scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel)
{
	if (nanogui::Screen::scrollEvent(p, rel))
	{
		return true;
	}
	if (mScenario != nullptr)
	{
		return mScenario->ScrollEvent(p, rel);
	}
	return false;
}

void cApp::draw(NVGcontext *ctx)
{
	//Draw the user interface
	Screen::draw(ctx);
}

void cApp::drawContents()
{
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	Update();
	DrawScenario();
}

bool cApp::resizeEvent(const Eigen::Vector2i& size)
{
	bool val = nanogui::Screen::resizeEvent(size);
	if (mScenario != nullptr)
	{
		mScenario->Resize(size);
	}
	return val;
}

double cApp::GetFPS() const
{
	return gFPS;
}

void cApp::BuildScenario()
{
	mScenario = std::unique_ptr<cScenario>(new cClothScenario());
	mScenario->Resize(mSize);
	mScenario->Init();
}

void cApp::UpdateScenario(double time)
{
	double time_elapsed = time - mPrevTime;
	double time_step = 1 / GetFPS();
	time_elapsed = std::min(time_elapsed, time_step);
	StepScenario(time_elapsed);
	mPrevTime = time;
}

void cApp::StepScenario(double time_elapsed)
{
	if (mScenario != nullptr)
	{
		mScenario->Update(time_elapsed);
	}
}

cClothScenario* cApp::GetClothScene()
{
	cClothScenario* cloth_scene = reinterpret_cast<cClothScenario*>(mScenario.get());
	return cloth_scene;
}


void cApp::Update()
{
	if (EnableAnimation())
	{
		// Advances animation
		UpdateScenario(glfwGetTime());
	}
	UpdateGUI();
}

void cApp::DrawScenario()
{
	if (mScenario != nullptr)
	{
		mScenario->Draw();
	}
}

bool cApp::EnableAnimation() const
{
	bool enable = mEnableAnimation;
	if (mPlayButton != nullptr)
	{
		enable &= mPlayButton->pushed();
	}
	return enable;
}

void cApp::ClearGUI()
{
	if (mGUIWindow != nullptr)
	{
		removeChild(mGUIWindow);
	}
}

void cApp::BuildGUI()
{
	ClearGUI();

	auto cloth_scene = GetClothScene();

	// Build GUI panel that will contain the interface
	mGUIWindow = new nanogui::Window(this, "Scene Control");
	mGUIWindow->setPosition(nanogui::Vector2i(0, 0));
	mGUIWindow->setLayout(new nanogui::GroupLayout());

	// FPS textbox
	mFPSLabel = new nanogui::Label(mGUIWindow, "FPS:  0", "sans-bold");
	mFPSLabel->setFontSize(18);

	// Add combo box for interators
	auto lab = new nanogui::Label(mGUIWindow, "Integrator", "sans-bold");
	mIntegratorCombo = new nanogui::ComboBox(mGUIWindow, { "Explicit", "Midpoint", "Trapezoid", "Implicit" });
	tComboCallback int_combo_callback = std::bind(&cApp::IntegratorComboCallback, this, std::placeholders::_1);
	mIntegratorCombo->setCallback(int_combo_callback);
	mIntegratorCombo->setSelectedIndex(cloth_scene->GetIntegrator());

	// Add panel for stepsize control
	new nanogui::Label(mGUIWindow, "Stepsize", "sans-bold");
	Widget* stepsize_panel = new Widget(mGUIWindow);
	stepsize_panel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
						nanogui::Alignment::Middle, 0, 6));

	// Decrement
	auto stepsize_dec = new nanogui::Button(stepsize_panel, "", ENTYPO_ICON_DOWN);
	stepsize_dec->setCallback(std::bind(&cApp::DecSimStepsize, this));

	mStepSizeTextBox = new nanogui::TextBox(stepsize_panel);
	mStepSizeTextBox->setFixedSize(Eigen::Vector2i(70, 30));
	SetSimStepsize(cloth_scene->GetStepsize());

	// Increment
	auto stepsize_inc = new nanogui::Button(stepsize_panel, "", ENTYPO_ICON_UP);
	stepsize_inc->setCallback(std::bind(&cApp::IncSimStepsize, this));


	// Add panel for stiffness control
	new nanogui::Label(mGUIWindow, "Stiffness", "sans-bold");
	Widget* stiffness_panel = new Widget(mGUIWindow);
	stiffness_panel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
							nanogui::Alignment::Middle, 0, 6));

	// Decrement
	auto stiffness_dec = new nanogui::Button(stiffness_panel, "", ENTYPO_ICON_DOWN);
	stiffness_dec->setCallback(std::bind(&cApp::DecClothStiffness, this));

	mStiffnessTextBox = new nanogui::TextBox(stiffness_panel);
	mStiffnessTextBox->setFixedSize(Eigen::Vector2i(70, 30));
	SetClothStiffness(cloth_scene->GetStiffness());

	// Increment
	auto stiffness_inc = new nanogui::Button(stiffness_panel, "", ENTYPO_ICON_UP);
	stiffness_inc->setCallback(std::bind(&cApp::IncClothStiffness, this));


	// Slider to control the strength of the wind
	new nanogui::Label(mGUIWindow, "Wind Strength", "sans-bold");
	mWindSlider = new nanogui::Slider(mGUIWindow);
	mWindSlider->setCallback(std::bind(&cApp::WindSliderCallback, this, std::placeholders::_1));

	double wind_strength = cloth_scene->GetWindStrength();
	mWindSlider->setValue(static_cast<float>(wind_strength / gMaxWindStrength));


	// Add panel for playback control
	new nanogui::Label(mGUIWindow, "Playback", "sans-bold");
	Widget* playback = new Widget(mGUIWindow);
	playback->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
						nanogui::Alignment::Middle, 0, 6));

	// Reload scene button
	auto reload = new nanogui::Button(playback, "", ENTYPO_ICON_CCW);
	reload->setCallback(std::bind(&cApp::Reload, this));
	
	// Play/Pause button
	mPlayButton = new nanogui::Button(playback, "", ENTYPO_ICON_PAUS);
	mPlayButton->setFlags(nanogui::Button::ToggleButton);
	mPlayButton->setPushed(true);
	mPlayButton->setChangeCallback(std::bind(&cApp::TogglePlayCallback, this, std::placeholders::_1));

	// Step forward button
	auto step_forward = new nanogui::Button(playback, "", ENTYPO_ICON_FF);
	step_forward->setCallback(std::bind(&cApp::StepForwardCallback, this));

	// After all GUI has been built, call refresh to reorganize everything
	RefreshGUI();
}

void cApp::UpdateGUI()
{
	char str_buffer[32];
	auto cloth_scene = GetClothScene();
	sprintf(str_buffer, "FPS:  %.3f", cloth_scene->GetFPS());
	mFPSLabel->setCaption(std::string(str_buffer));
}

void cApp::RefreshGUI()
{
	performLayout();
}

void cApp::StepForwardCallback()
{
	mPlayButton->setPushed(false);
	TogglePlayCallback(false);
	StepScenario(1 / GetFPS());
}

void cApp::TogglePlayCallback(bool pushed)
{
	if (pushed)
	{
		mPlayButton->setIcon(ENTYPO_ICON_PAUS);
	}
	else
	{
		mPlayButton->setIcon(ENTYPO_ICON_PLAY);
	}
}

void cApp::WindSliderCallback(double val)
{
	auto cloth_scene = GetClothScene();
	cloth_scene->SetWindStrength(val * gMaxWindStrength);
}

void cApp::IntegratorComboCallback(int i)
{
	auto cloth_scene = GetClothScene();
	cloth_scene->SetIntegrator(static_cast<cCloth::eIntegrator>(i));
	cloth_scene->Reset();
}

void cApp::DecSimStepsize()
{
	auto cloth_scene = GetClothScene();
	SetSimStepsize(cloth_scene->GetStepsize() - 0.001);
}

void cApp::IncSimStepsize()
{
	auto cloth_scene = GetClothScene();
	SetSimStepsize(cloth_scene->GetStepsize() + 0.001);
}

void cApp::SetSimStepsize(double stepsize)
{
	auto cloth_scene = GetClothScene();
	cloth_scene->SetStepsize(std::max(0.0, stepsize));

	char str_buffer[32];
	sprintf(str_buffer, "%.3f", cloth_scene->GetStepsize());
	mStepSizeTextBox->setValue(std::string(str_buffer));
}

void cApp::DecClothStiffness()
{
	auto cloth_scene = GetClothScene();
	SetClothStiffness(cloth_scene->GetStiffness() - 1);
}

void cApp::IncClothStiffness()
{
	auto cloth_scene = GetClothScene();
	SetClothStiffness(cloth_scene->GetStiffness() + 1);
}

void cApp::SetClothStiffness(double stiffness)
{
	auto cloth_scene = GetClothScene();
	cloth_scene->SetStiffness(std::max(0.0, stiffness));

	char str_buffer[32];
	sprintf(str_buffer, "%.0f", cloth_scene->GetStiffness());
	mStiffnessTextBox->setValue(std::string(str_buffer));
}

void cApp::Reload()
{
	mScenario->Reset();
}

void cApp::BuildShortFileNames(const std::vector<std::string>& files, std::vector<std::string>& out_names) const
{
	// shorten the filepaths for display in the GUI
	out_names.clear();
	std::vector<std::string> short_names;
	for (size_t f = 0; f < files.size(); ++f)
	{
		const std::string& curr_file = files[f];

		int idx = 0;
		for (int i = static_cast<int>(curr_file.size()) - 1; i >= 0; --i)
		{
			char curr_char = curr_file[i];
			if (curr_char == '\\' || curr_char == '/')
			{
				idx = i + 1;
				break;
			}
		}

		std::string filename = curr_file.substr(idx, curr_file.size() - idx);
		out_names.push_back(filename);
	}
}