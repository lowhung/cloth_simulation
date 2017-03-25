#include "App.h"

const std::string gWinTitle = "CPSC 426 Assignment 4";
int gWinWidth = 800;
int gWinHeight = 450;

int main(int argc, char ** argv) 
{
	try {
		nanogui::init();
		{
			nanogui::ref<cApp> app = new cApp(gWinWidth, gWinHeight, gWinTitle);
			app->Init();

			app->drawAll();
			app->setVisible(true);

			int mill_per_frame = static_cast<int>(1000 / app->GetFPS());
			nanogui::mainloop(mill_per_frame);
		}
		nanogui::shutdown();
	}
	catch (const std::runtime_error &e)
	{
		std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
		std::cerr << error_msg << std::endl;
		return -1;
	}

	return 0;
}
