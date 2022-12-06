/**
 * @file main.cpp  
 *
 * @brief GTI320 Labo 2 - creates a NanoGUI application.
 *
 * Nom: Mathias Glorieux
 * Code permanent : AR53050
 * Email : mathias.glorieux.1@ens.etsmtl.ca
 *
 */

#include "IKApplication.h"

#if defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
#if defined(_WIN32)
#  pragma warning(push)
#  pragma warning(disable: 4457 4456 4005 4312)
#endif

#if defined(_WIN32)
#  pragma warning(pop)
#endif
#if defined(_WIN32)
#  if defined(APIENTRY)
#    undef APIENTRY
#  endif
#  include <windows.h>
#endif

int main(int argc, char** argv) 
{
	nanogui::init();
	nanogui::ref<IKApplication> app = new IKApplication();
	app->draw_all();
	app->set_visible(true);
	nanogui::mainloop();
	nanogui::shutdown();

	return 0;
}