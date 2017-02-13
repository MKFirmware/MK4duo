/**
 * Configuration_Overall_Pins.h
 * Here you can define all your custom settings and they will overwrite configurations in the main configuration files.
 */



#ifdef MULTI_CONFIGURATION_OVERALL

	#ifdef DRAKE_KOSSEL_MINI
		 #include "src/printer/Conf_Drake_Kossel_Mini_Pins.h"
	#endif

	#ifdef ADELCHI_KOSSEL_MINI
		 #include "src/printer/Conf_Adechi_Kossel_Mini_Pins.h"
	#endif

	#ifdef FABLAB_MATERIA_101
		 #include "src/printer/Conf_Materia101_Pins.h"
	#endif

   	#ifdef FABLAB_MAKERBOT
		 #include "src/printer/Overall_Fablab_Makerbot_Pins.h"
	#endif
#endif
