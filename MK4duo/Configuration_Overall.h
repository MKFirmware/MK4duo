/**
 * Configuration_Overall.h
 * Here you can define all your custom settings and they will overwrite configurations in the main configuration files.
 */



#define MULTI_CONFIGURATION_OVERALL

  //#define DRAKE_KOSSEL_MINI
 //#define ADELCHI_KOSSEL_MINI
 //#define FABLAB_MATERIA_101
 //#define DRAKE_MKS_TEST
#define FABLAB_MAKERBOT

#ifdef MULTI_CONFIGURATION_OVERALL

	#ifdef DRAKE_KOSSEL_MINI
		 #include "src/printer/Conf_Drake_Kossel_Mini_Overall.h"
	#endif

	#ifdef ADELCHI_KOSSEL_MINI
		 #include "src/printer/Conf_Adechi_Kossel_Mini_Overall.h"
	#endif

	#ifdef FABLAB_MATERIA_101
		 #include "src/printer/Conf_Materia101_Overall.h"
	#endif

    #ifdef FABLAB_MAKERBOT
		 #include "src/printer/Overall_Fablab_Makerbot_basic.h"
         #include "src/printer/Overall_Fablab_Makerbot_Cartesian.h"
         #include "src/printer/Overall_Fablab_Makerbot_Feature.h"
         #include "src/printer/Overall_Fablab_Makerbot_Temperature.h"
	#endif

#endif
