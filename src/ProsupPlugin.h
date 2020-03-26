#ifndef SOFA_PLUGIN_PROSUPPLUGIN_H
#define SOFA_PLUGIN_PROSUPPLUGIN_H

#ifndef WIN32
	#define SOFA_EXPORT_DYNAMIC_LIBRARY 
	#define SOFA_IMPORT_DYNAMIC_LIBRARY
    #define SOFA_PROSUP_API
#else
        #ifdef SOFA_BUILD_PROSUP
		#define SOFA_EXPORT_DYNAMIC_LIBRARY __declspec( dllexport )
                #define SOFA_PROSUP_API SOFA_EXPORT_DYNAMIC_LIBRARY
	#else
		#define SOFA_IMPORT_DYNAMIC_LIBRARY __declspec( dllimport )
                #define SOFA_PROSUP_API SOFA_IMPORT_DYNAMIC_LIBRARY
	#endif
#endif

#endif
