cmake_minimum_required(VERSION 2.8.12)
project(ProsupPlugin)


set(HEADER_FILES
	ProsupPlugin.h
	initProsupPlugin.h
	planning/MyComponent.h
	planning/LigamentTightener.h
	planning/meshprocessing.h
        planning/PScontactListener.h
	planning/StressStrainComponent.h

)

set(SOURCE_FILES
	initProsupPlugin.cpp
	planning/MyComponent.cpp
	planning/LigamentTightener.cpp
	planning/meshprocessing.cpp
	planning/PScontactListener.cpp
	planning/StressStrainComponent.cpp

)


set (README_FILES
	  ProsupPlugin.txt
)

### Subdirectories

set(SOFACOMMON_TARGETS
    SofaComponentCommon
    SofaGeneralLoader
)


include_directories("${CImg_DIR}")

find_package(SofaFramework REQUIRED COMPONENTS )
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${MOC_FILES} ${SOURCE_FILES} ${README_FILES}) 
target_link_libraries(${PROJECT_NAME} SofaCore SofaComponentBase SofaGeneralLoader SofaBaseVisual SofaOpenglVisual SofaBaseTopology SofaBaseMechanics SofaHelper SofaUserInteraction SofaValidation)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_PROSUPPLUGIN")


