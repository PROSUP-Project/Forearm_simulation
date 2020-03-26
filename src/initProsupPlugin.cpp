/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include "ProsupPlugin.h"

#include <sofa/helper/system/config.h>
namespace sofa
{

namespace component
{

	//Here are just several convenient functions to help user to know what contains the plugin

	extern "C" {
                SOFA_PROSUP_API  void initExternalModule();
                SOFA_PROSUP_API  const char* getModuleName();
                SOFA_PROSUP_API  const char* getModuleVersion();
                SOFA_PROSUP_API  const char* getModuleLicense();
                SOFA_PROSUP_API  const char* getModuleDescription();
                SOFA_PROSUP_API  const char* getModuleComponentList();
	}
	
	void initExternalModule()
	{
		static bool first = true;
		if (first)
		{
			first = false;
		}
	}

	const char* getModuleName()
	{
                 return "PROSUP";
	}

	const char* getModuleVersion()
	{
                return "0.0";
	}

	const char* getModuleLicense()
	{
		return "LGPL";
	}


	const char* getModuleDescription()
	{
                return "a simple example of a plugin component module for forearme pro/supnation simulation";
	}

	const char* getModuleComponentList()
	{
                return "Not defined yet";
	}

} 


} 
SOFA_PROSUP_API
//SOFA_LINK_CLASS(sofa::helper::io::MeshProcessing)
/* To be linked later Noura
SOFA_LINK_CLASS(SimulationPlanningController)
SOFA_LINK_CLASS(PlanningStateMonitor)
SOFA_LINK_CLASS(LinearMotionController)
SOFA_LINK_CLASS(SteerableNeedleContact)
SOFA_LINK_CLASS(SteerableNeedleConstraints)
SOFA_LINK_CLASS(SteerableNeedleSurfaceConstraint)
SOFA_LINK_CLASS(SteerableNeedleTissueParameters)
SOFA_LINK_CLASS(SteerableNeedleContainer)
*/
