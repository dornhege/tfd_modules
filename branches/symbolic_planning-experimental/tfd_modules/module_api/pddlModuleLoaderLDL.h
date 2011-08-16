#ifndef P_D_D_L_MODULE_LOADER_L_D_L_H
#define P_D_D_L_MODULE_LOADER_L_D_L_H

#include "pddlModuleLoader.h"
#include "moduleLoaderLDL.h"

//FIXME: probably better as singleton

class PDDLModuleLoaderLDL : public ModuleLoaderLDL, public PDDLModuleLoader
{
   public:
      PDDLModuleLoaderLDL();
      ~PDDLModuleLoaderLDL();

};

#endif

