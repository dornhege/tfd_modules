#ifndef MODULE_LOADER_L_D_L_H
#define MODULE_LOADER_L_D_L_H

#include "moduleLoader.h"

/// A module loader implementation using libdl (dlopen/dlsym, etc.)
class ModuleLoaderLDL : virtual public ModuleLoader
{
   public:
      ModuleLoaderLDL();
      ~ModuleLoaderLDL();

      virtual void* getFunction(string fnString);
};

#endif

