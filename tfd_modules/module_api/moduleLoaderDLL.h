#ifndef MODULE_LOADER_D_L_L_H
#define MODULE_LOADER_D_L_L_H

#include "moduleLoader.h"

/// A module loader implementation for loading windows DLL files.
//UNFINISHED!
class ModuleLoaderDLL : virtual public ModuleLoader
{
   public:
      ModuleLoaderDLL();
      ~ModuleLoaderDLL();

      virtual void* getFunction(string fnString);

   protected:
      /// If a libname is given in unix style convert it to the matching windows dll name.
      string soToDll(const string & libname);

};

#endif

