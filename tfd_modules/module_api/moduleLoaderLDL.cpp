#include "moduleLoaderLDL.h"
#include <iostream>
#include <dlfcn.h>
#include <stdio.h>


ModuleLoaderLDL::ModuleLoaderLDL()
{

}

ModuleLoaderLDL::~ModuleLoaderLDL()
{
   for(map<string, void*>::iterator it = _openLibs.begin(); it != _openLibs.end(); it++) {
      dlclose(it->second);
   }
}

void* ModuleLoaderLDL::getFunction(string fnString)
{
   string libName = extractLibName(fnString);
   if(libName.empty())
      return NULL;
   void* libHandle = NULL;
   if(_openLibs.find(libName) != _openLibs.end()) {
      libHandle = _openLibs[libName];
   } else { // need to open the lib
      libHandle = dlopen(libName.c_str(), RTLD_NOW);
      if(libHandle == NULL) {
         fprintf(stderr, "Opening library %s failed.\n", libName.c_str());
         fputs(dlerror(), stderr);
         fprintf(stderr, "\n");
         return NULL;
      }
      _openLibs[libName] = libHandle;
   }
 
   string fnName = extractFunctionName(fnString);
   if(fnName.empty())
      return NULL;


   void* fn = dlsym(libHandle, fnName.c_str());

   char *error;
   if((error = dlerror()) != NULL)  {
      fputs(error, stderr);
      return NULL;
   }

   return fn;
}

