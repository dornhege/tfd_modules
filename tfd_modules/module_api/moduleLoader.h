#ifndef MODULE_LOADER_H
#define MODULE_LOADER_H

#include <string>
#include <map>
using std::string;
using std::map;

/// Helper class to dynamically load named functions from libraries.
/**
 * Basically this class offers the functionality to pass a string as "checkcond@libtest.so"
 * returning the function pointer to "checkcond in libtest.so".
 */
class ModuleLoader
{
   public:
      ModuleLoader();
      ~ModuleLoader();

   public:
      virtual void* getFunction(string fnString) = 0;

   protected:
      string extractFunctionName(string fnString);
      string extractLibName(string fnString);

   protected:
      map<string, void*> _openLibs;  ///< contains the handles for open libraries
};

//make sure lib opened/closed once. -> evtl. close libs in descut
//hwoto general mit function pointers... (beliebige)

#endif

