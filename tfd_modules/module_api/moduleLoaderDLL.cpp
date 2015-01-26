#include "moduleLoaderDLL.h"
#include <assert.h>

ModuleLoaderDLL::ModuleLoaderDLL()
{

}

ModuleLoaderDLL::~ModuleLoaderDLL()
{
   // _openLibs should have been filled in getFunction
   // now we might want to close them
}

bool startsWith(const std::string & str, const std::string substr)
{
   return (str.find(substr) == 0);
}

bool endsWith(const std::string & str, const std::string substr)
{
   size_t pos = str.rfind(substr);
   if(pos == std::string::npos) // doesnt even contain it
      return false;

   size_t len = str.length();
   size_t elen = substr.length();
   // at end means: Pos found + length of end equal length of full string.
   if( pos + elen == len ) {
      return true;
   }

   // not at end
   return false;
}

/**
 * Usually unix libs are libNAME.so and in windows we would have NAME.dll
 * So: strip .so and preceeding lib.
 */
string ModuleLoaderDLL::soToDll(const string & libname)
{
   if(!endsWith(libname, ".so"))    // not .so - something custom or even .dll -> do nothing
     return libname;
   // it ends with ".so" -> replace it
   string res = libname.substr(0, libname.length() - 2);
   res += "dll";
   if(startsWith(res, "lib")) {
      res = res.substr(3);
   }

   return res;
}

void* ModuleLoaderDLL::getFunction(string fnString)
{
   string libName = extractLibName(fnString);
   libName = soToDll(libName);
   if(libName.empty())
      return NULL;
   void* libHandle = NULL;
   if(_openLibs.find(libName) != _openLibs.end()) {
      libHandle = _openLibs[libName];
   } else { // need to open the lib
      // not implemented yet: open lib
      assert(false);
      _openLibs[libName] = libHandle;
   }
 
   string fnName = extractFunctionName(fnString);
   if(fnName.empty())
      return NULL;

   // not implemented yet: retrieve function pointer
   assert(false);

   return NULL;
}

