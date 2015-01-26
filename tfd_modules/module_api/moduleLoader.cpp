#include "moduleLoader.h"

ModuleLoader::ModuleLoader()
{

}

ModuleLoader::~ModuleLoader()
{

}

string ModuleLoader::extractFunctionName(string fnString)
{
   size_t pos = fnString.find("@");
   if(pos == string::npos)
      return "";
   return fnString.substr(0, pos);
}

string ModuleLoader::extractLibName(string fnString)
{
   size_t pos = fnString.find("@");
   if(pos == string::npos)
      return "";
   return fnString.substr(pos + 1);
}

