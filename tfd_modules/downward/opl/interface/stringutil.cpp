#include "tfd_modules/opl/stringutil.h"
#include <algorithm>
#include <locale>
#include <stdio.h>
#include <iostream>

namespace StringUtil
{

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

   std::string toLower(const std::string & s)
   {
      std::string ret;
      std::transform(s.begin(), s.end(), back_inserter(ret), (int(*)(int)) std::tolower);
      return ret;
   }

   std::string toUpper(const std::string & s)
   {
      std::string ret;
      std::transform(s.begin(), s.end(), back_inserter(ret), (int(*)(int)) std::toupper);
      return ret;
   }

   std::string trim(const std::string & s)
   {
      if(s.length() == 0)
         return s;
      size_t b = s.find_first_not_of(" \t\r\n");
      size_t e = s.find_last_not_of(" \t\r\n");
      if(b == std::string::npos)
         return "";
      return std::string(s, b, e - b + 1);
   }

   std::vector<std::string> split(const std::string & s, const char* delim)
   {
      std::vector<std::string> elems;
      std::string::size_type lastPos = 0;
      std::string::size_type pos     = 0;

      do {
         pos = s.find_first_of(delim, lastPos);
         // ignore empty tokens from two consecutive delimiter
         // or a delimiter at the end of the string
         if (pos != lastPos && lastPos < s.length())
         {
             elems.push_back(s.substr(lastPos, pos - lastPos));
         }
         lastPos = pos + 1;
      } while(std::string::npos != pos);

      return elems;
   }

   std::string createFromNumber(int value)
   {
       char buffer[100];
       snprintf(buffer, 100, "%d", value);
       return std::string(buffer);
   }

   std::string createFromNumber(double value)
   {
       char buffer[100];
       snprintf(buffer, 100, "%f", value);
//       std::cout << buffer << std::endl;
//       printf(buffer);
       return std::string(buffer);
   }
};



