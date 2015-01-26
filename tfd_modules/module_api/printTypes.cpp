#include "tfd_modules/module_api/pddlModuleTypes.h"
#include "tfd_modules/module_api/printTypes.h"
#include <iomanip>
#include <iostream>

#include <deque>
using std::deque;
namespace modules {

std::ostream & operator<<(std::ostream & os, const Parameter & p)
{
   if(p.value.empty()) {
      os << "?" << p.name << " - " << p.type;
   } else {
      os << "?" << p.name << " - " << p.type << " : " << p.value;
   }
   return os;
}

std::ostream & operator<<(std::ostream & os, const ParameterList & pl)
{
   bool first = true;
   for(ParameterList::const_iterator it = pl.begin(); it != pl.end(); it++) {
      if(first) {
         os << *it;
         first = false;
      } else {
         os << ", " << *it;
      }
   }
   return os;
}

std::ostream & operator<<(std::ostream & os, const Predicate & p)
{
   os << p.name << "(";
   os << p.parameters;
   os << ") : ";
   os << p.value;
   return os;
}

int pcomp(const Predicate & n1, const Predicate & n2) 
{
   bool res = false;
   if(n1.parameters.size() != n2.parameters.size())
      return n1.parameters.size() > n2.parameters.size();
   if(n1.parameters.size() == 1 && n2.parameters.size() == 1) {
      if(n1.parameters.front().value.compare(n2.parameters.front().value) != 0) {
         res = n1.parameters.front().value < n2.parameters.front().value;
         return res;
      }
   }
   if(n1.name.length() != n2.name.length())
      return n1.name.length() < n2.name.length();
   res = n1.name < n2.name;
   return res;
}

std::ostream & operator<<(std::ostream & os, const PredicateList & nl_)
{
   deque<Predicate> dn;
   for(PredicateList::const_iterator it = nl_.begin(); it != nl_.end(); it++) {
      dn.push_back(*it);
   }
   sort(dn.begin(), dn.end(), pcomp);
   PredicateList pl;
   for(deque<Predicate>::iterator it = dn.begin(); it != dn.end(); it++) {
      pl.push_back(*it);
   }

   bool first = true;
   for(PredicateList::const_iterator it = pl.begin(); it != pl.end(); it++) {
      if(PredicateList_OStreamMode::s_Condensed) {
         if(!it->value)    // only include valid ones
            continue;
         os << it->name << "(";
         bool pfirst = true;
         for(ParameterList::const_iterator itp = it->parameters.begin(); itp != it->parameters.end(); itp++) {
            if(!pfirst)
               os << ", " << itp->value;
            else {
               os << itp->value;
               pfirst = false;
            }
         }
         os << ") ";
      } else {
         os << *it;
         first = false;
         os << std::endl;
      }
   }
   return os;
}

std::ostream & operator<<(std::ostream & os, const NumericalFluent & n)
{
   os << n.name << "(";
   os << n.parameters;
   os << ") : ";
   os << n.value;
   return os;
}

int nfcomp(const NumericalFluent & n1, const NumericalFluent & n2) 
{
   bool res = false;
   if(n1.parameters.size() != n2.parameters.size())
      return n1.parameters.size() > n2.parameters.size();
   if(n1.parameters.size() == 1 && n2.parameters.size() == 1) {
      if(n1.parameters.front().value.compare(n2.parameters.front().value) != 0) {
         res = n1.parameters.front().value < n2.parameters.front().value;
         return res;
      }
   }
   if(n1.name.length() != n2.name.length())
      return n1.name.length() < n2.name.length();
   res = n1.name < n2.name;
   return res;
}

std::ostream & operator<<(std::ostream & os, const NumericalFluentList & nl_)
{
   deque<NumericalFluent> dn;
   for(NumericalFluentList::const_iterator it = nl_.begin(); it != nl_.end(); it++) {
      dn.push_back(*it);
   }
   sort(dn.begin(), dn.end(), nfcomp);
   NumericalFluentList nl;
   for(deque<NumericalFluent>::iterator it = dn.begin(); it != dn.end(); it++) {
      nl.push_back(*it);
   }
   bool first = true;
   int count = 0;
   for(NumericalFluentList::const_iterator it = nl.begin(); it != nl.end(); it++) {
      if(NumericalFluentList_OStreamMode::s_Condensed) {
         std::ios_base::fmtflags flagsBak = os.flags();
         os << std::fixed << std::setprecision(2);
         os << it->name << "(";
         bool pfirst = true;
         for(ParameterList::const_iterator itp = it->parameters.begin(); itp != it->parameters.end(); itp++) {
            if(!pfirst)
               os << ", " << itp->value;
            else {
               os << itp->value;
               pfirst = false;
            }
         }
         os << ") = " << it->value << "   ";
         count++;
         if(count >= 4) {
            count = 0;
            os << std::endl;
         }
         os.setf(flagsBak);
      } else {
         if(first) {
            os << *it;
            first = false;
         } else {
            os << ", " << *it;
         }
         os<<std::endl;
      }
   }
   return os;
}

bool PredicateList_OStreamMode::s_Condensed = false;
bool NumericalFluentList_OStreamMode::s_Condensed = false;

} // namespace modules

