#include "transport.h"
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <iostream>
#include <deque>
#include <set>
#include <math.h>
#include <algorithm>
#include <sys/time.h>
#include <ros/ros.h>

using namespace std;

struct quader {
  double sx;
  double sy;
  double sz;
  quader(double x, double y, double z) : sx(x), sy(y), sz(z) { }
};

#define DEBUG_MOD (0)

void packItem(deque<double> & items, quader q)
{
   // sort quader
   if(q.sx > q.sy) {
      double t = q.sx;
      q.sx = q.sy;
      q.sy = t;
   }
   if(q.sy > q.sz) {
      double t = q.sz;
      q.sz = q.sy;
      q.sy = t;
   }
   if(q.sx > q.sy) {
      double t = q.sx;
      q.sx = q.sy;
      q.sy = t;
   }

   double biggest = -1;
   for(deque<double>::iterator it = items.begin(); it != items.end(); it++) {
      if(*it > q.sx)
         continue;
      biggest = *it;
      items.erase(it);
      break;
   }
   if(biggest == -1)
      return;

   quader smallestQ(q.sx - biggest, biggest, biggest);
   quader mediumQ(q.sx, q.sy - biggest, biggest);
   quader largestQ(q.sx, q.sy, q.sz - biggest);

   packItem(items, smallestQ);
   packItem(items, mediumQ);
   packItem(items, largestQ);
}

bool checkLoading(deque<double> sizes, double cap)
{
  deque<double> itemSizes;
  double third = 1.0/3.0;
  quader full(pow(cap, third), pow(cap, third), pow(cap, third));
  for(deque<double>::iterator it = sizes.begin(); it != sizes.end(); it++) {
     itemSizes.push_back(pow(*it, third));
  }
  sort(itemSizes.begin(), itemSizes.end());
  reverse(itemSizes.begin(), itemSizes.end());
  packItem(itemSizes, full);
  return itemSizes.empty();
}

double can_load(const ParameterList & parameterList,
       predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int )
{
  ROS_DEBUG("Calling module %s", __func__);

  static int count = 0;
  count++;
  //cout << "Count: " << count << endl;
   if(DEBUG_MOD)
      cout << parameterList << endl;

  PredicateList* list = NULL;
  predicateCallback(list);

  if(list == NULL) {
     printf("WAAAAAHHH\n");
     exit(1);
     return false;
  }

  if(DEBUG_MOD) {
     cout << "PREDICATES ARE: " << endl;
     cout << *list << endl;
  }

  string me = parameterList.front().value;
  string pack = parameterList.back().value;

  deque<string> packs_in_truck;
  for(PredicateList::iterator it = list->begin(); it != list->end(); it++) {
      Predicate p = *it;
      if(!p.value)
         continue;
      if(p.name != "in")
         continue;
      if(p.parameters.back().value == me)
         packs_in_truck.push_back(p.parameters.front().value);
  }

  if(DEBUG_MOD) {
     for(deque<string>::iterator it = packs_in_truck.begin(); it != packs_in_truck.end(); it++) {
        cout << *it << endl;
     }
     if(packs_in_truck.empty()) {
        cout << "NO packs in " << me << endl;
     } else {
        cout << packs_in_truck.size() << " packs i truck " << me << endl;
     }
  }
  packs_in_truck.push_back(pack);

  NumericalFluentList* nfl = new NumericalFluentList();
  for(deque<string>::iterator it = packs_in_truck.begin(); it != packs_in_truck.end(); it++) {
     ParameterList pl;
     pl.push_back(Parameter("p", "package", *it));
     nfl->push_back(NumericalFluent("package-size", pl));
  }
  ParameterList plt;
  plt.push_back(Parameter("v", "vehicle", me));
  nfl->push_back(NumericalFluent("capacity", plt));

  if(!numericalFluentCallback(nfl)) {
     printf("WAAAAAAAAAHHHHHHHHH2\n");
     exit(1);
     return false;
  }

  if(DEBUG_MOD)
     cout << *nfl << endl;

  // TODO get sizes 1 - n-2, size n-1 == package, size n == capcity
  // 1..n-2 + n == full_load
  deque<double> packs;
  for(NumericalFluentList::iterator it = nfl->begin(); it != nfl->end(); it++) {
     packs.push_back((*it).value);
  } 
  double cap = packs.back();
  packs.erase(packs.end() - 1);
  double pSize = packs.back();
  packs.erase(packs.end() - 1);

  if(DEBUG_MOD)
     cout << "Packs" << packs.size() <<endl;
  double sumPacks = 0;
  for(deque<double>::iterator it = packs.begin(); it != packs.end(); it++) {
     if(DEBUG_MOD)
        cout << *it << endl;
     sumPacks += *it;
  }
  if(DEBUG_MOD) {
     cout << "newpack " << pSize << endl;
     cout << "cap " << cap << endl;
  }

  double loadCap = sumPacks + cap;
  if(DEBUG_MOD) {
     cout << "loadCap " << loadCap;

     cout << endl;
  }

  packs.push_back(pSize);

  bool loadPoss = checkLoading(packs, loadCap);

  //if(!loadPoss)
    //cout << "NOT POSSIBLE!!!" << endl;

  return loadPoss;
}

#if 0
int effectCall(ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback,
      vector<double>& writtenVars)
{
  NumericalFluentList* list = new NumericalFluentList();
  list->push_back(NumericalFluent("effectscalled",parameterList));   // THIS IS NASTY - pddl capitalization - where to fix this???
  numericalFluentCallback(list);
  writtenVars[0] = (list->front().value + 1.0);
  return 1;
}
#endif

void init(int argc, char** argv)
{
   printf("Initilizing Transport Module: ");
   for(int i = 0; i < argc; i++) {
      printf("%s ", argv[i]);
   }
   printf("\n");
}

subplanType genSubplan(const string & operatorName, const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int heuristic)
{
   printf("Generating subplan for %s\n", operatorName.c_str());
   return strdup(operatorName.c_str());
}

/// For final plan output: Convert a subplan into a string.
string outputSubplan(subplanType sp)
{
   char* str = static_cast<char*>(sp);
   return str;
}

void execSubplan(modulePlanType modulePlan)
{
   printf("Executing subplan:\n");
   for(modulePlanType::iterator it = modulePlan.begin(); it != modulePlan.end(); it++) {
      string sps = outputSubplan(*it);
      printf("%s\n", sps.c_str());
   }
   printf("\n");
}



