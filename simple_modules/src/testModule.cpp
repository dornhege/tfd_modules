#include "testModule.h"
#include <stdlib.h>
#include <sys/time.h>
using namespace std;
#include <ros/ros.h>

double checkTrue(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
   ROS_DEBUG("Calling %s module", __func__);
   return true;
}

double checkFalse(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
   ROS_DEBUG("Calling %s module", __func__);
   return false;
}

double checkRandom(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
   ROS_DEBUG("Calling %s module", __func__);
   struct timeval tv;
   gettimeofday(&tv, NULL);
   srand(tv.tv_usec); // move to init from module or _init
   return (rand() % 2 == 0);
}

double checkParamEqualCharacterCount(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
   ROS_DEBUG("Calling %s module", __func__);
   if(parameterList.empty())
      return false;
   Parameter p1 = parameterList.front();
   // we care for the instance, not the name itself
   int len = p1.value.length();
   return (len % 2 == 0);
}


int countIt(const ParameterList & parameterList,
          predicateCallbackType  predicateCallback,
          numericalFluentCallbackType numericalFluentCallback,
	 vector<double>& writtenVars)
{
   ROS_DEBUG("Calling %s module", __func__);
   NumericalFluentList* nlf = new NumericalFluentList();
   nlf->push_back(NumericalFluent("stepped-on", parameterList));
   numericalFluentCallback(nlf);

   writtenVars[0] = nlf->front().value + 1.0;
   return 1;
}

int dummyEffect(const ParameterList & parameterList,
          predicateCallbackType  predicateCallback,
          numericalFluentCallbackType numericalFluentCallback,
	 vector<double>& writtenVars)
{
   ROS_DEBUG("Calling %s module", __func__);
   if(predicateCallback == NULL)
      return false;
   PredicateList pl;
   ParameterList paramList;
   paramList.push_back(Parameter("","","pos-1"));
   pl.push_back(Predicate("free", paramList));
   PredicateList* plp = &pl;
   bool ret = predicateCallback(plp); 
   if(!ret)
      return false;
   if(pl.empty())      // callback removed our predicate!!!!!!!!!!!!
      return false;
   // now we could / should react on the value of the predicate. skip
   // this for the moment....
   std::cout << "(free pos-1) is currently " << pl.front().value <<
          std::endl;
   // write arbitrary value back
   writtenVars[0] = 17.0;
   return 0;
}


double costDrive(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
   ROS_DEBUG("Calling %s module", __func__);

   string from = parameterList.at(0).value;
   string to = parameterList.at(1).value;

   //cout << "driving from " << from << " to " << to << endl;

   if(from == "l1") {
      if(to == "l1") {
         return 1.0;
      } else if(to == "l2") {
         return 10.0;
      } else if(to == "l3") {
         return 42.0;
      }
   } else if(from == "l2") {
      if(to == "l1") {
         return 10.0;
      } else if(to == "l2") {
         return 1.0;
      } else if(to == "l3") {
         return 10.0;
      }
   } else if(from == "l3") {
      if(to == "l1") {
         return 42.0;
      } else if(to == "l2") {
         return 10.0;
      } else if(to == "l3") {
         return 1.0;
      }
   }

   return 1.0;
}

