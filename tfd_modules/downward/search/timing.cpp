#include "timing.h"
#ifndef _WIN32
#include <unistd.h>
#include <sys/times.h>
#else
#include <time.h>
#endif

Timing::Timing(const char* name, bool useUserTimes, int scoped_mode) : _useUserTimes(useUserTimes)
{
  _name = name;
  _stats = new Statistics<double>(name);
  _scopedMode = scoped_mode;
  start();
}

Timing::~Timing()
{
   if(_scopedMode & SP_INFO)
       printInfo();
   if(_scopedMode & SP_DEBUG)
       printDebug();
   if(_scopedMode & SP_STATS)
       printStats();
   delete _stats;
}

bool Timing::_timingEnabled = true;

void Timing::setGlobalTiming(bool enabled)
{
   _timingEnabled = enabled;
}

void Timing::start()
{
   if(_useUserTimes) {
#ifndef _WIN32
      struct tms timeS;
      times(&timeS);
      long clocks_per_sec = sysconf(_SC_CLK_TCK);
      _startTime = (double)timeS.tms_utime/((double)clocks_per_sec);
#else
		 //TODO: MG how to get usert time in windows?
			_startTime = ros::WallTime::now().toSec();
#endif
   } else {
      _startTime = ros::WallTime::now().toSec();
   }
   _ended = false;
}

void Timing::end()
{
   if(!_ended) {
      if(_useUserTimes) {
#ifndef _WIN32
         struct tms timeS;
         times(&timeS);
         long clocks_per_sec = sysconf(_SC_CLK_TCK);
         _endTime = (double)timeS.tms_utime/((double)clocks_per_sec);
#else
				_endTime = ros::WallTime::now().toSec();
#endif
      } else {
         _endTime = ros::WallTime::now().toSec();
      }
      _stats->addMeasurement(_endTime - _startTime);
   }
   _ended = true;
}

double Timing::peekDiff()
{
    if(!_ended) {
        double endTime = 0;
        if(_useUserTimes) {
#ifndef _WIN32
            struct tms timeS;
            times(&timeS);
            long clocks_per_sec = sysconf(_SC_CLK_TCK);
            endTime = (double)timeS.tms_utime/((double)clocks_per_sec);
#else
            endTime = ros::WallTime::now().toSec();
#endif
        } else {
            endTime = ros::WallTime::now().toSec();
        }
        return endTime - _startTime;
    } else {
        return _endTime - _startTime;
    }
}

double Timing::diff()
{
   end();
   return _endTime - _startTime;
}

void Timing::printInfo(bool alwaysPrint)
{
   if(!alwaysPrint) {
      if(!_timingEnabled)
         return;
   }

#ifdef _WIN32
#ifndef snprintf
#define snprintf _snprintf
#endif
#endif

   char buf[4096];
   snprintf(buf, 4095, "%s took: %.1fms.\n", _name, diff() * 1000.0);
   //printf("%s", buf);
   buf[strlen(buf) - 1] = '\0';  // remove \n
   ROS_INFO("%s", buf);
}

void Timing::printDebug(bool alwaysPrint)
{
   if(!alwaysPrint) {
      if(!_timingEnabled)
         return;
   }

#ifdef _WIN32
#ifndef snprintf
#define snprintf _snprintf
#endif
#endif

   char buf[4096];
   snprintf(buf, 4095, "%s took: %.1fms.\n", _name, diff() * 1000.0);
   //printf("%s", buf);
   buf[strlen(buf) - 1] = '\0';  // remove \n
   ROS_DEBUG("%s", buf);
}

void Timing::printStats(bool alwaysPrint)
{
   if(!alwaysPrint) {
      if(!_timingEnabled)
         return;
   }

   // choose a nice scale
   double mean = _stats->getMean();
   // basically prevent any 0.0xxx output
   if(mean < 0.0001)
       _stats->print(1000.0*1000.0, "us");
   else if(mean < 0.1)
       _stats->print(1000.0, "ms");
   else
       _stats->print(1.0, "s");
}

