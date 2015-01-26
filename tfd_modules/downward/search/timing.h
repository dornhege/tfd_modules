#ifndef TIMING_H
#define TIMING_H

#if ROS_BUILD
#include <ros/ros.h>
#endif
#include "statistics.h"

/// A Simple class for timing purposes.
/**
 * Common Usage:
 * Timing t("Nasty Function");
 * nastyFunction();
 * t.printInfo();
 *
 * Will printDebug: "Nasty Function took 123ms."
 *
 * The timing can be (re-)started and ended by calling the respective functions.
 * diff() and printInfo() will end the timing implicitly.
 */
class Timing
{
   public:
      enum SCOPE_PRINT {
        SP_NONE = 0,
        SP_INFO = 1,
        SP_DEBUG = 2,
        SP_STATS = 4,
      };

      /// Create a timing object, if useUserTimes is true, times() will be used instead of getCurrentTime()
      Timing(const char* name = "Timing", bool useUserTimes = true, int scoped_mode = SP_NONE);
      ~Timing();

      /// Resets start time to current time.
      void start();
      /// Stops the timing until start is called again.
      void end();

      /// Secs since construction (or start()) - does not end() the timing.
      double peekDiff();

      /// Secs since construction (or start())
      /**
       * This function also calls end().
       */
      double diff();

      /// Does print time since construction.
      /**
       * This function also calls end().
       * \param [in] alwaysPrint if true, overrides global setting and prints anyways
       */
      void printInfo(bool alwaysPrint = false);
      /// Same as printInfo, but uses ROS_DEBUG.
      void printDebug(bool alwaysPrint = false);
      void printStats(bool alwaysPrint = false);

      /// Set, if timing info will be printed out for all Timing instances.
      static void setGlobalTiming(bool enabled);

      Statistics<double> & getStats() { return *_stats; }

   private:
      bool _useUserTimes;

      double _startTime;
      double _endTime;
      const char* _name;
      bool _ended;

      int _scopedMode;

      Statistics<double> * _stats;

      static bool _timingEnabled;
};

#endif

