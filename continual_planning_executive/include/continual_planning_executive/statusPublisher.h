#ifndef STATUS_PUBLISHER_H
#define STATUS_PUBLISHER_H

#include "continual_planning_executive/ContinualPlanningStatus.h"
#include "continual_planning_executive/plan.h"
#include "continual_planning_executive/symbolicState.h"
#include <ros/ros.h>

class StatusPublisher
{
   public:
      StatusPublisher();
      ~StatusPublisher();

      void startedStateEstimation();
      void finishedStateEstimation(bool success, const SymbolicState & state);

      void startedMonitoring();
      void finishedMonitoring(bool success);

      void startedPlanning();
      void finishedPlanning(bool success, const Plan & plan);

      void startedExecution(const DurativeAction & a);
      void finishedExecution(bool success, const DurativeAction & a);

      void updateCurrentPlan(const Plan & plan);

      void finishedContinualPlanning(bool success, const std::string & result);


      void publishStatus(int component, int status, const std::string & description);

   private:
      ros::Publisher _pubStatus;
};

#endif

