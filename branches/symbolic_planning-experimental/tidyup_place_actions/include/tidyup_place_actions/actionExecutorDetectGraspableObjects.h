#ifndef ACTION_EXECUTOR_DETECT_GRASPABLE_OBJECTS_H
#define ACTION_EXECUTOR_DETECT_GRASPABLE_OBJECTS_H

#include "continual_planning_executive/actionExecutorService.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/DetectGraspableObjects.h>

namespace tidyup_place_actions
{

    class ActionExecutorDetectGraspableObjects : public ActionExecutorService<tidyup_msgs::DetectGraspableObjects>
    {
        public:
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillGoal(tidyup_msgs::DetectGraspableObjects::Request & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(bool success, tidyup_msgs::DetectGraspableObjects::Response & response,
                    const DurativeAction & a, SymbolicState & current);

        protected:
            ros::ServiceClient _serviceClientGraspability;

        private:
            static const bool s_RequestGraspability = false;
    };

};

#endif

