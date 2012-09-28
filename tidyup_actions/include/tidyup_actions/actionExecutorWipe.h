#ifndef ACTION_EXECUTOR_WIPE_H
#define ACTION_EXECUTOR_WIPE_H

#include "continual_planning_executive/actionExecutorService.hpp"
#include "continual_planning_executive/symbolicState.h"
#include "coverage_srvs/CleanSpot.h"

namespace tidyup_actions
{

   class ActionExecutorWipe : public ActionExecutorService<coverage_srvs::CleanSpot>
    {
        public:
            virtual void initialize(const std::deque<std::string> & arguments);
            virtual bool fillGoal(coverage_srvs::CleanSpot::Request & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(bool success, coverage_srvs::CleanSpot::Response & response,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

