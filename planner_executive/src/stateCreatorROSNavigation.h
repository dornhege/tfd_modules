#ifndef STATE_CREATOR_R_O_S_NAVIGATION_H
#define STATE_CREATOR_R_O_S_NAVIGATION_H

#include "stateCreator.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>

class StateCreatorROSNavigation : public StateCreator
{
   public:
      StateCreatorROSNavigation();
      ~StateCreatorROSNavigation();

      virtual bool fillState(SymbolicState & state);

   protected:
      tf::TransformListener _tf;

      double _goalTolerance;
};

#endif

