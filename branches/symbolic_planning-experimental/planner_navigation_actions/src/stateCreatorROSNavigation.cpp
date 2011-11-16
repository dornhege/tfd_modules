#include "planner_navigation_actions/stateCreatorROSNavigation.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(planner_navigation_actions, state_creator_ros_navigation,
        planner_navigation_actions::StateCreatorROSNavigation, continual_planning_executive::StateCreator)

namespace planner_navigation_actions
{

StateCreatorROSNavigation::StateCreatorROSNavigation()
{
   ros::NodeHandle nhPriv("~");
   nhPriv.param("goal_tolerance", _goalTolerance, 0.5);
   ROS_INFO("Tolerance for accepting goals set to %f.", _goalTolerance);
}

StateCreatorROSNavigation::~StateCreatorROSNavigation()
{
}

bool StateCreatorROSNavigation::fillState(SymbolicState & state)
{
   state.addObject("l0", "location");

   tf::StampedTransform transform;
   try{
      _tf.lookupTransform("/map", "/base_link",  
            ros::Time(0), transform);
   }
   catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return false;
   }

   state.setBooleanPredicate("at", "l0", true);
   state.setNumericalFluent("x", "l0", transform.getOrigin().x());
   state.setNumericalFluent("y", "l0", transform.getOrigin().y());
   state.setNumericalFluent("z", "l0", transform.getOrigin().z());
   state.setNumericalFluent("qx", "l0", transform.getRotation().x());
   state.setNumericalFluent("qy", "l0", transform.getRotation().y());
   state.setNumericalFluent("qz", "l0", transform.getRotation().z());
   state.setNumericalFluent("qw", "l0", transform.getRotation().w());

   // TODO (at t0) + make sure there is action for that.

   // Check if we reached any goals
   pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> targets = 
      state.getTypedObjects().equal_range("target");
   
   vector<string> paramList;
   paramList.push_back("dummy");
   
   double minDist = HUGE_VAL;
   string nearestTarget = "";

   for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++) {
      string target = it->second;
      Predicate p;
      paramList[0] = target;
      p.parameters = paramList;

      // check if its explored already
      p.name = "explored";
      bool explored = false;
      if(!state.hasBooleanPredicate(p, &explored)) {
         ROS_ERROR("%s: state has no explored predicate for target %s.", __func__, target.c_str());
      } else {
         if(explored)
            continue;
      }

      p.name = "x";
      double valueX;
      if(!state.hasNumericalFluent(p, &valueX)) {
         ROS_ERROR("%s: target object: %s - no x-location in goal.", __func__, target.c_str());
         continue;
      }
      double valueY;
      p.name = "y";
      if(!state.hasNumericalFluent(p, &valueY)) {
         ROS_ERROR("%s: target object: %s - no y-location in goal.", __func__, target.c_str());
         continue;
      }

      double dx = transform.getOrigin().x() - valueX;
      double dy = transform.getOrigin().y() - valueY;
      // Found a target - update state!
      if(hypot(dx, dy) < _goalTolerance) {
         ROS_INFO("Target %s explored!", target.c_str());
         state.setBooleanPredicate("explored", target, true);
      }
      if(hypot(dx, dy) < minDist) {
         minDist = hypot(dx, dy);
         nearestTarget = target;
      }
   }
   ROS_INFO("Nearest target is %s (%f m).", nearestTarget.c_str(), minDist);

   return true;
}

};

