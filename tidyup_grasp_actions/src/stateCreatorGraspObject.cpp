#include "tidyup_grasp_actions/stateCreatorGraspObject.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_grasp_actions, state_creator_grasp_object,
        tidyup_grasp_actions::StateCreatorGraspObject, continual_planning_executive::StateCreator)

namespace tidyup_grasp_actions 
{

    StateCreatorGraspObject::StateCreatorGraspObject()
    {
        ros::NodeHandle nhPriv("~");
        nhPriv.param("goal_tolerance", _goalTolerance, 0.5);
        ROS_INFO("Tolerance for accepting nav goals set to %f.", _goalTolerance);
    }

    StateCreatorGraspObject::~StateCreatorGraspObject()
    {
    }

    bool StateCreatorGraspObject::fillState(SymbolicState & state)
    {
        tf::StampedTransform transform;
        try{
            _tf.lookupTransform("/map", "/base_link",  
                    ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return false;
        }

        // Check if we are at any grasp_locations 
        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> targets = 
            state.getTypedObjects().equal_range("grasp_location");

        vector<string> paramList;
        paramList.push_back("dummy");

        double minDist = HUGE_VAL;
        string nearestTarget = "";

        int atGraspLocations = 0;
        for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++) {
            string target = it->second;
            Predicate p;
            paramList[0] = target;
            p.parameters = paramList;

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
                ROS_INFO("(at) target %s !", target.c_str());
                state.setBooleanPredicate("at-base", target, true);
                atGraspLocations++;
            } else {
                state.setBooleanPredicate("at-base", target, false);
            }
            if(hypot(dx, dy) < minDist) {
                minDist = hypot(dx, dy);
                nearestTarget = target;
            }
        }
        ROS_INFO("Nearest target is %s (%f m).", nearestTarget.c_str(), minDist);

        // Real robot location
        state.addObject("l0", "location");
        state.setNumericalFluent("x", "l0", transform.getOrigin().x());
        state.setNumericalFluent("y", "l0", transform.getOrigin().y());
        state.setNumericalFluent("z", "l0", transform.getOrigin().z());
        state.setNumericalFluent("qx", "l0", transform.getRotation().x());
        state.setNumericalFluent("qy", "l0", transform.getRotation().y());
        state.setNumericalFluent("qz", "l0", transform.getRotation().z());
        state.setNumericalFluent("qw", "l0", transform.getRotation().w());
        state.setNumericalFluent("timestamp", "l0", ros::Time::now().toSec());
        state.addObject("/map", "frameid");
        state.setObjectFluent("frame-id", "l0", "/map");

        if(atGraspLocations == 0) {
            state.setBooleanPredicate("at-base", "l0", true);
        } else {
            state.setBooleanPredicate("at-base", "l0", false);    //hm what do we give in the cost module here?
            if(atGraspLocations > 1) {
                ROS_WARN("We are at %d grasp locations at the same time!.", atGraspLocations);
            }
        }

        // can-navigate is always true for now
        for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++) {
            string target = it->second;
            for(SymbolicState::TypedObjectConstIterator it2 = targets.first; it2 != targets.second; it2++) {
                string target2 = it2->second;
                state.setBooleanPredicate("can-navigate", target + " " + target2, true);
            }
            if(atGraspLocations == 0)
                state.setBooleanPredicate("can-navigate", target + " l0", true);
        }

        return true;
    }

};

