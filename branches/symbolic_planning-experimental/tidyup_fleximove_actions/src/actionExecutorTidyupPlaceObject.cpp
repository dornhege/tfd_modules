#include "tidyup_place_actions/actionExecutorTidyupPlaceObject.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_place_actions, action_executor_place_object,
        tidyup_place_actions::ActionExecutorTidyupPlaceObject,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_place_actions
{

    bool ActionExecutorTidyupPlaceObject::fillGoal(tidyup_msgs::PlaceObjectGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 4);
        string location = a.parameters[0];
        string object = a.parameters[1];
        string targetPose = a.parameters[2];
        string arm = a.parameters[3];

        // set arm
        goal.left_arm = false;
        goal.right_arm = false;
        if(arm == "left_arm") {
            goal.left_arm = true;
        }
        if(arm == "right_arm") {
            goal.right_arm = true;
        }

        // set the target pose from state
        Predicate p;
        p.parameters.push_back(targetPose);

        p.name = "frame-id";
        if(!current.hasObjectFluent(p, &goal.position.header.frame_id))
            return false;
        p.name = "timestamp";
        double ts;
        if(!current.hasNumericalFluent(p, &ts))
            return false;
        goal.position.header.stamp = ros::Time(ts);
        p.name = "x";
        if(!current.hasNumericalFluent(p, &goal.position.pose.position.x))
            return false;
        p.name = "y";
        if(!current.hasNumericalFluent(p, &goal.position.pose.position.y))
            return false;
        p.name = "z";
        if(!current.hasNumericalFluent(p, &goal.position.pose.position.z))
            return false;
        p.name = "qx";
        if(!current.hasNumericalFluent(p, &goal.position.pose.orientation.x))
            return false;
        p.name = "qy";
        if(!current.hasNumericalFluent(p, &goal.position.pose.orientation.y))
            return false;
        p.name = "qz";
        if(!current.hasNumericalFluent(p, &goal.position.pose.orientation.z))
            return false;
        p.name = "qw";
        if(!current.hasNumericalFluent(p, &goal.position.pose.orientation.w))
            return false;

        return (goal.left_arm || goal.right_arm);
    }

    void ActionExecutorTidyupPlaceObject::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const tidyup_msgs::PlaceObjectResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("PlaceObject returned result: %s", result.result.c_str());
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Putdown succeeded.");
            ROS_ASSERT(a.parameters.size() == 4);
            string targetName = a.parameters[1];
            string objectPose = a.parameters[2];
            string arm = a.parameters[3];
            current.setObjectFluent("arm-position", arm, "unknown_armpos");
            current.setObjectFluent("at-object", targetName, objectPose);
            current.setBooleanPredicate("hand-free", arm, true);
            current.setBooleanPredicate("grasped", targetName + " " + arm, false);

            // for any object arm location: can-putdown false
            pair< multimap<string,string>::const_iterator, multimap<string,string>::const_iterator > objectRange;
            objectRange = current.getTypedObjects().equal_range("movable_object");
            pair< multimap<string,string>::const_iterator, multimap<string,string>::const_iterator > locationRange;
            locationRange = current.getTypedObjects().equal_range("grasp_location");
            vector<string> parameters;
            parameters.push_back("object");
            parameters.push_back(objectPose);
            parameters.push_back("arm");
            parameters.push_back("grasp_location");
            for(multimap<string,string>::const_iterator locationIterator = locationRange.first;
                    locationIterator != locationRange.second; locationIterator++)
            {
                for(multimap<string,string>::const_iterator objectIterator = objectRange.first;
                        objectIterator != objectRange.second; objectIterator++)
                {
                    // (can-putdown ?o ?p ?a ?l)
                    parameters[0] = objectIterator->second;
                    parameters[3] = locationIterator->second;
                    parameters[2] = "right_arm";
                    current.setBooleanPredicate("can-putdown", parameters, false);
                    parameters[2] = "left_arm";
                    current.setBooleanPredicate("can-putdown", parameters, false);
                }
            }
        }
    }

};

