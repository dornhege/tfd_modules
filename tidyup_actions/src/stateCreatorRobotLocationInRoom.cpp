#include "tidyup_actions/stateCreatorRobotLocationInRoom.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_actions, state_creator_robot_location_in_room,
        tidyup_actions::StateCreatorRobotLocationInRoom, continual_planning_executive::StateCreator)

namespace tidyup_actions
{

    StateCreatorRobotLocationInRoom::StateCreatorRobotLocationInRoom()
    {
    }

    StateCreatorRobotLocationInRoom::~StateCreatorRobotLocationInRoom()
    {
    }

    void StateCreatorRobotLocationInRoom::initialize(const std::deque<std::string> & arguments)
    {
        ROS_ASSERT(arguments.size() == 2);

        _robotPoseObject = arguments[0];
        _locationType = arguments[1];
    }

    bool StateCreatorRobotLocationInRoom::fillState(SymbolicState & state)
    {
        // if location-in-room for robot_location is not set, infer it
        // robot_location is in that room that the nearest location we know location-in-room from is in
        string robotRoom;
        Predicate p;
        p.name = "location-in-room";
        p.parameters.push_back(_robotPoseObject);
        if(!state.hasObjectFluent(p, &robotRoom)) {
            // get the action robot position
            tf::StampedTransform transform;
            try{
                _tf.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                return false;
            }

            // get all locations and search for the nearest that has location-in-room set
            pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> locations =
                state.getTypedObjects().equal_range(_locationType);

            double minDist = HUGE_VAL;
            robotRoom = "";
            for(SymbolicState::TypedObjectConstIterator it = locations.first; it != locations.second; it++) {
                string location = it->second;
                p.parameters[0] = location;

                // first check if location-in-room is set
                p.name = "location-in-room";
                string room;
                if(!state.hasObjectFluent(p, &room)) {
                    continue;
                }

                p.name = "x";
                double valueX;
                if(!state.hasNumericalFluent(p, &valueX)) {
                    ROS_ERROR("%s: location: %s - no x-location.", __func__, location.c_str());
                    continue;
                }
                double valueY;
                p.name = "y";
                if(!state.hasNumericalFluent(p, &valueY)) {
                    ROS_ERROR("%s: location: %s - no y-location.", __func__, location.c_str());
                    continue;
                }

                double dist = hypot(valueX - transform.getOrigin().x(), valueY - transform.getOrigin().y());
                if(dist < minDist) {
                    minDist = dist;
                    robotRoom = room;
                }
            }

            // search done, now enter robotRoom if found
            if(robotRoom.empty()) {
                ROS_WARN("Could not infer location-in-room for robot_location as no nearest location with location-in-room set was found.");
                return false;
            } else {
                ROS_INFO("Determined (location-in-room %s) = %s", _robotPoseObject.c_str(), robotRoom.c_str());
                state.setObjectFluent("location-in-room", _robotPoseObject, robotRoom);
            }
        }

        return true;
    }

};

