/*
 * tidyup_planning_scene_updater.cpp
 *
 *  Created on: 10 Aug 2012
 *      Author: andreas
 */

#include "planner_modules_pr2/tidyup_planning_scene_updater.h"
#include <tidyup_utils/planning_scene_interface.h>
#include <tidyup_utils/arm_state.h>
#include <tidyup_utils/stringutil.h>
#include <hardcoded_facts/geometryPoses.h>
#include <ros/ros.h>

using std::vector;
using std::map;
using std::set;
using std::string;

TidyupPlanningSceneUpdater* TidyupPlanningSceneUpdater::instance = NULL;

TidyupPlanningSceneUpdater::TidyupPlanningSceneUpdater() : logName("[psu]")
{
    string doorLocationFileName;
    ros::param::get("/continual_planning_executive/door_location_file", doorLocationFileName);
    loadDoorPoses(doorLocationFileName);

    defaultAttachPose.position.x = 0.032;
    defaultAttachPose.position.y = 0.015;
    defaultAttachPose.position.z = 0.0;
    defaultAttachPose.orientation.x = 0.707;
    defaultAttachPose.orientation.y = -0.106;
    defaultAttachPose.orientation.z = -0.690;
    defaultAttachPose.orientation.w = 0.105;

    ROS_INFO("%s initialized.\n", logName.c_str());
}

TidyupPlanningSceneUpdater::~TidyupPlanningSceneUpdater()
{
}

bool TidyupPlanningSceneUpdater::readState(
        const string& robotLocation,
        predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback,
        geometry_msgs::Pose& robotPose,
        map<string, geometry_msgs::Pose>& movableObjects,
        map<string, string>& graspedObjects,
        map<string, string>& objectsOnStatic,
        set<string>& openDoors)
{
    if (instance == NULL) instance = new TidyupPlanningSceneUpdater();
    return instance->readState_(robotLocation, predicateCallback, numericalFluentCallback, robotPose, movableObjects, graspedObjects, objectsOnStatic, openDoors);
}

bool TidyupPlanningSceneUpdater::readState_(
        const string& robotLocation,
        predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback,
        geometry_msgs::Pose& robotPose,
        map<string, geometry_msgs::Pose>& movableObjects,
        map<string, string>& graspedObjects,
        map<string, string>& objectsOnStatic,
        set<string>& openDoors)
{
    // get poses of all movable objects
    PlanningSceneInterface* psi = PlanningSceneInterface::instance();
    psi->resetPlanningScene();
    geometry_msgs::Pose pose;
    const vector <arm_navigation_msgs::CollisionObject>& collisionObjects = psi->getCollisionObjects();
    for (vector <arm_navigation_msgs::CollisionObject>::const_iterator it = collisionObjects.begin();
            it != collisionObjects.end(); it++)
    {
        const string& objectName = it->id;
        if (StringUtil::startsWith(objectName, "table"))
            continue;
        if (StringUtil::startsWith(objectName, "door"))
            continue;
        if (! fillPoseFromState(pose, objectName, numericalFluentCallback))
        {
            psi->removeObject(objectName);
        }
        else
        {
            movableObjects.insert(make_pair(objectName, pose));
        }
    }
    const vector <arm_navigation_msgs::AttachedCollisionObject>& attachedObjects = psi->getAttachedCollisionObjects();
    for (vector <arm_navigation_msgs::AttachedCollisionObject>::const_iterator it = attachedObjects.begin(); it != attachedObjects.end(); it++)
    {
        const string& objectName = it->object.id;
        if (! fillPoseFromState(pose, objectName, numericalFluentCallback))
        {
            psi->removeObject(objectName);
        }
        else
        {
            movableObjects.insert(make_pair(objectName, pose));
        }
    }

    // find grasped objects and objects on tables
    PredicateList* predicates = NULL;
    if (!predicateCallback(predicates))
    {
        ROS_ERROR("%s predicateCallback failed.", logName.c_str());
        return false;
    }
    ROS_ASSERT(predicates != NULL);
    for (PredicateList::iterator it = predicates->begin(); it != predicates->end(); it++)
    {
        Predicate p = *it;
        if (!p.value)
            continue;
        if (p.name == "on")
        {
            ROS_ASSERT(p.parameters.size() == 2);
            // (on movable static)
            objectsOnStatic.insert(make_pair(p.parameters.front().value, p.parameters.back().value));
        }
        else if (p.name == "grasped")
        {
            ROS_ASSERT(p.parameters.size() == 2);
            // (grasped object arm)
            graspedObjects.insert(make_pair(p.parameters.front().value, p.parameters.back().value));
        }
        else if (p.name == "door-open")
        {
            ROS_ASSERT(p.parameters.size() == 1);
            // (door-open door)
            openDoors.insert(p.parameters.front().value);
        }
    }

    // Robot pose
    if (! fillPoseFromState(robotPose, robotLocation, numericalFluentCallback))
    {
        ROS_ERROR("%s get robot location failed.", logName.c_str());
    }
    return true;
}

bool TidyupPlanningSceneUpdater::update(const geometry_msgs::Pose& robotPose,
        const map<string, geometry_msgs::Pose>& movableObjects,
        const map<string, string>& graspedObjects,
        const set<string>& openDoors)
{
    if (instance == NULL) instance = new TidyupPlanningSceneUpdater();
    return instance->update_(robotPose, movableObjects, graspedObjects, openDoors);
}

bool TidyupPlanningSceneUpdater::update_(const geometry_msgs::Pose& robotPose,
        const map<string, geometry_msgs::Pose>& movableObjects,
        const map<string, string>& graspedObjects,
        const set<string>& openDoors)
{
    PlanningSceneInterface* psi = PlanningSceneInterface::instance();
//    psi->resetPlanningScene();

    // set robot state in planning scene
    ROS_INFO("%s update robot state in planning scene", logName.c_str());
    arm_navigation_msgs::RobotState state = psi->getRobotState();
    state.multi_dof_joint_state.poses[0] = robotPose;
    ArmState::get("/arm_configurations/side_tuck/position/", "right_arm").replaceJointPositions(state.joint_state);
    ArmState::get("/arm_configurations/side_tuck/position/", "left_arm").replaceJointPositions(state.joint_state);

    // update pose of graspable object in the planning scene
    for(map<string, geometry_msgs::Pose>::const_iterator movabelObjectIt = movableObjects.begin(); movabelObjectIt != movableObjects.end(); movabelObjectIt++)
    {
        string object_name = movabelObjectIt->first;
        ROS_INFO("%s updating object %s", logName.c_str(), object_name.c_str());
        // if this object is attached somewhere we need to detach it
        if (psi->getAttachedCollisionObject(object_name) != NULL)
        {
            psi->detachObjectAndAdd(object_name);
        }
        // object is not attached, update pose
        if (psi->getCollisionObject(object_name) != NULL)
        {
            psi->updateObject(object_name, movabelObjectIt->second);
        }
        else
        {
            ROS_ERROR("%s object %s does not exist in planning scene.", logName.c_str(), object_name.c_str());
            return false;
        }
    }

    // attach putdown object to the correct arm
//    const arm_navigation_msgs::CollisionObject* object = psi->getCollisionObject(request.putdown_object);
    for (map<string, string>::const_iterator graspedIt = graspedObjects.begin(); graspedIt != graspedObjects.end(); graspedIt++)
    {
        const string& objectName = graspedIt->first;
        const string& arm = graspedIt->second;
        ROS_INFO("%s attaching object %s to arm %s", logName.c_str(), objectName.c_str(), arm.c_str());
        ArmState::get("/arm_configurations/side_carry/position/", arm).replaceJointPositions(state.joint_state);
        psi->attachObjectToGripper(objectName, arm);
        psi->updateObject(objectName, defaultAttachPose);
    }
    psi->setRobotState(state);

    // update doors
    for (map<string, Door>::const_iterator doorIt = doors.begin(); doorIt != doors.end(); doorIt++)
    {
        if (openDoors.find(doorIt->first) != openDoors.end())
        {
            // door is open
            psi->updateObject(doorIt->first, doorIt->second.openPose);
        }
        else
        {
            // door is closed
            psi->updateObject(doorIt->first, doorIt->second.closedPose);
        }
    }

    // send changes
    return psi->sendDiff();
}

bool TidyupPlanningSceneUpdater::fillPoseFromState(geometry_msgs::Pose& pose,
        const string& poseName,
        numericalFluentCallbackType numericalFluentCallback)
{
    // create the numerical fluent request
    ParameterList startParams;
    startParams.push_back(Parameter("", "", poseName));
    NumericalFluentList nfRequest;
    nfRequest.reserve(7);
    nfRequest.push_back(NumericalFluent("x", startParams));
    nfRequest.push_back(NumericalFluent("y", startParams));
    nfRequest.push_back(NumericalFluent("z", startParams));
    nfRequest.push_back(NumericalFluent("qx", startParams));
    nfRequest.push_back(NumericalFluent("qy", startParams));
    nfRequest.push_back(NumericalFluent("qz", startParams));
    nfRequest.push_back(NumericalFluent("qw", startParams));

    // get the fluents
    NumericalFluentList* nfRequestP = &nfRequest;
    if (!numericalFluentCallback(nfRequestP))
    {
        ROS_ERROR("fillPoseFromState failed for object: %s", poseName.c_str());
        return false;
    }

    // fill pose stamped
    pose.position.x = nfRequest[0].value;
    pose.position.y = nfRequest[1].value;
    pose.position.z = nfRequest[2].value;
    pose.orientation.x = nfRequest[3].value;
    pose.orientation.y = nfRequest[4].value;
    pose.orientation.z = nfRequest[5].value;
    pose.orientation.w = nfRequest[6].value;
    return true;
}

void TidyupPlanningSceneUpdater::loadDoorPoses(const string& doorLocationFileName)
{
    GeometryPoses locations = GeometryPoses();
    ROS_ASSERT_MSG(locations.load(doorLocationFileName), "Could not load locations from \"%s\".", doorLocationFileName.c_str());
    const std::map<std::string, geometry_msgs::PoseStamped>& poses = locations.getPoses();
    for(std::map<std::string, geometry_msgs::PoseStamped>::const_iterator posesIterator = poses.begin(); posesIterator != poses.end(); posesIterator++)
    {
        string name = posesIterator->first;
        if (StringUtil::startsWith(name, "door"))
        {
            string doorName;
            bool open = false;
            if(StringUtil::endsWith(name, "_closed"))
            {
                doorName = name.substr(0, name.length()-7);
            }
            else if(StringUtil::endsWith(name, "_open"))
            {
                doorName = name.substr(0, name.length()-5);
                open = true;
            }
            else
            {
                ROS_ERROR("navstack planning scene: misformated door location entry %s in file %s", name.c_str(), doorLocationFileName.c_str());
                continue;
            }
            map<string, Door>::iterator doorIterator = doors.find(doorName);
            if (doorIterator == doors.end())
            {
                doors.insert(make_pair(doorName, Door(doorName)));
                doorIterator = doors.find(doorName);
            }
            Door& door = doorIterator->second;
            if (open)
            {
                door.openPose = posesIterator->second.pose;
            }
            else
            {
                door.closedPose = posesIterator->second.pose;
            }
        }
    }
}

