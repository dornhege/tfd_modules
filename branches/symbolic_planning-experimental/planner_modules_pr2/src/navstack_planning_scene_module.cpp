#include "planner_modules_pr2/navstack_planning_scene_module.h"
#include "planner_modules_pr2/navstack_module.h"
#include "hardcoded_facts/geometryPoses.h"
#include "tidyup_utils/stringutil.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <utility>
using std::map; using std::pair; using std::make_pair;

VERIFY_CONDITIONCHECKER_DEF(planning_scene_pathCost);

PlanningSceneNavigationModule* PlanningSceneNavigationModule::singleton_instance = NULL;

PlanningSceneNavigationModule* PlanningSceneNavigationModule::instance()
{
    if (singleton_instance == NULL) singleton_instance = new PlanningSceneNavigationModule();
    return singleton_instance;
}

void PlanningSceneNavigationModule::loadDoorPoses(const string& doorLocationFileName)
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
                door.openPose = posesIterator->second;
            }
            else
            {
                door.closedPose = posesIterator->second;
            }
        }
    }
}

void PlanningSceneNavigationModule::fillPoseFromState(geometry_msgs::Pose& pose, const string& poseName, numericalFluentCallbackType numericalFluentCallback)
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
        ROS_INFO("fillPoseFromState failed for object: %s", poseName.c_str());
        return;
    }

    // fill pose stamped
    pose.position.x = nfRequest[0].value;
    pose.position.y = nfRequest[1].value;
    pose.position.z = nfRequest[2].value;
    pose.orientation.x = nfRequest[3].value;
    pose.orientation.y = nfRequest[4].value;
    pose.orientation.z = nfRequest[5].value;
    pose.orientation.w = nfRequest[6].value;
}

bool PlanningSceneNavigationModule::setPlanningSceneDiffFromState(const ParameterList & parameterList,
        predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback)
{
    // update objects in planning scene
    vector<arm_navigation_msgs::CollisionObject>& objectList = spsdService.request.planning_scene_diff.collision_objects;
    vector<arm_navigation_msgs::CollisionObject>::iterator objectIterator = objectList.begin();
    for ( ; objectIterator != objectList.end(); objectIterator++)
    {
        const string& objectName = objectIterator->id;
        map<string, Door>::const_iterator doorIterator = doors.find(objectName);
//        ROS_INFO("update object: %s", objectName.c_str());
        if (doorIterator != doors.end())
        {
            // set door pose
            PredicateList predicates;
            ParameterList pl;
            pl.push_back(Parameter("", "", objectName));
            predicates.push_back(Predicate("door-open", pl));
            PredicateList* predicateRequest = &predicates;
            if ( ! predicateCallback(predicateRequest))
            {
                ROS_ERROR("predicateCallback failed for door: %s", objectName.c_str());
                return false;
            }
            if (predicates[0].value)
            {
                // door is open
                objectIterator->poses[0] = doorIterator->second.openPose.pose;
//                ROS_WARN("door open: %s", objectName.c_str());
            }
            else
            {
                // door is closed
                objectIterator->poses[0] = doorIterator->second.closedPose.pose;
//                ROS_INFO("door closed: %s", objectName.c_str());
            }
        }
        else
        {
            fillPoseFromState(objectIterator->poses[0], objectName, numericalFluentCallback);
        }
    }
    // set robot state
    ROS_ASSERT(parameterList.size() > 1);
    const string& robotStartPose = parameterList[0].value;
    fillPoseFromState(spsdService.request.planning_scene_diff.robot_state.multi_dof_joint_state.poses[0], robotStartPose, numericalFluentCallback);
    return setPlanningSceneService.call(spsdService);
}

void PlanningSceneNavigationModule::initModule(int argc, char** argv)
{
    // load door locations
    string doorLocationFileName;
    g_NodeHandle->getParam("/continual_planning_executive/door_location_file", doorLocationFileName);
    loadDoorPoses(doorLocationFileName);

    // init service for planning scene
    std::string service_name = "/environment_server/set_planning_scene_diff";
    while (!ros::service::waitForService(service_name, ros::Duration(3.0)))
    {
        ROS_ERROR("Service %s not available - waiting.", service_name.c_str());
    }
    setPlanningSceneService = g_NodeHandle->serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(service_name, true);
    if (!setPlanningSceneService)
    {
        ROS_FATAL("Could not initialize get plan service from %s (client name: %s)", service_name.c_str(), setPlanningSceneService.getService().c_str());
    }
    // init arm joint states
    armStates.push_back(ArmState("right_arm", "/arm_configurations/side_tuck/position/right_arm"));
    armStates.push_back(ArmState("left_arm", "/arm_configurations/side_tuck/position/left_arm"));

    // init planning scene
    if (setPlanningSceneService.call(spsdService))
    {
        spsdService.request.planning_scene_diff = spsdService.response.planning_scene;
    }
    else
    {
        ROS_ERROR("%s Could not initialize planning scene.", __PRETTY_FUNCTION__);
    }

    // replace arm joint states
    for (int i = RIGHT_ARM_AT_SIDE; i <= LEFT_ARM_AT_SIDE; i++)
    {
        armStates[i].replaceJointPositions(spsdService.request.planning_scene_diff.robot_state.joint_state);
    }

    ROS_INFO("Initialized planning scene navstack module.\n");
}


void planning_scene_navstack_init(int argc, char** argv)
{
    navstack_init(argc, argv);
    PlanningSceneNavigationModule::instance()->initModule(argc, argv);
}

double planning_scene_pathCost(const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
    // first lookup in the cache if we answered the query already
    double cost = 0;
    if (g_PathCostCache.get(computePathCacheKey(parameterList[0].value, parameterList[1].value), cost))
    {
        return cost;
    }
    // set planning scene
    if (PlanningSceneNavigationModule::instance()->setPlanningSceneDiffFromState(parameterList, predicateCallback, numericalFluentCallback))
    {
        cost = pathCost(parameterList, predicateCallback, numericalFluentCallback, relaxed);
        return cost;
    }
    else
    {
        ROS_ERROR("%s Could not set planning scene diff.", __PRETTY_FUNCTION__);
    }
    return INFINITE_COST;
}

