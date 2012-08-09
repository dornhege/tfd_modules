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
    PlanningSceneInterface* psi = PlanningSceneInterface::instance();
    psi->resetPlanningScene();
    for (map<string, Door>::const_iterator doorIterator = doors.begin(); doorIterator != doors.end(); doorIterator++)
    {
        string doorName = doorIterator->first;
        if (psi->getCollisionObject(doorName) != NULL)
        {
            // update door pose
            PredicateList predicates;
            ParameterList pl;
            pl.push_back(Parameter("", "", doorName));
            predicates.push_back(Predicate("door-open", pl));
            PredicateList* predicateRequest = &predicates;
            if ( ! predicateCallback(predicateRequest))
            {
                ROS_ERROR("predicateCallback failed for door: %s", doorName.c_str());
                return false;
            }
            geometry_msgs::Pose pose;
            if (predicates[0].value)
            {
                // door is open
                psi->updateObject(doorName, doorIterator->second.openPose.pose);
            }
            else
            {
                // door is closed
                psi->updateObject(doorName, doorIterator->second.closedPose.pose);
            }
        }
    }
    // set robot state
    ROS_ASSERT(parameterList.size() > 1);
    const string& robotStartPose = parameterList[0].value;
    arm_navigation_msgs::RobotState state = psi->getRobotState();
    ArmState::get("/arm_configurations/side_tuck/position/", "right_arm").replaceJointPositions(state.joint_state);
    ArmState::get("/arm_configurations/side_tuck/position/", "left_arm").replaceJointPositions(state.joint_state);
    fillPoseFromState(state.multi_dof_joint_state.poses[0], robotStartPose, numericalFluentCallback);
    psi->setRobotState(state);
    return psi->sendDiff();
}

void PlanningSceneNavigationModule::initModule(int argc, char** argv)
{
    // load door locations
    string doorLocationFileName;
    g_NodeHandle->getParam("/continual_planning_executive/door_location_file", doorLocationFileName);
    loadDoorPoses(doorLocationFileName);

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

