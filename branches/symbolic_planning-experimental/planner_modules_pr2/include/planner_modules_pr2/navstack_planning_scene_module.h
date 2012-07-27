#ifndef NAVSTACK_PLANNING_SCENE_MODULE_H
#define NAVSTACK_PLANNING_SCENE_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include "tidyup_utils/arm_state.h"
#include "tidyup_utils/planning_scene_interface.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace modules;

class PlanningSceneNavigationModule
{
public:
    static PlanningSceneNavigationModule* instance();
    static void fillPoseFromState(geometry_msgs::Pose& pose, const string& poseName, numericalFluentCallbackType numericalFluentCallback);

    bool setPlanningSceneDiffFromState(const ParameterList & parameterList,
            predicateCallbackType predicateCallback,
            numericalFluentCallbackType numericalFluentCallback);
    void initModule(int argc, char** argv);

    // TEST and DEBUG methods
//    const arm_navigation_msgs::SetPlanningSceneDiff& getPlanningScene() const {return spsdService;}
//    bool setPlanningSceneDiff(arm_navigation_msgs::SetPlanningSceneDiff& service) {return setPlanningSceneService.call(service);}

private:
    static PlanningSceneNavigationModule* singleton_instance;
    struct Door
    {
        string name;
        geometry_msgs::PoseStamped closedPose;
        geometry_msgs::PoseStamped openPose;
        Door(string name){this->name = name;}
    };
    map<string, Door> doors;
    vector<ArmState> armStates;
    PlanningSceneNavigationModule(){}
    void loadDoorPoses(const string& doorLocationFileName);
};


#ifdef __cplusplus
extern "C" {
#endif

void planning_scene_navstack_init(int argc, char** argv);

double planning_scene_pathCost(const ParameterList & parameterList, predicateCallbackType predicateCallback,
      numericalFluentCallbackType numericalFluentCallback, int relaxed);

#ifdef __cplusplus
}
#endif


#endif

