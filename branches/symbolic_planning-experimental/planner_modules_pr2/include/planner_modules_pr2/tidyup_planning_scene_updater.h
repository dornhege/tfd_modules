/*
 * tidyup_planning_scene_updater.h
 *
 *  Created on: 10 Aug 2012
 *      Author: andreas
 */

#ifndef TIDYUP_PLANNING_SCENE_UPDATER_H_
#define TIDYUP_PLANNING_SCENE_UPDATER_H_

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <map>
#include <set>
#include <geometry_msgs/Pose.h>

using namespace modules;

class TidyupPlanningSceneUpdater
{
public:
    virtual ~TidyupPlanningSceneUpdater();

    static bool readState(
            const string& robotLocation,
            predicateCallbackType predicateCallback,
            numericalFluentCallbackType numericalFluentCallback,
            geometry_msgs::Pose& robotPose,
            std::map<std::string, geometry_msgs::Pose>& movableObjects,
            std::map<std::string, std::string>& graspedObjects,
            std::map<std::string, std::string>& objectsOnStatic,
            std::set<std::string>& openDoors);

    static bool update(const geometry_msgs::Pose& robotPose,
            const std::map<std::string, geometry_msgs::Pose>& movableObjects,
            const std::map<std::string, std::string>& graspedObjects,
            const std::set<std::string>& openDoors);

private:
    TidyupPlanningSceneUpdater();
    bool readState_(
            const string& robotLocation,
            predicateCallbackType predicateCallback,
            numericalFluentCallbackType numericalFluentCallback,
            geometry_msgs::Pose& robotPose,
            std::map<std::string, geometry_msgs::Pose>& movableObjects,
            std::map<std::string, std::string>& graspedObjects,
            std::map<std::string, std::string>& objectsOnStatic,
            std::set<std::string>& openDoors);

    bool update_(const geometry_msgs::Pose& robotPose,
            const std::map<std::string, geometry_msgs::Pose>& movableObjects,
            const std::map<std::string, std::string>& graspedObjects,
            const std::set<std::string>& openDoors);

    bool fillPoseFromState(geometry_msgs::Pose& pose,
            const std::string& poseName,
            numericalFluentCallbackType numericalFluentCallback);

    std::string logName;
    geometry_msgs::Pose defaultAttachPose;
    struct Door
    {
        string name;
        geometry_msgs::Pose closedPose;
        geometry_msgs::Pose openPose;
        Door(string name){this->name = name;}
    };
    map<string, Door> doors;
    void loadDoorPoses(const string& doorLocationFileName);
    static TidyupPlanningSceneUpdater* instance;
};

#endif /* TIDYUP_PLANNING_SCENE_UPDATER_H_ */
