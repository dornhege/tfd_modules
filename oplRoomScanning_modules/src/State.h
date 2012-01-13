
#ifndef RoomScanning_STATE_H_
#define RoomScanning_STATE_H_

#include <tfd_modules/opl/AbstractState.h>

#include "Pose.h"
#include "Target.h"
#include "Robot.h"
#include "Door.h"

namespace opl
{

namespace RoomScanning
{

class State : public opl::interface::AbstractState
{
private:

    std::map<std::string, Pose*> poses;
    std::map<std::string, Target*> targets;
    std::map<std::string, Door*> doors;
    std::map<std::string, Robot*> robots;


public:
    static State* stateInstance;
    State();
    virtual ~State();

    void initialize();

    const std::map<std::string, Pose*>& getPoses() const {return poses;}
    const std::map<std::string, Target*>& getTargets() const {return targets;}
    const std::map<std::string, Door*>& getDoors() const {return doors;}
    const std::map<std::string, Robot*>& getRobots() const {return robots;}

    bool inRange(const opl::RoomScanning::Pose* p1, const opl::RoomScanning::Pose* p2) const;

friend class StateFactory;
private:
    std::string getObjectID() const {return "";}
};

}

}

#endif /* RoomScanning_STATE_H_ */
