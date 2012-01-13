
#ifndef TransportModules_STATE_H_
#define TransportModules_STATE_H_

#include "tfd_modules/opl/AbstractState.h"

#include "Target.h"
#include "Vehicle.h"
#include "Locatable.h"
#include "Location.h"
#include "Package.h"

namespace opl
{

namespace TransportModules
{

class State : public opl::interface::AbstractState
{
private:

    std::map<std::string, Target*> targets;
    std::map<std::string, Location*> locations;
    std::map<std::string, Locatable*> locatables;
    std::map<std::string, Vehicle*> vehicles;
    std::map<std::string, Package*> packages;


public:
    static State* stateInstance;
    State();
    virtual ~State();

    void initialize();

    const std::map<std::string, Target*>& getTargets() const {return targets;}
    const std::map<std::string, Location*>& getLocations() const {return locations;}
    const std::map<std::string, Locatable*>& getLocatables() const {return locatables;}
    const std::map<std::string, Vehicle*>& getVehicles() const {return vehicles;}
    const std::map<std::string, Package*>& getPackages() const {return packages;}

    bool road(const opl::TransportModules::Location* l1, const opl::TransportModules::Location* l2) const;
    double roadLength(const opl::TransportModules::Location* l1, const opl::TransportModules::Location* l2) const;

friend class StateFactory;
private:
    std::string getObjectID() const {return "";}
};

}

}

#endif /* TransportModules_STATE_H_ */
