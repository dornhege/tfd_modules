
#ifndef TransportModules_Vehicle_canLoad_plannerCall_H_
#define TransportModules_Vehicle_canLoad_plannerCall_H_

#include "State.h"

namespace opl
{

namespace TransportModules
{

bool Vehicle_canLoad(const State* currentState,
        const opl::TransportModules::Vehicle* this_pointer, 
        const opl::TransportModules::Package* p, 
        int relaxed);

}

}

#endif 
