#ifndef ACTION_EXECUTOR_SERVICE_H
#define ACTION_EXECUTOR_SERVICE_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"
#include <ros/ros.h>

/// Templated base class for creating action executors that call a service for execution.
/**
 * The class is templated over the service.
 *
 * A minimal implementation should derive from this class (instantiating the template properly)
 * and implement fillGoal.
 *
 * canExecute can be overridden to test more specific than just the name.
 * updateState can be used to update the planner state upon execution depending on the result
 * of the execution.
 */
template <class Service>
class ActionExecutorService : public continual_planning_executive::ActionExecutorInterface
{
   public:
      ActionExecutorService();
      ~ActionExecutorService();

      /// Initialize the executor by creating the ServiceClient.
      /**
       * \param [in] arguments should be name of action (as in the plan),
       *    and the name of the service to connect to.
       */
      virtual void initialize(const std::deque<std::string> & arguments);

      /// Determine if the action can be executed by this implementation.
      /**
       * Default implementation only compares _actionName with the name of the DurativeAction.
       */
      virtual bool canExecute(const DurativeAction & a, const SymbolicState & current) const;

      /// Executes the action using SimpleActionClient.
      virtual bool executeBlocking(const DurativeAction & a, SymbolicState & current);

      /// Fill the goal to execute this action.
      /**
       * \param [out] goal the goal to be filled (i.e. the service's request part).
       * \param [in] a the DurativeAction that is to be executed.
       * \param [in] current the current state where a should be executd.
       */
      virtual bool fillGoal(typename Service::Request & goal,
             const DurativeAction & a, const SymbolicState & current) = 0;

      /// Update the state after an action was executed.
      /**
       * \param [in, out] success the return value of the service call,
       *    can be changed based on service response
       * \param [in] response what the call returned
       * \param [in] a the action that was executed
       * \param [in, out] current the current planner state to be updated
       */
      virtual void updateState(bool & success, typename Service::Response & response,
              const DurativeAction & a, SymbolicState & current) {ROS_INFO("ActionExecutorService: default implementation without state update.");}

      /// can't really do anything here
      virtual void cancelAction() {}

   protected:
      ros::NodeHandle* _nh;
      ros::ServiceClient _serviceClient;

      std::string _actionName;
};


template <class Service>
ActionExecutorService<Service>::ActionExecutorService() : _nh(NULL)
{
}

template <class Service>
ActionExecutorService<Service>::~ActionExecutorService()
{
    delete _nh;
}


template <class Service>
void ActionExecutorService<Service>::initialize(const std::deque<std::string> & arguments)
{
    ROS_ASSERT(arguments.size() >= 2);
    _actionName = arguments.at(0);
    ROS_INFO("Initializing ActionExecutor for action %s...", _actionName.c_str());

    std::string serviceName = arguments.at(1);
    _nh = new ros::NodeHandle();

    // wait for the service to come up
    while(!ros::service::waitForService(serviceName, ros::Duration(5.0))) {
        ROS_WARN("Service %s not available - waiting.", serviceName.c_str());
    }

    _serviceClient = _nh->serviceClient<Service>(serviceName);
    if(!_serviceClient) {
        ROS_FATAL("Could not initialize service from %s (client name: %s)",
                serviceName.c_str(), _serviceClient.getService().c_str());
    }

    ROS_INFO("Initialized ActionExecutor for action %s.", _actionName.c_str());
}

template <class Service>
bool ActionExecutorService<Service>::canExecute(
        const DurativeAction & a, const SymbolicState & current) const
{
    return (a.name == _actionName);
}

template <class Service>
bool ActionExecutorService<Service>::executeBlocking(const DurativeAction & a, SymbolicState & current)
{
    Service service;
    if(!fillGoal(service.request, a, current)) {
        return false;
    }

    if(!_serviceClient) {
        ROS_ERROR("Persistent service connection to %s failed.", _serviceClient.getService().c_str());

        // try to reconnect - this shouldn't happen.
        _serviceClient = _nh->serviceClient<Service>(_serviceClient.getService(), true);
        if(!_serviceClient) {
            ROS_FATAL("Could not reconnect service from %s", _serviceClient.getService().c_str());
            return false;
        }
    }

    ROS_INFO("Calling service for action: %s.", _actionName.c_str());

    // blocking call
    bool success = _serviceClient.call(service);

    updateState(success, service.response, a, current);

    if(success) {
        ROS_INFO("Reached goal for action: %s.", _actionName.c_str());
        return true;
    }

    ROS_INFO("Could not reach goal for action %s!", _actionName.c_str());
    return false;
}

#endif

