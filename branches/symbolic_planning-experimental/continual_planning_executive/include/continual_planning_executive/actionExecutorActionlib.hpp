#ifndef ACTION_EXECUTOR_ACTIONLIB_H
#define ACTION_EXECUTOR_ACTIONLIB_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"
#include <actionlib/client/simple_action_client.h>

/// Templated base class for creating action executors for actionlib actions using SimpleActionClient.
/**
 * The class is templated over the action (e.g. move_base_msgs::MoveBaseAction), the
 * corresponding goal (e.g. move_base_msgs::MoveBaseGoal) and the matching result.
 *
 * A minimal implementation should derive from this class (instantiating the template properly)
 * and implement fillGoal.
 *
 * canExecute can be overridden to test more specific than just the name.
 * updateState can be used to update the planner state upon execution depending on the result
 * of the execution.
 */
template <class Action, class ActionGoal, class ActionResult>
class ActionExecutorActionlib : public continual_planning_executive::ActionExecutorInterface
{
   public:
      ActionExecutorActionlib();
      ~ActionExecutorActionlib();


      /// Initialize the executor by creating the ActionClient.
      /**
       * \param [in] arguments should be name of action (as in the plan),
       *    and the name of the action server to connect to,
       *    e.g. "driveBase move_base"
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
       * \param [out] goal the ActionGoal to be filled.
       * \param [in] a the DurativeAction that is to be executed.
       * \param [in] current the current state where a should be executd.
       */
      virtual bool fillGoal(ActionGoal & goal, const DurativeAction & a, const SymbolicState & current) = 0;

      /// Update the state after an action was executed.
      /**
       * \param [in] actionReturnState the state of the executed action
       * \param [in] result the result returned by the action
       * \param [in, out] current the current planner state to be updated
       */
      virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState, const ActionResult & result,
              const DurativeAction & a, SymbolicState & current) {}

      /// just cancelAllGoals
      virtual void cancelAction();

   protected:
      typedef actionlib::SimpleActionClient<Action> ActionClient;
      ActionClient* _actionClient;

      std::string _actionName;
};


template <class Action, class ActionGoal, class ActionResult>
ActionExecutorActionlib<Action, ActionGoal, ActionResult>::ActionExecutorActionlib() : _actionClient(NULL)
{
}

template <class Action, class ActionGoal, class ActionResult>
ActionExecutorActionlib<Action, ActionGoal, ActionResult>::~ActionExecutorActionlib()
{
    _actionClient->cancelAllGoals();

    delete _actionClient;
}


template <class Action, class ActionGoal, class ActionResult>
void ActionExecutorActionlib<Action, ActionGoal, ActionResult>::initialize(const std::deque<std::string> & arguments)
{
    ROS_ASSERT(arguments.size() >= 2);
    _actionName = arguments.at(0);
    ROS_INFO("Initializing ActionExecutor for action %s...", _actionName.c_str());

    std::string actionServeName = arguments.at(1);

    _actionClient = new ActionClient(actionServeName, true);
    ROS_INFO("Created action client.");
    // wait for the action server to come up
    while(!_actionClient->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the %s action server to come up...", actionServeName.c_str());
    }
    ROS_INFO("Initialized ActionExecutor for action %s.", _actionName.c_str());
}

template <class Action, class ActionGoal, class ActionResult>
bool ActionExecutorActionlib<Action, ActionGoal, ActionResult>::canExecute(
        const DurativeAction & a, const SymbolicState & current) const
{
    return (a.name == _actionName);
}

template <class Action, class ActionGoal, class ActionResult>
bool ActionExecutorActionlib<Action, ActionGoal, ActionResult>::executeBlocking(const DurativeAction & a, SymbolicState & current)
{
    ActionGoal goal;
    if(!fillGoal(goal, a, current)) {
        return false;
    }

    ROS_INFO("Sending goal for action: %s.", _actionName.c_str());
    _actionClient->sendGoal(goal);

    // blocking call
    _actionClient->waitForResult();

    updateState(_actionClient->getState(), *(_actionClient->getResult()), a, current);

    if(_actionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Reached goal for action: %s.", _actionName.c_str());
        return true;
    }

    ROS_INFO("Could not reach goal for action %s! Resulting action state: %s.",
           _actionName.c_str(), _actionClient->getState().toString().c_str());
    return false;
}

template <class Action, class ActionGoal, class ActionResult>
void ActionExecutorActionlib<Action, ActionGoal, ActionResult>::cancelAction()
{
    _actionClient->cancelAllGoals();
}

#endif

