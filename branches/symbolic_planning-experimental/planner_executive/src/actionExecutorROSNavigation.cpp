#include "actionExecutorROSNavigation.h"

ActionExecutorROSNavigation::ActionExecutorROSNavigation(SymbolicState* current) : _currentState(current)
{
   _actionClient = new MoveBaseClient("move_base", true);
   // wait for the action server to come up
   while(!_actionClient->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }
}

ActionExecutorROSNavigation::~ActionExecutorROSNavigation()
{
   // TODO: stop and delete
}

bool ActionExecutorROSNavigation::canExecute(const DurativeAction & a) const
{
   // TODO: check this is the right kind of action and we do not have one running
   
   //return true;   // exec whole plan HACK

   return a.startTime < 1.0;
}

bool ActionExecutorROSNavigation::executeBlocking(const DurativeAction & a)
{
   if(_currentState == NULL)
      return false;

   move_base_msgs::MoveBaseGoal goal;

   //we'll send a goal to the robot to move 1 meter forward
   goal.target_pose.header.frame_id = "/map";      // TODO: whole thing tf frame resolv?
   goal.target_pose.header.stamp = ros::Time::now();

   ROS_ASSERT(a.parameters.size() == 2);
   string targetName = a.parameters[1];

   // extract nicer + warn.
   Predicate p;
   p.parameters.push_back(targetName);
   p.name = "x";
   if(!_currentState->hasNumericalFluent(p, &goal.target_pose.pose.position.x))
      return false;
   p.name = "y";
   if(!_currentState->hasNumericalFluent(p, &goal.target_pose.pose.position.y))
      return false;
   p.name = "z";
   if(!_currentState->hasNumericalFluent(p, &goal.target_pose.pose.position.z))
      return false;
   p.name = "qx";
   if(!_currentState->hasNumericalFluent(p, &goal.target_pose.pose.orientation.x))
      return false;
   p.name = "qy";
   if(!_currentState->hasNumericalFluent(p, &goal.target_pose.pose.orientation.y))
      return false;
   p.name = "qz";
   if(!_currentState->hasNumericalFluent(p, &goal.target_pose.pose.orientation.z))
      return false;
   p.name = "qw";
   if(!_currentState->hasNumericalFluent(p, &goal.target_pose.pose.orientation.w))
      return false;

   ROS_INFO("Sending goal");
   _actionClient->sendGoal(goal);

   // blocking call
   _actionClient->waitForResult();

   if(_actionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Reached move_base target.");
      return true;
   }

   ROS_INFO("Could not reach target!");
   return false;
}

