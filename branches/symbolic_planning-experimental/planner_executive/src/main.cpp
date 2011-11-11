#include <stdio.h>
#include <ros/ros.h>

#include "symbolicState.h"
#include "stateCreatorROSNavigation.h"
#include "plannerTFDM.h"
#include "hardcoded_facts/geometryPoses.h"
#include "planExecutorROSNavigation.h"

static StateCreatorROSNavigation* s_StateCreator = NULL;
static PlannerTFDM* s_Planner = NULL;
static PlanExecutorROSNavigation* s_PlanExecutor = NULL;

static SymbolicState s_CurrentState;
static SymbolicState s_Goal;
static Plan s_CurrentPlan;

// constantly do the following:
// estimate current state
// check that curState + curPlan will lead to goal
// if not:
//    replan


// executros
//  check/keep running actions
//  preempt if conflicting? planner always assumes no running actions?
//  should never decide: oops s.th. changed lets redo everything
//  -- make monitoring nice/safe

//  Plan monitoring does not work, if move_base/make_plan fails during movement
//  also: Execution failures should trigger replan somehow: i.e. drive to gap -> fail -> replan
//  probably need partial/subgoal plan then to get away and make new situation

// cehck contiplan paper

bool needReplanning(const SymbolicState & current, const SymbolicState & goal, const Plan & currentPlan)
{
   static SymbolicState lastReplanState;

   if(currentPlan.empty()) {
      lastReplanState = current;
      return true;
   }

   // FIXME call monitoring and check for assertions
   if(current.booleanEquals(lastReplanState))
         return false;
   
   ROS_INFO("state changed since last replanning.");
   lastReplanState = current;

   // later: app(current, currentPlan) != goal

   // yeah...
   return true;
}

Plan monitorAndReplan(const SymbolicState & current, const SymbolicState & goal, const Plan & currentPlan)
{
   if(!needReplanning(current, goal, currentPlan)) {
      return currentPlan;
   }

   // fancy prefix stuff

   // REPLAN
   Plan plan;
   PlannerInterface::PlannerResult result = s_Planner->plan(current, goal, plan);
   if(result == PlannerInterface::PR_SUCCESS || result == PlannerInterface::PR_SUCCESS_TIMEOUT) {
      ROS_INFO("Planning successfull.");
   } else {
      ROS_WARN("Planning failed, result: %s", PlannerInterface::PlannerResultStr(result).c_str());
   }
   return plan;
}

bool estimateCurrentState(SymbolicState & state)
{
   bool ret = s_StateCreator->fillState(state);
   ROS_INFO_STREAM("Current state is: " << state);
   return ret;
}

/// The main continual planning loop.
/**
 * \returns true, if continual planning has to continue
 */
bool continualPlanningLoop()
{
   if(!estimateCurrentState(s_CurrentState)) {
      ROS_WARN("State estimation failed.");     // FIXME: continue execution until this works?
      return true;
   }

   if(s_Goal.isFulfilledBy(s_CurrentState)) {
      return false;
   }

   Plan newPlan = monitorAndReplan(s_CurrentState, s_Goal, s_CurrentPlan);
   if(newPlan.empty()) {
      ROS_ERROR("Empty plan returned by monitorAndReplan.");
      return false;           // can't do anything
   }

   // exec plan
   s_CurrentPlan = newPlan;

   // TODO: just send and remember actions
   // supervise those while running and estimating state.
   s_PlanExecutor->executeBlocking(s_CurrentPlan);

   return true;
}

bool init()
{
   ros::NodeHandle nhPriv("~");
   // get domain, goal
   std::string domainFile;
   if(!nhPriv.getParam("domain_file", domainFile)) {
      ROS_ERROR("Could not get ~domain_file parameter.");
      return false;
   }
   std::string goalLocationsFile;
   if(!nhPriv.getParam("goal_locations", goalLocationsFile)) {
      ROS_ERROR("Could not get ~goal_locations parameter.");
      return false;
   }

   // planner interface
   s_Planner = new PlannerTFDM(domainFile);
   s_Planner->setModuleOptions("(navstack_init@libplanner_modules_pr2.so)");
   s_Planner->setTimeout(60.0);

   // init/goal
   GeometryPoses goalLocations;
   if(!goalLocations.load(goalLocationsFile)) {
      ROS_ERROR("Could not load goal locations from \"%s\".", goalLocationsFile.c_str());
      return false;
   }
   const std::map<std::string, geometry_msgs::Pose> & goalPoses = goalLocations.getPoses();
   forEach(const GeometryPoses::NamedPose & np, goalPoses) {
      s_CurrentState.addObject(np.first, "target");
      s_Goal.addObject(np.first, "target");

      s_CurrentState.setNumericalFluent("x", np.first, np.second.position.x);
      s_CurrentState.setNumericalFluent("y", np.first, np.second.position.y);
      s_CurrentState.setNumericalFluent("z", np.first, np.second.position.z);
      s_CurrentState.setNumericalFluent("qx", np.first, np.second.orientation.x);
      s_CurrentState.setNumericalFluent("qy", np.first, np.second.orientation.y);
      s_CurrentState.setNumericalFluent("qz", np.first, np.second.orientation.z);
      s_CurrentState.setNumericalFluent("qw", np.first, np.second.orientation.w);
      s_CurrentState.setBooleanPredicate("explored", np.first.c_str(), false);
      s_Goal.setBooleanPredicate("explored", np.first.c_str(), true);
   }

   ROS_INFO_STREAM("Goal initialized to:\n" << s_Goal);

   s_StateCreator = new StateCreatorROSNavigation();

   s_PlanExecutor = new PlanExecutorROSNavigation(&s_CurrentState);
   s_PlanExecutor->createActionExecutors();
   
   return (s_StateCreator != NULL && s_Planner != NULL);
}

int main(int argc, char** argv)
{
   ROS_INFO("Planner Executive started.");

   ros::init(argc, argv, "planner_executive");

   ros::NodeHandle nh;

   if(!init()) {
      ROS_FATAL("Init failed.");
      return 1;
   }

   ros::Rate loopSleep(5);
   while(ros::ok()) {
      ros::spinOnce();

      if(!continualPlanningLoop()) {
         break;
      }

      loopSleep.sleep();
   }

   if(s_Goal.isFulfilledBy(s_CurrentState)) {
      if(ros::ok())
         ROS_INFO("Continual planning ended.\nGoal reached by agent!");
      else
         printf("Continual planning ended.\nGoal reached by agent!\n");
   } else {
      if(ros::ok())
         ROS_ERROR("Continual planning ended.\nGoal was not reached.");
      else
         printf("Continual planning ended.\nGoal was not reached.\n");
   }

   return 0;
}

