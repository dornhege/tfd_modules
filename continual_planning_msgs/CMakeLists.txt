cmake_minimum_required(VERSION 2.8.3)
project(continual_planning_msgs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)
add_message_files(
	FILES
	ContinualPlanningStatus.msg  
	TemporalAction.msg
)

add_service_files(
	FILES
	ExecuteActionDirectly.srv  
	SetContinualPlanningControl.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
