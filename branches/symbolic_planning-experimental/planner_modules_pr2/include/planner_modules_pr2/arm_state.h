/*
 * arm_state.h
 *
 *  Created on: 17 Jul 2012
 *      Author: andreas
 */

#ifndef ARM_STATE_H_
#define ARM_STATE_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


class ArmState
{
public:
    static std::string armDescriptionBaseParameter;
    static std::string armDescriptionArmLinksParameter;

private:
    std::string armName;
    sensor_msgs::JointState armState;

public:
    ArmState(const std::string& armName, const std::string& armStateParameter);
    const std::string& getArmName() const {return armName;}
    const sensor_msgs::JointState& getJointStates() const {return armState;}
    void replaceJointPositions(sensor_msgs::JointState& state) const;
    static void replaceJointPositions(sensor_msgs::JointState& state, const sensor_msgs::JointState& joints);
};



#endif /* ARM_STATE_H_ */
