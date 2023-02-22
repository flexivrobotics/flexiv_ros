/**
 * @file joint_impedance_controller.hpp
 * Joint impedance control as ROS controller. Adapted from
 * ros_controllers/effort_controllers
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_HPP_
#define FLEXIV_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

// ROS
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

// Flexiv

namespace flexiv_controllers {

class JointImpedanceController : public controller_interface::Controller<
                                     hardware_interface::EffortJointInterface>
{
public:
    // JointImpedanceController();
    // ~JointImpedanceController();

    bool init(
        hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh);

    void update(const ros::Time& time, const ros::Duration& period);

    void starting(const ros::Time& time);

    void setCommand(double pos_command);

    void setCommand(double pos_command, double vel_command);

private:
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<double> k_p_;
    std::vector<double> k_d_;
    std::array<double, 7> q_init_;
    static const size_t num_joints_ = 7;
};

} // namespace flexiv_controllers

#endif // FLEXIV_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_HPP_
