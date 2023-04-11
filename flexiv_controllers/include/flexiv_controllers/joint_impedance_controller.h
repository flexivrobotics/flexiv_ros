/**
 * @file joint_impedance_controller.hpp
 * @brief Joint impedance control as ROS controller. Adapted from
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
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <flexiv_msgs/JointPosVel.h>
#include <ros/time.h>
#include <urdf/model.h>

// Flexiv

namespace flexiv_controllers {

class JointImpedanceController : public controller_interface::Controller<
                                     hardware_interface::EffortJointInterface>
{
public:
    struct Commands
    {
        std::array<double, 7> positions_; // Last commanded position
        std::array<double, 7> velocities_; // Last commanded velocity
        bool has_velocity_; // false if no velocity command has been specified
    };

    JointImpedanceController() = default;
    ~JointImpedanceController();

    bool init(
        hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh);

    void update(const ros::Time& time, const ros::Duration& period);

    void starting(const ros::Time& time);

    void setCommand(std::array<double, 7> pos_command);

    void setCommand(
        std::array<double, 7> pos_command, std::array<double, 7> vel_command);

private:
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<urdf::JointConstSharedPtr> joints_urdf_;
    std::vector<double> k_p_;
    std::vector<double> k_d_;
    static const size_t num_joints_ = 7;

    // Commands
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
    ros::Subscriber sub_command_;

    void setCommandCB(const flexiv_msgs::JointPosVelConstPtr& msg);

    void enforceJointLimits(int joint_index, double& pos_command);
};

} // namespace flexiv_controllers

#endif // FLEXIV_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_HPP_
