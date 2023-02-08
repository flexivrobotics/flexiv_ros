/**
 * @file flexiv_hardware_interface.cpp
 * Hardware interface to Flexiv robots for ros_control.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <vector>
#include <string>
#include <thread>

#include "flexiv/Robot.hpp"
#include "flexiv/Exception.hpp"
#include "flexiv_hardware/flexiv_hardware_interface.h"

namespace flexiv_hardware {
FlexivHardwareInterface::FlexivHardwareInterface() { }

bool FlexivHardwareInterface::init(
    ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    // States
    joint_position_state_.resize(num_joints_, 0.0);
    joint_velocity_state_.resize(num_joints_, 0.0);
    joint_effort_state_.resize(num_joints_, 0.0);

    // External force
    ext_force_in_tcp_.resize(6, 0.0);
    ext_force_in_base_.resize(6, 0.0);

    // Commands
    joint_position_command_.resize(num_joints_, 0.0);
    joint_velocity_command_.resize(num_joints_, 0.0);
    joint_effort_command_.resize(num_joints_, 0.0);
    internal_joint_position_command_.resize(num_joints_, 0.0);

    // Controller
    position_controller_running_ = false;
    velocity_controller_running_ = false;
    effort_controller_running_ = false;
    controllers_initialized_ = false;

    if (!initParameters(root_nh, robot_hw_nh)) {
        ROS_ERROR("Failed to parse all required parameters.");
        return false;
    }

    if (!initRobot()) {
        ROS_ERROR("Failed to initialize robot.");
        return false;
    }
    initROSInterfaces(robot_hw_nh);

    // Publisher
    ext_force_in_tcp_pub_.reset(
        new realtime_tools::RealtimePublisher<flexiv_msgs::ExternalForce>(
            root_nh, "external_force_in_tcp", 1));
    ext_force_in_base_pub_.reset(
        new realtime_tools::RealtimePublisher<flexiv_msgs::ExternalForce>(
            root_nh, "external_force_in_base", 1));

    ROS_INFO_STREAM_NAMED(
        "flexiv_hardware_interface", "Loaded Flexiv Hardware Interface.");

    return true;
}

bool FlexivHardwareInterface::initParameters(
    ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    // names of the joints given in the controller config file
    if (!robot_hw_nh.getParam("joints", joint_names_)
        || joint_names_.size() != num_joints_) {
        ROS_ERROR("No joints names found on param server");
        return false;
    }

    if (!urdf_model_.initParamWithNodeHandle("robot_description", root_nh)) {
        ROS_ERROR("Could not initialize URDF model from robot_description");
        return false;
    }

    if (!robot_hw_nh.getParam("robot_ip", robot_ip_)) {
        ROS_ERROR("No robot_ip found on param server");
        return false;
    }

    if (!robot_hw_nh.getParam("local_ip", local_ip_)) {
        ROS_ERROR("No local_ip found on param server");
        return false;
    }

    return true;
}

void FlexivHardwareInterface::initROSInterfaces(
    ros::NodeHandle& /*robot_hw_nh*/)
{
    for (std::size_t i = 0; i < num_joints_; ++i) {
        // Create joint state interface for all joints
        joint_state_interface_.registerHandle(
            hardware_interface::JointStateHandle(joint_names_[i],
                &joint_position_state_[i], &joint_velocity_state_[i],
                &joint_effort_state_[i]));

        // Add command interfaces to joints
        hardware_interface::JointHandle joint_handle_position
            = hardware_interface::JointHandle(
                joint_state_interface_.getHandle(joint_names_[i]),
                &joint_position_command_[i]);
        position_joint_interface_.registerHandle(joint_handle_position);

        hardware_interface::JointHandle joint_handle_velocity
            = hardware_interface::JointHandle(
                joint_state_interface_.getHandle(joint_names_[i]),
                &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(joint_handle_velocity);

        hardware_interface::JointHandle joint_handle_effort
            = hardware_interface::JointHandle(
                joint_state_interface_.getHandle(joint_names_[i]),
                &joint_effort_command_[i]);
        effort_joint_interface_.registerHandle(joint_handle_effort);
    }

    // Load the joint limits
    setupLimitInterface<joint_limits_interface::PositionJointSoftLimitsHandle>(
        position_joint_limit_interface_, position_joint_interface_);
    setupLimitInterface<joint_limits_interface::VelocityJointSoftLimitsHandle>(
        velocity_joint_limit_interface_, velocity_joint_interface_);
    setupLimitInterface<joint_limits_interface::EffortJointSoftLimitsHandle>(
        effort_joint_limit_interface_, effort_joint_interface_);

    registerInterface(&joint_state_interface_); // From RobotHW base class.
    registerInterface(&position_joint_interface_); // From RobotHW base class.
    registerInterface(&velocity_joint_interface_); // From RobotHW base class.
    registerInterface(&effort_joint_interface_); // From RobotHW base class.
}

bool FlexivHardwareInterface::initRobot()
{
    // Connect to the robot
    try {
        robot_ = std::make_unique<flexiv::Robot>(robot_ip_, local_ip_);
        ;
    } catch (const flexiv::Exception& e) {
        ROS_ERROR("Failed to connect to robot: %s", e.what());
        return false;
    }

    // Enable the robot, make sure the E-stop is released before enabling
    try {
        robot_->enable();
    } catch (const flexiv::Exception& e) {
        ROS_ERROR("Could not enable robot: %s", e.what());
        return false;
    }

    // Wait for the robot to become operational
    while (!robot_->isOperational()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ROS_INFO("Robot is now operational.");

    // get current position and set to initial position
    setInitPosition();

    return true;
}

void FlexivHardwareInterface::enforceLimits(const ros::Duration& period)
{
    if (period.toSec() > 0.0) {
        position_joint_limit_interface_.enforceLimits(period);
        velocity_joint_limit_interface_.enforceLimits(period);
        effort_joint_limit_interface_.enforceLimits(period);
    }
}

void FlexivHardwareInterface::publishExternalForce()
{
    if (ext_force_in_tcp_pub_) {
        if (ext_force_in_tcp_pub_->trylock()) {
            ext_force_in_tcp_pub_->msg_.force.x = ext_force_in_tcp_[0];
            ext_force_in_tcp_pub_->msg_.force.y = ext_force_in_tcp_[1];
            ext_force_in_tcp_pub_->msg_.force.z = ext_force_in_tcp_[2];
            ext_force_in_tcp_pub_->msg_.moment.x = ext_force_in_tcp_[3];
            ext_force_in_tcp_pub_->msg_.moment.y = ext_force_in_tcp_[4];
            ext_force_in_tcp_pub_->msg_.moment.z = ext_force_in_tcp_[5];
            ext_force_in_tcp_pub_->unlockAndPublish();
        }
    }
    if (ext_force_in_base_pub_) {
        if (ext_force_in_base_pub_->trylock()) {
            ext_force_in_base_pub_->msg_.force.x = ext_force_in_base_[0];
            ext_force_in_base_pub_->msg_.force.y = ext_force_in_base_[1];
            ext_force_in_base_pub_->msg_.force.z = ext_force_in_base_[2];
            ext_force_in_base_pub_->msg_.moment.x = ext_force_in_base_[3];
            ext_force_in_base_pub_->msg_.moment.y = ext_force_in_base_[4];
            ext_force_in_base_pub_->msg_.moment.z = ext_force_in_base_[5];
            ext_force_in_base_pub_->unlockAndPublish();
        }
    }
}

void FlexivHardwareInterface::read(
    const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
    flexiv::RobotStates robot_states;

    // Read the current joint positions
    if (robot_->isOperational() && robot_->getMode() != flexiv::MODE_IDLE) {
        robot_->getRobotStates(robot_states);
        joint_position_state_ = robot_states.q;
        joint_velocity_state_ = robot_states.dtheta;
        joint_effort_state_ = robot_states.tau;

        ext_force_in_tcp_ = robot_states.extWrenchInTcp;
        ext_force_in_base_ = robot_states.extWrenchInBase;

        internal_joint_position_command_ = joint_position_state_;
    }

    publishExternalForce();
}

void FlexivHardwareInterface::write(
    const ros::Time& /*time*/, const ros::Duration& period)
{
    // Enforce safety joint limits for all registered handles
    enforceLimits(period);

    std::vector<double> target_acceleration(num_joints_, 0);
    std::vector<double> target_velocity(num_joints_, 0);
    std::fill(target_acceleration.begin(), target_acceleration.end(), 0.0);
    std::fill(target_velocity.begin(), target_velocity.end(), 0.0);

    if (position_controller_running_
        && robot_->getMode() == flexiv::MODE_JOINT_POSITION) {
        robot_->streamJointPosition(
            joint_position_command_, target_velocity, target_acceleration);
    } else if (velocity_controller_running_
               && robot_->getMode() == flexiv::MODE_JOINT_POSITION) {
        for (std::size_t i = 0; i < num_joints_; i++) {
            internal_joint_position_command_[i]
                += joint_velocity_command_[i] * period.toSec();
        }
        robot_->streamJointPosition(internal_joint_position_command_,
            target_velocity, target_acceleration);
    } else if (effort_controller_running_
               && robot_->getMode() == flexiv::MODE_JOINT_TORQUE) {
        robot_->streamJointTorque(joint_effort_command_);
    }
}

void FlexivHardwareInterface::setInitPosition()
{
    flexiv::RobotStates robot_states;

    // Read the current joint positions
    if (robot_->isOperational()) {
        robot_->getRobotStates(robot_states);
        joint_position_state_ = robot_states.q;
    }

    // set the initial joint position as command
    joint_position_command_ = joint_position_state_;
}

void FlexivHardwareInterface::reset()
{
    // Reset joint limits state, in case of mode switch or e-stop
    position_joint_limit_interface_.reset();
}

bool FlexivHardwareInterface::checkControllerClaims(
    const std::set<std::string>& claimed_resources)
{
    for (const std::string& it : joint_names_) {
        for (const std::string& resource : claimed_resources) {
            if (it == resource) {
                return true;
            }
        }
    }
    return false;
}

bool FlexivHardwareInterface::prepareSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list,
    const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    if (controllers_initialized_ && !start_list.empty()) {
        for (auto& controller : start_list) {
            if (!controller.claimed_resources.empty()) {
                ROS_ERROR(
                    "FlexivHardwareInterface: Cannot start controller %s, "
                    "because it "
                    "claims resources that are already claimed by another "
                    "controller.",
                    controller.name.c_str());
                return false;
            }
        }
    }

    controllers_initialized_ = true;
    ROS_INFO("Controllers are now initialized.");
    return true;
}

void FlexivHardwareInterface::doSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list,
    const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    for (auto& controller_it : stop_list) {
        for (auto& resource_it : controller_it.claimed_resources) {
            if (checkControllerClaims(resource_it.resources)) {
                if (resource_it.hardware_interface
                    == "hardware_interface::PositionJointInterface") {
                    position_controller_running_ = false;
                } else if (resource_it.hardware_interface
                           == "hardware_interface::VelocityJointInterface") {
                    velocity_controller_running_ = false;
                } else if (resource_it.hardware_interface
                           == "hardware_interface::EffortJointInterface") {
                    effort_controller_running_ = false;
                }
            }
        }
    }
    for (auto& controller_it : start_list) {
        for (auto& resource_it : controller_it.claimed_resources) {
            if (checkControllerClaims(resource_it.resources)) {
                if (resource_it.hardware_interface
                    == "hardware_interface::PositionJointInterface") {
                    velocity_controller_running_ = false;
                    effort_controller_running_ = false;
                    position_controller_running_ = true;
                    if (robot_->getMode() != flexiv::MODE_JOINT_POSITION) {
                        robot_->setMode(flexiv::MODE_JOINT_POSITION);
                    }
                } else if (resource_it.hardware_interface
                           == "hardware_interface::VelocityJointInterface") {
                    position_controller_running_ = false;
                    effort_controller_running_ = false;
                    velocity_controller_running_ = true;
                    if (robot_->getMode() != flexiv::MODE_JOINT_POSITION) {
                        robot_->setMode(flexiv::MODE_JOINT_POSITION);
                    }
                } else if (resource_it.hardware_interface
                           == "hardware_interface::EffortJointInterface") {
                    position_controller_running_ = false;
                    velocity_controller_running_ = false;
                    effort_controller_running_ = true;
                    if (robot_->getMode() != flexiv::MODE_JOINT_TORQUE) {
                        robot_->setMode(flexiv::MODE_JOINT_TORQUE);
                    }
                }
            }
        }
    }
}

} // namespace flexiv_hardware
