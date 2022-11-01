/**
 * @file flexiv_hardware_interface.hpp
 * Hardware interface to Flexiv robots for ros_control.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE_FLEXIV_HARDWARE_INTERFACE_H
#define FLEXIV_HARDWARE_FLEXIV_HARDWARE_INTERFACE_H

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <realtime_tools/realtime_publisher.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// Flexiv
#include "flexiv/Robot.hpp"
#include "flexiv_msgs/ExternalForce.h"

namespace flexiv_hardware {
/**
 * @brief This class handles the interface between the ROS system and the
 * hardware via RDK.
 *
 */
class FlexivHardwareInterface : public hardware_interface::RobotHW
{
public:
    /**
     * @brief Construct a new FlexivHardwareInterface object
     *
     */
    FlexivHardwareInterface();

    virtual ~FlexivHardwareInterface() = default;

    /**
     * @brief Handles the setup functionality for the ROS interface. This
     * includes parsing ROS parameters, connecting to the robot, and setting up
     * hardware interfaces for ros_control.
     *
     * @param[in] root_nh Root level ROS node handle.
     * @param[in] robot_hw_nh ROS node handle for the robot namespace.
     * @return True if successful, false otherwise.
     */
    virtual bool init(
        ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

    /**
     * @brief Reads the parameterization of the hardware class from the ROS
     * parameter server (e.g. local_ip, robot_ip, joint_names etc.)
     *
     * @param[in] root_nh A node handle in the root namespace of the control
     * node.
     * @param[in] robot_hw_nh A node handle in the namespace of the robot
     * hardware.
     * @return True if successful, false otherwise.
     */
    virtual bool initParameters(
        ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);

    /**
     * @brief Initializes the class in terms of ros_control interfaces.
     *
     * @note You have to call initParameters beforehand. Use the complete
     * initialization routine \ref init() method to control robots.
     * @param[in] robot_hw_nh A node handle in the namespace of the robot
     * hardware.
     */
    virtual void initROSInterfaces(ros::NodeHandle& robot_hw_nh);

    /**
     * @brief Reads data from the robot.
     *
     * @param[in] time Current time.
     * @param[in] period Duration of current control loop iteration.
     */
    virtual void read(
        const ros::Time& time, const ros::Duration& period) override;

    /**
     * @brief Writes data to the robot.
     *
     * @param[in] time Current time.
     * @param[in] period Duration of current control loop iteration.
     */
    virtual void write(
        const ros::Time& time, const ros::Duration& period) override;

    /**
     * @brief Set all members to default values.
     *
     */
    virtual void reset();

    /**
     * @brief Prepares switching between controllers.
     *
     * @param[in] start_list List of requested controllers to start.
     * @param[in] stop_list List of requested controllers to stop.
     * @return True if the preparation has been successful, false otherwise.
     */
    virtual bool prepareSwitch(
        const std::list<hardware_interface::ControllerInfo>& start_list,
        const std::list<hardware_interface::ControllerInfo>& stop_list)
        override;

    /**
     * @brief Perform controllers switching.
     *
     * @param[in] start_list List of requested controllers to start.
     * @param[in] stop_list List of requested controllers to stop.
     */
    virtual void doSwitch(
        const std::list<hardware_interface::ControllerInfo>& start_list,
        const std::list<hardware_interface::ControllerInfo>& stop_list)
        override;

    /** @brief Enforce limits on position, velocity, and effort.
     *
     * @param[in] period The duration of the current cycle.
     */
    virtual void enforceLimits(const ros::Duration& period);

protected:
    /**
     * @brief Uses the robot_ip to connect to the robot via RDK.
     *
     * @return True if successful, false otherwise.
     */
    virtual bool initRobot();

    /**
     * @brief Get current joint position and set the joint command with those
     * values.
     *
     */
    virtual void setInitPosition();

    /**
     * @brief Configures a limit interface to enforce limits on effort, velocity
     * or position level on joint commands.
     *
     * @param[in] limits_interface Limit interface for the robot.
     * @param[in] command_interface  Command interface to match with the limit
     * interface.
     */
    template <typename T>
    void setupLimitInterface(
        joint_limits_interface::JointLimitsInterface<T>& limit_interface,
        hardware_interface::JointCommandInterface& command_interface)
    {
        joint_limits_interface::SoftJointLimits soft_limits;
        joint_limits_interface::JointLimits joint_limits;
        for (size_t i = 0; i < num_joints_; i++) {
            const std::string& joint_name = joint_names_[i];
            auto urdf_joint = urdf_model_.getJoint(joint_name);
            if (!urdf_joint || !urdf_joint->safety || !urdf_joint->limits) {
                ROS_WARN(
                    "Joint %s has incomplete limits and safety specs. Skipping "
                    "it in the joint "
                    "limit interface!",
                    joint_name.c_str());
                continue;
            }
            if (joint_limits_interface::getSoftJointLimits(
                    urdf_joint, soft_limits)) {
                if (joint_limits_interface::getJointLimits(
                        urdf_joint, joint_limits)) {
                    T limit_handle(command_interface.getHandle(joint_name),
                        joint_limits, soft_limits);
                    limit_interface.registerHandle(limit_handle);
                } else {
                    ROS_ERROR(
                        "Could not parse joint limits for joint %s for joint "
                        "limit interfaces",
                        joint_name.c_str());
                }
            } else {
                ROS_ERROR(
                    "Could not parse soft joint limits for joint %s for joint "
                    "limit interfaces",
                    joint_name.c_str());
            }
        }
    }

    /**
     * @brief Checks whether a resource list contains joints from this hardware
     * interface.
     *
     * @param[in] claimed_resources List of claimed resources.
     * @return True if the list contains joints from this hardware interface,
     * false otherwise.
     */
    virtual bool checkControllerClaims(
        const std::set<std::string>& claimed_resources);

    /**
     * @brief Publishes external force applied on TCP in TCP frame \f$
     * ^{TCP}F_{ext}~[N][Nm] \f$ and base frame \f$ ^{0}F_{ext}~[N][Nm] \f$.
     *
     */
    virtual void publishExternalForce();

    // Hardware interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;

    // Joint limits interfaces
    joint_limits_interface::PositionJointSoftLimitsInterface
        position_joint_limit_interface_;
    joint_limits_interface::VelocityJointSoftLimitsInterface
        velocity_joint_limit_interface_;
    joint_limits_interface::EffortJointSoftLimitsInterface
        effort_joint_limit_interface_;

    // Configuration
    std::unique_ptr<flexiv::Robot> robot_;
    std::string robot_ip_;
    std::string local_ip_;
    std::vector<std::string> joint_names_;
    static const size_t num_joints_ = 7;
    urdf::Model urdf_model_;

    // States
    std::vector<double> joint_position_state_;
    std::vector<double> joint_velocity_state_;
    std::vector<double> joint_effort_state_;
    // External force
    std::vector<double> ext_force_in_tcp_;
    std::vector<double> ext_force_in_base_;

    // Commands
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_effort_command_;
    std::vector<double> internal_joint_position_command_;

    // Controller
    bool position_controller_running_;
    bool velocity_controller_running_;
    bool effort_controller_running_;
    bool controllers_initialized_;

    // Publisher
    std::unique_ptr<
        realtime_tools::RealtimePublisher<flexiv_msgs::ExternalForce>>
        ext_force_in_tcp_pub_;
    std::unique_ptr<
        realtime_tools::RealtimePublisher<flexiv_msgs::ExternalForce>>
        ext_force_in_base_pub_;
};
} // namespace flexiv_hardware

#endif // FLEXIV_HARDWARE_FLEXIV_HARDWARE_INTERFACE_H
