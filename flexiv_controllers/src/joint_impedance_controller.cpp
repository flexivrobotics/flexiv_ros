#include <flexiv_controllers/joint_impedance_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace flexiv_controllers {

JointImpedanceController::~JointImpedanceController()
{
    sub_command_.shutdown();
}

bool JointImpedanceController::init(
    hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
{
    // Get joint name from parameter server
    std::vector<std::string> joint_names;
    if (!nh.getParam("joints", joint_names)) {
        ROS_ERROR("JointImpedanceController: Could not parse joint names");
    }
    if (joint_names.size() != 7) {
        ROS_ERROR_STREAM(
            "JointImpedanceController: Wrong number of joint names, got "
            << joint_names.size() << " instead of 7 names!");
        return false;
    }

    // Get controller gains
    if (!nh.getParam("k_p", k_p_)) {
        ROS_ERROR("JointImpedanceController: No k_p parameters provided!");
        return false;
    }
    if (!nh.getParam("k_d", k_d_)) {
        ROS_ERROR("JointImpedanceController: No k_d parameters provided!");
        return false;
    }
    if (k_p_.size() != num_joints_) {
        ROS_ERROR("JointImpedanceController: k_p should be of size 7");
        return false;
    }
    if (k_d_.size() != num_joints_) {
        ROS_ERROR("JointImpedanceController: k_d should be of size 7");
        return false;
    }
    for (auto i = 0ul; i < k_p_.size(); i++) {
        if (k_p_[i] < 0 || k_d_[i] < 0) {
            ROS_ERROR("JointImpedanceController: Wrong impedance parameters!");
            return false;
        }
    }

    // Start command subscriber
    sub_command_ = nh.subscribe<flexiv_msgs::JointPosVel>(
        "command", 1, &JointImpedanceController::setCommandCB, this);

    // Get joint handles from hardware interface
    joints_.resize(num_joints_);
    for (std::size_t i = 0; i < num_joints_; ++i) {
        try {
            joints_[i] = hw->getHandle(joint_names[i]);
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
                "JointImpedanceController: Exception getting joint handles: "
                << e.what());
            return false;
        }
    }

    // Get URDF info about joints
    urdf::Model urdf;
    if (!urdf.initParamWithNodeHandle("robot_description", nh)) {
        ROS_ERROR("JointImpedanceController: Failed to parse urdf file");
        return false;
    }
    joints_urdf_.resize(num_joints_);
    for (std::size_t i = 0; i < num_joints_; ++i) {
        joints_urdf_[i] = urdf.getJoint(joint_names[i]);
        if (!joints_urdf_[i]) {
            ROS_ERROR(
                "JointImpedanceController: Could not find joint '%s' in urdf",
                joint_names[i].c_str());
            return false;
        }
    }

    return true;
}

void JointImpedanceController::setCommand(std::array<double, 7> pos_command)
{
    command_struct_.has_velocity_ = false;
    for (std::size_t i = 0; i < num_joints_; ++i) {
        command_struct_.positions_[i] = pos_command[i];
    }

    command_.writeFromNonRT(command_struct_);
}

void JointImpedanceController::setCommand(
    std::array<double, 7> pos_command, std::array<double, 7> vel_command)
{
    command_struct_.has_velocity_ = true;
    for (std::size_t i = 0; i < num_joints_; ++i) {
        command_struct_.positions_[i] = pos_command[i];
        command_struct_.velocities_[i] = pos_command[i];
    }

    command_.writeFromNonRT(command_struct_);
}

void JointImpedanceController::starting(const ros::Time& /*time*/)
{
    std::array<double, 7> pos_command;

    for (std::size_t i = 0; i < num_joints_; ++i) {
        pos_command[i] = joints_[i].getPosition();
        enforceJointLimits(i, pos_command[i]);
        command_struct_.positions_[i] = pos_command[i];
    }
    command_struct_.has_velocity_ = false;

    command_.initRT(command_struct_);
}

void JointImpedanceController::update(
    const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
    command_struct_ = *(command_.readFromRT());

    for (std::size_t i = 0; i < num_joints_; ++i) {
        // Get current joint positions and velocities
        double q = joints_[i].getPosition();
        double dq = joints_[i].getVelocity();

        // Get target positions and velocities
        double q_des = command_struct_.positions_[i];
        double dq_des = 0;
        if (command_struct_.has_velocity_) {
            dq_des = command_struct_.velocities_[i];
        }

        enforceJointLimits(i, q_des);

        // Compute torque
        double tau = k_p_[i] * (q_des - q) + k_d_[i] * (dq_des - dq);
        joints_[i].setCommand(tau);
    }
}

void JointImpedanceController::setCommandCB(
    const flexiv_msgs::JointPosVelConstPtr& msg)
{
    std::array<double, 7> command_positions;
    std::array<double, 7> command_velocities;
    if (msg->positions.size() == num_joints_
        && msg->velocities.size() == num_joints_) {
        memcpy(
            &command_positions, &msg->positions[0], sizeof(command_positions));
        memcpy(&command_velocities, &msg->velocities[0],
            sizeof(command_velocities));
        setCommand(command_positions, command_velocities);
    } else if (msg->positions.size() == num_joints_
               && msg->velocities.size() != num_joints_) {
        memcpy(
            &command_positions, &msg->positions[0], sizeof(command_positions));
        setCommand(command_positions);
    } else {
        ROS_ERROR(
            "JointImpedanceController: Invalid command message received!");
    }
}

void JointImpedanceController::enforceJointLimits(
    int joint_index, double& pos_command)
{
    if (joints_urdf_[joint_index]->type == urdf::Joint::REVOLUTE
        || joints_urdf_[joint_index]->type == urdf::Joint::PRISMATIC) {
        if (pos_command
            > joints_urdf_[joint_index]->limits->upper) // above upper limnit
        {
            pos_command = joints_urdf_[joint_index]->limits->upper;
        } else if (pos_command < joints_urdf_[joint_index]
                                     ->limits->lower) // below lower limit
        {
            pos_command = joints_urdf_[joint_index]->limits->lower;
        }
    }
}

} // namespace flexiv_controllers

PLUGINLIB_EXPORT_CLASS(flexiv_controllers::JointImpedanceController,
    controller_interface::ControllerBase)
