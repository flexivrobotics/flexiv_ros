#include <flexiv_controllers/joint_impedance_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace flexiv_controllers {

bool JointImpedanceController::init(
    hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
{
    // Get joint name from parameter server
    std::vector<std::string> joint_names;
    if (!nh.getParam("joint_names", joint_names)) {
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

    return true;
}

void JointImpedanceController::starting(const ros::Time& /*time*/)
{
    for (std::size_t i = 0; i < num_joints_; ++i) {
        q_init_[i] = joints_[i].getPosition();
    }
}

void JointImpedanceController::update(
    const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
    std::array<double, num_joints_> q_des = q_init_;

    for (std::size_t i = 0; i < num_joints_; ++i) {
        // Get current joint positions and velocities
        double q = joints_[i].getPosition();
        double dq = joints_[i].getVelocity();

        // Compute torque
        double tau = k_p_[i] * (q_des[i] - q) - k_d_[i] * (dq);
        joints_[i].setCommand(tau);
    }
}

} // namespace flexiv_controllers

PLUGINLIB_EXPORT_CLASS(flexiv_controllers::JointImpedanceController,
    controller_interface::ControllerBase)
