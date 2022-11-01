/**
 * @file flexiv_hardware_interface_node.cpp
 * Hardware interface node to run the control loop.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "flexiv_hardware/flexiv_hardware_interface.h"

std::unique_ptr<flexiv_hardware::FlexivHardwareInterface>
    flexiv_hardware_interface;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "flexiv_hardware_interface_node");
    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle robot_hw_nh("~");

    // Set up timers
    ros::Time timestamp;
    ros::Duration period;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;

    // Set up the hardware interface
    flexiv_hardware_interface.reset(
        new flexiv_hardware::FlexivHardwareInterface());
    if (!flexiv_hardware_interface->init(nh, robot_hw_nh)) {
        ROS_ERROR("Failed to initialize Flexiv Hardware Interface.");
        exit(1);
    }
    // Create the controller manager
    controller_manager::ControllerManager cm(
        flexiv_hardware_interface.get(), nh);

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(
        stopwatch_now - stopwatch_last)
                       .count());
    stopwatch_last = stopwatch_now;

    ros::Rate rate(1000.0);

    // Control loop - repeatedly calls read() and write() to the hardware
    // interface at a specified frequency
    while (ros::ok()) {
        // Read the current state from hardware
        flexiv_hardware_interface->read(timestamp, period);

        // Get current time and elapsed time since last read
        timestamp = ros::Time::now();
        stopwatch_now = std::chrono::steady_clock::now();
        period.fromSec(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                stopwatch_now - stopwatch_last)
                .count());
        stopwatch_last = stopwatch_now;

        // Update the controller manager
        cm.update(timestamp, period);

        // Write the hardware
        flexiv_hardware_interface->write(timestamp, period);
        // if (period.toSec() > 0.001) {
        //     ROS_INFO("Actual frequency: %.3f Hz", 1.0 / period.toSec());
        // }
        rate.sleep();
    }

    spinner.stop();
    ROS_INFO_STREAM_NAMED("flexiv_hardware_interface", "Shutting down.");
    return 0;
}
