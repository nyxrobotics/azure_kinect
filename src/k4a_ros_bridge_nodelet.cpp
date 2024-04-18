// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/k4a_ros_bridge_nodelet.h"

// System headers
//

// Library headers
//
#include <pluginlib/class_list_macros.h>

// Project headers
//

PLUGINLIB_EXPORT_CLASS(Azure_Kinect_ROS_Driver::K4AROSBridgeNodelet, nodelet::Nodelet)

namespace Azure_Kinect_ROS_Driver
{
K4AROSBridgeNodelet::K4AROSBridgeNodelet() : Nodelet(), k4a_device(nullptr)
{
}

K4AROSBridgeNodelet::~K4AROSBridgeNodelet()
{
  this->k4a_device.reset(nullptr);
}

void K4AROSBridgeNodelet::watchdogTimerCallback(const ros::TimerEvent&)
{
  if (!k4a_device->isRunning() || !ros::ok() || ros::isShuttingDown())
  {
    timer_.stop();
    NODELET_ERROR("K4A device is not running. Restarting nodelet.");
    // restartNodelet();
    this->k4a_device.reset(nullptr);
    sleep(3);
    this->onInit();
    // timer_.start();
    // this->~K4AROSBridgeNodelet();
    // ros::shutdown();
    // throw nodelet::Exception("K4A device is not running. Shutting down nodelet.");
  }
}

void K4AROSBridgeNodelet::restartNodelet()
{
  // Unload and reload the nodelet to restart it
  nodelet::Loader manager;
  std::string nodelet_name = getName();                              // Get the current nodelet's name
  std::string type = "Azure_Kinect_ROS_Driver/K4AROSBridgeNodelet";  // Get the current nodelet's type

  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  // Unload and reload the nodelet
  if (!manager.unload(nodelet_name) || !manager.load(nodelet_name, type, remappings, my_argv))
  {
    NODELET_FATAL("Failed to restart the nodelet.");
  }
}

void K4AROSBridgeNodelet::onInit()
{
  NODELET_INFO("K4A ROS Nodelet Start");

  k4a_device = std::unique_ptr<K4AROSDevice>(new K4AROSDevice(getNodeHandle(), getPrivateNodeHandle()));
  if (k4a_device->startCameras() != K4A_RESULT_SUCCEEDED)
  {
    k4a_device.reset(nullptr);
    throw nodelet::Exception("Could not start K4A_ROS_Device!");
  }
  NODELET_INFO("Cameras started");

  if (k4a_device->startImu() != K4A_RESULT_SUCCEEDED)
  {
    k4a_device.reset(nullptr);
    throw nodelet::Exception("Could not start K4A_ROS_Device!");
  }
  NODELET_INFO("IMU started");

  timer_ = getNodeHandle().createTimer(ros::Duration(0.1), &K4AROSBridgeNodelet::watchdogTimerCallback, this);
  NODELET_INFO("Watchdog timer started");
}
}  // namespace Azure_Kinect_ROS_Driver
