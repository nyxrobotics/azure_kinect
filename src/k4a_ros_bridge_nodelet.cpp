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
  timer_.stop();
  this->k4a_device.reset(nullptr);
  this->~Nodelet();
  // k4a_device.reset();
}

void K4AROSBridgeNodelet::watchdogTimerCallback(const ros::TimerEvent&)
{
  if (!k4a_device->isRunning())
  {
    NODELET_ERROR("K4A device is not running. Shutting down nodelet.");
    // timer_.stop();
    // this->k4a_device.reset(nullptr);
    // this->~Nodelet();
    // Terminate this nodelet without terminating the nodelet manager
    this->~K4AROSBridgeNodelet();
    // this->getMTPrivateNodeHandle().shutdown();
    // ros::requestShutdown();
    // throw nodelet::Exception("K4A device is not running. Shutting down nodelet.");
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

  timer_ = getNodeHandle().createTimer(ros::Duration(1.0), &K4AROSBridgeNodelet::watchdogTimerCallback, this);
  NODELET_INFO("Watchdog timer started");
}
}  // namespace Azure_Kinect_ROS_Driver
