// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_ROS_BRIDGE_NODELET_H
#define K4A_ROS_BRIDGE_NODELET_H

// System headers
//
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>
// Library headers
//
#include <ros/ros.h>
#include <nodelet/nodelet.h>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_device.h"

namespace Azure_Kinect_ROS_Driver
{
class K4AROSBridgeNodelet : public nodelet::Nodelet
{
public:
  K4AROSBridgeNodelet();
  ~K4AROSBridgeNodelet();

  virtual void onInit();

private:
  std::unique_ptr<K4AROSDevice> k4a_device;
  std::vector<std::string> getKinectPorts();
  void resetPorts(const std::vector<std::string>& ports);
};
}  // namespace Azure_Kinect_ROS_Driver

#endif  // K4A_ROS_BRIDGE_NODELET_H
