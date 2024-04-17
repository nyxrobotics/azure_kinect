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
  // k4a_device.reset();
}

void K4AROSBridgeNodelet::onInit()
{
  NODELET_INFO("K4A ROS Nodelet Start");
  // Get USB port number of the knect device connected
  std::vector<std::string> usb_device_ports = getKinectPorts();
  ROS_INFO_STREAM("-----Found " << usb_device_ports.size() << " Azure Kinect devices");

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

  // Running nodelet
  while (ros::ok())
  {
    if (!k4a_device->isRunning())
    {
      throw nodelet::Exception("K4A ROS Nodelet Stop");
    }
  }
  // Reset the USB port of the kinect device
  resetPorts(usb_device_ports);
  ROS_INFO("-----K4A ROS Nodelet Stop");
}

std::vector<std::string> K4AROSBridgeNodelet::getKinectPorts()
{
  std::vector<std::string> usb_device_ports;

  // Set alarm for timeout
  alarm(5);

  // Run lsusb command and parse its output
  FILE* lsusbPipe = popen("lsusb -v", "r");
  if (!lsusbPipe)
  {
    throw nodelet::Exception("Error running lsusb command");
  }

  char line[1024];
  std::string devicePath;
  while (fgets(line, sizeof(line), lsusbPipe) != nullptr)
  {
    // Check if this line contains "iProduct" field
    if (strstr(line, "iProduct") != nullptr)
    {
      // Check if the line contains "Azure Kinect"
      if (strstr(line, "Azure Kinect") != nullptr)
      {
        // Extract the device path from the previous lines
        if (!devicePath.empty())
        {
          // Store the USB device port for later use
          usb_device_ports.push_back(devicePath);
          devicePath.clear();
        }
      }
    }
    else if (strstr(line, "Bus") != nullptr && strstr(line, "Device") != nullptr)
    {
      // Extract the device path from the line
      char* colonPos = strchr(line, ':');
      if (colonPos != nullptr)
      {
        // Extract bus and device numbers
        std::string busDeviceString(colonPos + 2, 3);
        devicePath = "/dev/bus/usb/" + busDeviceString + "/";
      }
    }
  }
  pclose(lsusbPipe);

  // Cancel alarm
  alarm(0);

  return usb_device_ports;
}

void K4AROSBridgeNodelet::resetPorts(const std::vector<std::string>& ports)
{
  // Reset USB devices containing "Azure Kinect" in their description
  for (const auto& devicePath : ports)
  {
    int fd = open(devicePath.c_str(), O_WRONLY);
    if (fd != -1)
    {
      struct usbdevfs_ioctl command;
      command.ifno = 0;  // Interface number (usually 0)
      command.ioctl_code = USBDEVFS_DISCONNECT;
      if (ioctl(fd, USBDEVFS_IOCTL, &command) == -1)
      {
        NODELET_WARN("Error disconnecting USB device: %s", strerror(errno));
      }
      else
      {
        NODELET_INFO("USB device reset: %s", devicePath.c_str());
      }
      close(fd);
    }
    else
    {
      NODELET_WARN("Error opening USB device: %s", strerror(errno));
    }
  }
}
}  // namespace Azure_Kinect_ROS_Driver
