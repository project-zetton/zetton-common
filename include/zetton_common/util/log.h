#pragma once

#include "ros/ros.h"

// TODO(corenel) add definition to choose logger from ROS, glog or custom

#define Z_SUCCESS(msg) ROS_INFO("[Success] " + ##msg)
#define Z_FAILED(msg) ROS_WARN("[Failed] " + ##msg)
