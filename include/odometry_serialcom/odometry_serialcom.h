#ifndef INCLUDE_EVAROBOT_ODOMETRY_H_
#define INCLUDE_EVAROBOT_ODOMETRY_H_

#include "ros/ros.h"
#include <realtime_tools/realtime_publisher.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "std_srvs/Empty.h"
#include <tf/transform_broadcaster.h>
//#include "im_msgs/WheelVel.h"


#include <string>    
#include <sstream>

#include <math.h>

#include <ros/console.h>
//#include <ErrorCodes.h>
//the serial port
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#define PI 3.1416

using namespace std;

#endif
