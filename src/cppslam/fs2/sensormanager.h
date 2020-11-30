#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include "Eigen/Dense"
#include <math.h>
#include <vector>

#include "ros.h"
#include "mur_common/cone_msg.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"

#include "Observation.h"
#include "cone.h"


class sensormanager
{
    public:
        sensormanager();
    private:
        std::vector<Observation> z; 


};
#endif