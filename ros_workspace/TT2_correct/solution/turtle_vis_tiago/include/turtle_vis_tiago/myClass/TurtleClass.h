#ifndef TURTLECLASS_TIAGO_H
#define TURTLECLASS_TIAGO_H

/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <turtle_vis_tiago/DesiredPose.h>
#include <turtle_vis_tiago/send_desired_pose.h>

#include <nav_msgs/Odometry.h>              // added this line

/*********************************************************************
* EIGEN INCLUDES
********************************************************************/
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace Eigen;

namespace turtleSpace
{
    class TurtleClass
    {
    public:
        Vector3d turtlePose_g;
        Vector3d turtlePose_desired_g;
        pthread_mutex_t count_mutex;

        TurtleClass();
        ~TurtleClass();
        void getPose(const turtle_vis_tiago::DesiredPose::ConstPtr &msg);
        bool getDPose(turtle_vis_tiago::send_desired_pose::Request &req, turtle_vis_tiago::send_desired_pose::Response &res);
        Vector3d getLocalPose();
        Vector3d getLocalDesiredPose();

        void getTiagoPose(const nav_msgs::Odometry::ConstPtr &msg); // added this line
    };
}

#endif // TURTLECLASS_TIAGO_H
