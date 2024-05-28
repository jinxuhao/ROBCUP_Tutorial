/*********************************************************************
* Compiler:         gcc 4.6.3
*
* Company:          Institute for Cognitive Systems
*                   Technical University of Munich
*
* Author:           Emmanuel Dean (dean@tum.de)
*                   Karinne Ramirez (karinne.ramirez@tum.de)
*
* Compatibility:    Ubuntu 12.04 64bit (ros hydro)
*
* Software Version: V0.1
*
* Created:          01.06.2015
*
* Comment:          turtle connection and visualization (Sensor and Signals)
*
********************************************************************/


/*********************************************************************
* STD INCLUDES
********************************************************************/
#include <iostream>
#include <fstream>
#include <pthread.h>


/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

/*********************************************************************
* EIGEN INCLUDES
********************************************************************/
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>

/*********************************************************************
 * SEVICES AND MESSAGES
 * ******************************************************************/
#include <turtle_vis_tiago/DesiredPose.h>
#include <turtle_vis_tiago/send_desired_pose.h>

using namespace Eigen;


int main(int argc, char** argv)
{

    ros::init(argc, argv, "turtle_set_position_node",ros::init_options::AnonymousName);

    ROS_INFO_STREAM("**Client turtle desired position");

    ros::NodeHandle n;
    ros::Rate r(60);

    ros::ServiceClient client=n.serviceClient<turtle_vis_tiago::send_desired_pose>("TurtlePose");

    turtle_vis_tiago::send_desired_pose msg;

    std::string myString;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion qtf;

    while(ros::ok())
    {

        std::vector<double> vals;

        ROS_INFO_STREAM("Give me the desired position of the turtle: x,y,theta");
        std::cin>>myString;

        std::cout<<"typed: "<<myString<<std::endl;
        char *writable = new char[myString.size() + 1];
        std::copy(myString.begin(), myString.end(), writable);
        writable[myString.size()] = '\0';
        char *p = strtok(writable, ",");
        while (p) {
            printf ("Value: %s\n", p);
            vals.push_back(atof(p));
            p = strtok(NULL, ",");
        }

        ROS_WARN_STREAM("Values: "<<vals[0]<<" "<<vals[1]<<" "<<vals[2]);
        msg.request.p.x=vals[0];
        msg.request.p.y=vals[1];
        msg.request.p.theta=vals[2];

        qtf.setRPY(0,0,msg.request.p.theta);
        transform.setOrigin(tf::Vector3(msg.request.p.x,msg.request.p.y,0));
        transform.setRotation(qtf);

        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/world","/turtle_desired"));


        if(client.call(msg))
        {
            ROS_INFO_STREAM("Moving turtle to desired position: "<<
                            msg.request.p.x<<", "<<msg.request.p.y<<
                            ", "<<msg.request.p.theta);
        }
        else
        {
            ROS_ERROR_STREAM("Failed to call the service 'TurtlePose'");
            return 1;
        }

        delete[] writable;
    }



    return 0;
}
