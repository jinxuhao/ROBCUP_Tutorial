/*********************************************************************
* Compiler:         gcc 4.6.3
*
* Company:          Institute for Cognitive Systems
*                   Technical University of Munich
*
* Author:           Emmanuel Dean (dean@tum.de)
*                   Karinne Ramirez (karinne.ramirez@tum.de)
*
* Midified by:      Rogelio Guadarrama (rogelio.guadarrama@tum.de)
*
* Compatibility:    Ubuntu 12.04 64bit (ros indigo)
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
 * CUSTOM CLASS
 * ******************************************************************/
#include <turtle_vis_tiago/myClass/TurtleClass.h>

int main( int argc, char** argv )
{

    ros::init(argc, argv, "turtle_control",ros::init_options::AnonymousName);

    ROS_INFO_STREAM("**Publishing turtle control..");

    ros::NodeHandle n;
    ros::Rate r(30);



    //Service
    turtleSpace::TurtleClass turtleF;
    ros::ServiceServer service=n.advertiseService("TurtlePose",
                                                  &turtleSpace::TurtleClass::getDPose,
                                                  &turtleF);
    //CALL SERVICE FROM TERMINAL//
    //    rosservice call /TurtlePose '{p: [0.5, 0.0, 3.0]}'
    //    rosservice call /TurtlePose "{p: {x: 1.5, y: 1.0, theta: 0.0}}"

    //Topic
    ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",50);
    ros::Subscriber sub=n.subscribe("/mobile_base_controller/odom", 100, &turtleSpace::TurtleClass::getTiagoPose, &turtleF);

    /////////////////////////////////////////////// New control law

    Matrix2d Kp;

    //Proportional Gain

    double p_g=0.0;

    //LOAD p_gain FROM THE ROS PARAMETER SERVER

    ros::param::get("control_gain/p_gain",p_g);
    ROS_INFO_STREAM("p_g= "<<p_g);

    Kp<<p_g,  0,
         0 ,p_g;

    ROS_INFO_STREAM("Kp= \n"<<Kp);

    //Target
    Vector3d tiagoPose_desired_local;
    Vector3d tiagoPose;
    Vector2d error;
    double theta, d;

    Vector2d tiagoVel;
    Vector2d tiagoVelDes;
    Vector2d tiagoPose2d;
    Matrix2d A;

    tiagoPose << 0, 0, 0;
    tiagoVel  << 0, 0;
    tiagoVelDes  << 0, 0;
    tiagoPose2d  << 0, 0;
    theta = 0;
    d = 0.3;

    A << 0, 0,
         0, 0;

    geometry_msgs::Twist msg;

    while(ros::ok())
    {
        //GetDesiredPose and robot states
        tiagoPose_desired_local = turtleF.getLocalDesiredPose();
        tiagoPose = turtleF.getLocalPose();

        tiagoPose2d(0) = tiagoPose(0) + d*cos(theta);
        tiagoPose2d(1) = tiagoPose(1) + d*sin(theta);
        theta = tiagoPose(2);

        tiagoVelDes(0) = tiagoPose_desired_local(0);
        tiagoVelDes(1) = tiagoPose_desired_local(1);

        A << cos(theta), -d*sin(theta),
             sin(theta),  d*cos(theta);

        //Control
        error = tiagoVelDes - tiagoPose2d;
        tiagoVelDes = Kp*error;
        tiagoVel = A.inverse()*tiagoVelDes;

        // Publish velocity command
        msg.linear.x = tiagoVel(0);
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = tiagoVel(1);

        pub.publish(msg);

        ros::spinOnce();

        r.sleep();
    }

    return 0;
}


