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
#include <turtle_vis_hsrb/myClass/TurtleClass.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "turtle_control",ros::init_options::AnonymousName);
  ROS_INFO_STREAM("**Publishing turtle control..");

  ros::NodeHandle n;
  ros::Rate r(25);

  //Service
  turtleSpace::TurtleClass turtleF;
  ros::ServiceServer service=n.advertiseService("TurtlePose",
                                                &turtleSpace::TurtleClass::getDPose,
                                                &turtleF);
  //CALL SERVICE FROM TERMINAL//
  //    rosservice call /TurtlePose '{p: [0.5, 0.0, 3.0]}'
  //    rosservice call /TurtlePose "{p: {x: 1.5, y: 1.0, theta: 0.0}}"

  //Topic
  ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/base_velocity",50);
  ros::Subscriber sub=n.subscribe("/hsrb/odom", 100, &turtleSpace::TurtleClass::getHSRBPose, &turtleF);

  /////////////////////////////////////////////// New control law

  Matrix3d Kp;
  ros::Time ti;
  ti=ros::Time::now();

  //Proportional Gain

  double p_g=0.0;

  //LOAD p_gain FROM THE ROS PARAMETER SERVER

  ros::param::get("control_gain/p_gain",p_g);
  ROS_INFO_STREAM("p_g= "<<p_g);

  Kp<<p_g,0  ,0,
      0 ,p_g,0,
      0 ,0  ,p_g;

  ROS_INFO_STREAM("Kp= \n"<<Kp);

  //Target
  Vector3d hsrbPose_desired_local;
  Vector3d hsrbPose;
  Vector3d error;

  Vector3d hsrbVel;
  Vector3d hsrbVelDes;

  hsrbPose   << 0, 0, 0;
  hsrbVel    << 0, 0, 0;
  hsrbVelDes << 0, 0, 0;

  geometry_msgs::Twist msg;

  while(ros::ok())
  {
    //GetDesiredPose and robot states
    hsrbPose_desired_local = turtleF.getLocalDesiredPose();
    hsrbPose = turtleF.getLocalPose();

    hsrbVelDes = hsrbPose_desired_local;

    //Control
    error = hsrbVelDes - hsrbPose;
    hsrbVelDes = Kp*error;

    // Publish velocity command
    msg.linear.x  = hsrbVelDes(0);
    msg.linear.y  = hsrbVelDes(1);
    msg.linear.z  = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = hsrbVelDes(2);

    pub.publish(msg);

    ros::spinOnce();

    r.sleep();
  }

  return 0;
}


