/*********************************************************************
* Compiler:         gcc 4.6.3
*
* Company:          Institute for Cognitive Systems
*                   Technical University of Munich
*
* Author:           Rogelio Guadarrama
*
* Compatibility:    Ubuntu 16.04 64bit (ros kinetic)
*
* Software Version: V0.1
*
* Created:          25.10.2019
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



const visualization_msgs::Marker createMarkerMesh(std::string frame_id, int id, int shape,
                                                  double x, double y, double z, /*position*/
                                                  double q_w, double q_x, double q_y, double q_z, /*orientation in quatern*/
                                                  double s_x, double s_y, double s_z, std::string meshFile/*scale*/)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "tracker_markers";
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = q_x;
    marker.pose.orientation.y = q_y;
    marker.pose.orientation.z = q_z;
    marker.pose.orientation.w = q_w;
    marker.scale.x = s_x;
    marker.scale.y = s_y;
    marker.scale.z = s_z;
    marker.mesh_resource = meshFile;

    marker.color.r = 0;
    marker.color.g = 0.7;
    marker.color.b = 0.5;
    marker.color.a = 1;
    marker.lifetime = ros::Duration();

    return marker;
}



int main( int argc, char** argv )
{

    ros::init(argc, argv, "turtle_visualization",ros::init_options::AnonymousName);

    ROS_INFO_STREAM("**Publishing turtle position for rviz..");

    ros::NodeHandle n;
    ros::Rate r(60);

    static tf::TransformBroadcaster br;



    tf::Transform transform;

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    visualization_msgs::Marker turtle3D;



    turtle3D = createMarkerMesh("/turtle", 12345, visualization_msgs::Marker::MESH_RESOURCE,
                                /*pos xyz:*/ 0.0, 0.024, -0.021,
                                /*orientation quatern wxyz*/ 0 ,0 ,0, 1,
                                /*scale s_x s_y s_z*/ 0.025, 0.025, 0.025,
                                "package://turtle_vis_hsrb/meshes/turtle2.dae");

   turtleSpace::TurtleClass turtleFunc;
   ros::Subscriber sub=n.subscribe("turtle_control",100,
                                   &turtleSpace::TurtleClass::getPose,
                                   &turtleFunc);


    tf::Quaternion qtf;


    Vector3d turtlePose;

    turtlePose<<1,0,0;


    //Target
    Vector3d turtlePose_local;
    turtleFunc.turtlePose_g=turtlePose;
    turtlePose_local=turtlePose;



    while(ros::ok())
    {

        //GetPose
        turtlePose_local=turtleFunc.getLocalPose();

        //Control
        qtf.setRPY(0,0,turtlePose_local(2));
        transform.setOrigin(tf::Vector3(turtlePose_local(0),turtlePose_local(1),0));

        transform.setRotation(qtf );
        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/world","/turtle"));
        marker_pub.publish(turtle3D);

        ros::spinOnce();

        r.sleep();
    }
    //////////////////////////////////Skin Cells Visualization//////////////////////////////////
    return 0;
}

