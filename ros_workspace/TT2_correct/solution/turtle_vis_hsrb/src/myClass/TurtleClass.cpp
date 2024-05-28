#include<turtle_vis_hsrb/myClass/TurtleClass.h>

namespace turtleSpace {

TurtleClass::TurtleClass()
{
    ROS_INFO("Dumb constructor");
    turtlePose_g<<0,0,0;
    turtlePose_desired_g<<0,0,0;
    count_mutex = PTHREAD_MUTEX_INITIALIZER;
}
TurtleClass::~TurtleClass()
{
    ROS_INFO("Dumb distructor");
}

void TurtleClass::getPose(const turtle_vis_hsrb::DesiredPose::ConstPtr &msg)
{
    pthread_mutex_lock( &this->count_mutex );
    this->turtlePose_g(0)=msg->x;
    this->turtlePose_g(1)=msg->y;
    this->turtlePose_g(2)=msg->theta;
    pthread_mutex_unlock( &this->count_mutex );

    ROS_INFO_STREAM("Vis Turtle Pose: "<<this->turtlePose_g.transpose());
}

bool TurtleClass::getDPose(turtle_vis_hsrb::send_desired_pose::Request &req, turtle_vis_hsrb::send_desired_pose::Response &res)
{
    pthread_mutex_lock( &this->count_mutex );
    this->turtlePose_desired_g(0)=req.p.x;
    this->turtlePose_desired_g(1)=req.p.y;
    this->turtlePose_desired_g(2)=req.p.theta;
    pthread_mutex_unlock( &this->count_mutex );

    ROS_INFO_STREAM("Desired Pose: "<<this->turtlePose_desired_g.transpose());

    res.reply=1;

    return true;
}

Vector3d TurtleClass::getLocalPose()
{
    Vector3d local;
    pthread_mutex_lock( &this->count_mutex );
    local=this->turtlePose_g;
    pthread_mutex_unlock( &this->count_mutex );

    return local;
}

Vector3d TurtleClass::getLocalDesiredPose()
{
    Vector3d local;
    pthread_mutex_lock( &this->count_mutex );
    local=this->turtlePose_desired_g;
    pthread_mutex_unlock( &this->count_mutex );

    return local;
}

/////////////////////////////////////////////////////////////// added this function
void TurtleClass::getHSRBPose(const nav_msgs::Odometry::ConstPtr &msg)
{
    pthread_mutex_lock( &this->count_mutex );

    this->turtlePose_g(0)=msg->pose.pose.position.x;
    this->turtlePose_g(1)=msg->pose.pose.position.y;

    Quaterniond q;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;

    Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    this->turtlePose_g(2) = euler(2);

    pthread_mutex_unlock( &this->count_mutex );

    ROS_INFO_STREAM("Tiago Pose: "<<this->turtlePose_g.transpose());
}


}
