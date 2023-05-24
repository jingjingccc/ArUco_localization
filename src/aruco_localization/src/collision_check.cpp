#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

class Collision_Check
{
public:
    Collision_Check(ros::NodeHandle nh);
    void initialize();

private:
    ros::NodeHandle nh_;

    double control_frequency;
    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent &e);

    ros::Subscriber robot1_pose_sub;
    ros::Subscriber robot2_pose_sub;
    ros::Subscriber robot3_pose_sub;
    ros::Publisher collision_pub;

    void robot1_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &p);
    void robot2_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &p);
    void robot3_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &p);

    geometry_msgs::Pose robot1_pose;
    geometry_msgs::Pose robot2_pose;
    geometry_msgs::Pose robot3_pose;

    bool robot1_update;
    bool robot2_update;
    bool robot3_update;

    double calculate_distance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);

    double tolerance_distance_;

    void check_data_ok();
    ros::Time robot1_cb_time;
    ros::Time robot2_cb_time;
    ros::Time robot3_cb_time;
};

Collision_Check::Collision_Check(ros::NodeHandle nh)
{
    nh_ = nh;
    initialize();
}

void Collision_Check::initialize()
{
    robot1_pose_sub = nh_.subscribe("robot1_pose", 10, &Collision_Check::robot1_pose_cb, this);
    robot2_pose_sub = nh_.subscribe("robot2_pose", 10, &Collision_Check::robot2_pose_cb, this);
    robot3_pose_sub = nh_.subscribe("robot3_pose", 10, &Collision_Check::robot3_pose_cb, this);
    collision_pub = nh_.advertise<std_msgs::Int32MultiArray>("collision_check", 10);

    control_frequency = 50;
    robot1_update = false;
    robot2_update = false;
    robot3_update = false;

    tolerance_distance_ = 0.15;
    nh_.getParam("tolerance_distance", tolerance_distance_);

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency), &Collision_Check::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency), false);
    timer_.start();
}

void Collision_Check::robot1_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &p)
{
    robot1_cb_time = ros::Time::now();
    robot1_pose = p->pose;
}

void Collision_Check::robot2_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &p)
{
    robot2_cb_time = ros::Time::now();
    robot2_pose = p->pose;
}

void Collision_Check::robot3_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &p)
{
    robot3_cb_time = ros::Time::now();
    robot3_pose = p->pose;
}

void Collision_Check::check_data_ok()
{
    if (ros::Time::now().toSec() - robot1_cb_time.toSec() > 0.2)
    {
        robot1_update = false;
        ROS_WARN_STREAM_THROTTLE(5, "[Collision Check] : "
                                        << "robot1 pose not received");
    }
    else
    {
        robot1_update = true;
    }

    if (ros::Time::now().toSec() - robot2_cb_time.toSec() > 0.2)
    {
        robot2_update = false;
        ROS_WARN_STREAM_THROTTLE(5, "[Collision Check] : "
                                        << "robot2 pose not received");
    }
    else
    {
        robot2_update = true;
    }

    if (ros::Time::now().toSec() - robot3_cb_time.toSec() > 0.2)
    {
        robot3_update = false;
        ROS_WARN_STREAM_THROTTLE(5, "[Collision Check] : "
                                        << "robot3 pose not received");
    }
    else
    {
        robot3_update = true;
    }
}

double Collision_Check::calculate_distance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    return sqrt(pow(p1.position.x - p2.position.x, 2) + pow(p1.position.y - p2.position.y, 2));
}

void Collision_Check::timerCallback(const ros::TimerEvent &e)
{
    check_data_ok();
    bool collision_12 = false, collision_23 = false, collision_13 = false;
    if (robot1_update && robot2_update)
    {
        if (calculate_distance(robot1_pose, robot2_pose) > tolerance_distance_)
        {
            collision_12 = true;
        }
    }
    else if (robot2_update && robot3_update)
    {
        if (calculate_distance(robot2_pose, robot3_pose) > tolerance_distance_)
        {
            collision_23 = true;
        }
    }
    else if (robot1_update && robot3_update)
    {
        if (calculate_distance(robot1_pose, robot3_pose) > tolerance_distance_)
        {
            collision_13 = true;
        }
    }
    std_msgs::Int32MultiArray collisionArr;
    collisionArr.data.push_back(collision_12);
    collisionArr.data.push_back(collision_23);
    collisionArr.data.push_back(collision_13);
    collision_pub.publish(collisionArr);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_check");
    ros::NodeHandle nh("");
    Collision_Check collision_check(nh);
    while (ros::ok())
    {
        ros::spinOnce();
    }
}