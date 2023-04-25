// for ros
#include <ros/ros.h>

// ros msg
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

class Robot_location
{
public:
    Robot_location(ros::NodeHandle &nh);
    void initialize();

private:
    ros::NodeHandle nh_;

    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent &e);
    double control_frequency;

    void poseCallback(const geometry_msgs::PoseStamped p_);
    ros::Subscriber pose_sub_ = nh_.subscribe("/aruco_robot_1/pose", 10, &Robot_location::poseCallback, this);
    ros::Time callback_time_;

    bool pose_received;
    geometry_msgs::PoseStamped robot_pose_in_camera_frame;
    geometry_msgs::PoseStamped robot_pose_in_map_frame;

    tf::TransformListener listener;

    ros::Publisher robot1_pose_pub;
};

Robot_location::Robot_location(ros::NodeHandle &nh)
{
    nh_ = nh;
    initialize();
}

void Robot_location::initialize()
{
    robot1_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("robot1_pose", 10);

    control_frequency = 50;
    pose_received = false;

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency), &Robot_location::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency), false);
    timer_.start();
}

void Robot_location::poseCallback(const geometry_msgs::PoseStamped p_)
{
    callback_time_ = ros::Time::now();
    robot_pose_in_camera_frame = p_;
}

void Robot_location::timerCallback(const ros::TimerEvent &e)
{
    if (ros::Time::now().toSec() - callback_time_.toSec() > 0.2)
    {
        pose_received = false;
        ROS_WARN_STREAM_THROTTLE(5, "[Robot location] : "
                                        << "robot1 marker in camera frame is not ok");
    }
    else
    {
        pose_received = true;
    }
    // make sure (map -> camera) tf is ok and robot pose in camera frame is ok
    if (pose_received)
    {
        // transform pose

        try
        {
            robot_pose_in_camera_frame.header.stamp = ros::Time(0);
            listener.transformPose("map", robot_pose_in_camera_frame, robot_pose_in_map_frame);
        }
        catch (const tf::TransformException &ex)
        {
            try
            {
                robot_pose_in_camera_frame.header.stamp = ros::Time(0);
                listener.transformPose("map", robot_pose_in_camera_frame, robot_pose_in_map_frame);
            }
            catch (const tf::TransformException &ex)
            {
                ROS_WARN_STREAM_THROTTLE(10, ex.what());
            }
        }

        // publish robot pose in map frame
        robot1_pose_pub.publish(robot_pose_in_map_frame);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_loc");
    ros::NodeHandle nh("");
    Robot_location robot_location(nh);
    while (ros::ok())
    {
        ros::spinOnce();
    }
}