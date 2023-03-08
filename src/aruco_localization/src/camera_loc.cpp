// for ros
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
// ros msg
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

class Camera_location
{
public:
    Camera_location(ros::NodeHandle &nh);
    void initialize();

private:
    ros::NodeHandle nh_;

    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent &e);
    double control_frequency;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &p_);
    ros::Subscriber pose_sub_ = nh_.subscribe("/aruco_single/pose", 10, &Camera_location::poseCallback, this);
    ros::Time callback_time_;

    bool central_marker_received; // make sure static tf (map->central_marker_frame) is ok
    bool camera_to_marker_ok;     // make sure the topic "/aruco_single/pose" is ok
    void check_data_ok();
    void set_central_marker_msg();

    tf::TransformListener listener_;
    tf::TransformBroadcaster br_;
    geometry_msgs::TransformStamped trans;

    tf::Pose marker_pose;         // pose of the cenreal marker in camera_frame
    tf::Pose central_marker_pose; // pose of the central marker in map_frame
};

Camera_location::Camera_location(ros::NodeHandle &nh)
{
    nh_ = nh;
    initialize();
}

void Camera_location::initialize()
{
    central_marker_received = false;
    camera_to_marker_ok = false;
    central_marker_pose.setOrigin(tf::Vector3(-100, -100, -100));
    central_marker_pose.setRotation(tf::Quaternion(1, 1, 1, 1));
    control_frequency = 50;

    trans.header.frame_id = "map";
    trans.child_frame_id = "camera_frame";

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency), &Camera_location::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency), false);
    timer_.start();
}

void Camera_location::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &p_)
{
    callback_time_ = ros::Time::now();
    marker_pose.setOrigin(tf::Vector3(p_->pose.position.x, p_->pose.position.y, p_->pose.position.z));
    marker_pose.setRotation(tf::Quaternion(p_->pose.orientation.x, p_->pose.orientation.y, p_->pose.orientation.z, p_->pose.orientation.w));
}

void Camera_location::check_data_ok()
{
    if (!central_marker_received) // static tf
    {
        ROS_WARN_STREAM_THROTTLE(5, "[Camera location] : "
                                        << "map to central marker tf is not ok");
        bool ok = false;
        ok = listener_.canTransform("map", "central_marker_frame", ros::Time(0));
        if (ok)
        {
            central_marker_received = true;
            set_central_marker_msg();
        }
    }
    if (ros::Time::now().toSec() - callback_time_.toSec() > 0.5)
    {
        camera_to_marker_ok = false;
    }
    else
    {
        camera_to_marker_ok = true;
    }

    // and think how to display on Rivz

    if (!camera_to_marker_ok)
    {
        ROS_WARN_STREAM_THROTTLE(5, "[Camera location] : "
                                        << "camera frame to marker frame is not ok");
    }
}

void Camera_location::set_central_marker_msg()
{
    tf::StampedTransform m_p_;
    try
    {
        listener_.lookupTransform("map", "central_marker_frame", ros::Time(0), m_p_);
    }
    catch (const tf::TransformException &ex)
    {
        try
        {
            listener_.lookupTransform("map", "central_marker_frame", ros::Time(0), m_p_);
        }
        catch (const tf::TransformException &ex)
        {
            ROS_WARN_STREAM_THROTTLE(10, ex.what());
        }
    }
    central_marker_pose.setOrigin(tf::Vector3(m_p_.getOrigin().getX(), m_p_.getOrigin().getY(), m_p_.getOrigin().getZ()));
    central_marker_pose.setRotation(tf::Quaternion(m_p_.getRotation().getX(), m_p_.getRotation().getY(), m_p_.getRotation().getZ(), m_p_.getRotation().getW()));
}

void Camera_location::timerCallback(const ros::TimerEvent &e)
{
    check_data_ok();
    if (central_marker_received && camera_to_marker_ok)
    {
        // not using tf look up relation between map and central marker, use para
        // calculate the translate tf from map to camera_frame store in the "trans_tf"
        tf::Pose trans_tf = central_marker_pose * marker_pose.inverse();

        // send out the transform
        trans.header.stamp = ros::Time::now();
        trans.transform.translation.x = trans_tf.getOrigin().getX();
        trans.transform.translation.y = trans_tf.getOrigin().getY();
        trans.transform.translation.z = trans_tf.getOrigin().getZ();
        trans.transform.rotation.x = trans_tf.getRotation().getX();
        trans.transform.rotation.y = trans_tf.getRotation().getY();
        trans.transform.rotation.z = trans_tf.getRotation().getZ();
        trans.transform.rotation.w = trans_tf.getRotation().getW();
        br_.sendTransform(trans);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_loc");
    ros::NodeHandle nh("");
    Camera_location Camera_location(nh);
    while (ros::ok())
    {
        ros::spinOnce();
    }
}