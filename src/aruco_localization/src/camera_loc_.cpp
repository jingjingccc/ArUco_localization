// for ros
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ros msg
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

// eigen
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Geometry>

class Camera_location
{
public:
    Camera_location(ros::NodeHandle &nh);
    void initialize();

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer buffer_;

    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent &e);
    double control_frequency;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &p_);
    ros::Subscriber pose_sub_ = nh_.subscribe("/aruco_fixed_marker/pose", 10, &Camera_location::poseCallback, this);
    ros::Time callback_time_;

    bool fixed_marker_received; // make sure static tf (map->fixed_marker_frame) is ok
    bool camera_to_marker_ok;   // make sure the topic "/aruco_fixed_marker/pose" is ok // the received frequency must above 5 hz
    void check_data_ok();
    void set_fixed_marker_msg();

    tf::TransformListener listener_;
    tf::TransformBroadcaster br_;
    geometry_msgs::TransformStamped map_to_camera_trans;

    // tf::Pose marker_pose; // pose of the fixed marker in camera_frame
    geometry_msgs::Pose marker_pose;
    // tf::Pose fixed_marker_pose; // pose of the fixed marker in map frame
    geometry_msgs::Pose fixed_marker_pose;
};

Camera_location::Camera_location(ros::NodeHandle &nh)
{
    nh_ = nh;
    initialize();
}

void Camera_location::initialize()
{
    fixed_marker_received = false;
    camera_to_marker_ok = false;
    control_frequency = 50;

    map_to_camera_trans.header.frame_id = "map";
    map_to_camera_trans.child_frame_id = "camera_frame";

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency), &Camera_location::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency), false);
    timer_.start();
}

void Camera_location::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &p_)
{
    callback_time_ = ros::Time::now();
    // marker_pose.setOrigin(tf::Vector3(p_->pose.position.x, p_->pose.position.y, p_->pose.position.z));
    // marker_pose.setRotation(tf::Quaternion(p_->pose.orientation.x, p_->pose.orientation.y, p_->pose.orientation.z, p_->pose.orientation.w));
    marker_pose = p_->pose;
}

void Camera_location::check_data_ok()
{
    if (!fixed_marker_received) // static tf
    {
        ROS_WARN_STREAM_THROTTLE(5, "[Camera location] : "
                                        << "map to central marker tf is not ok");
        bool ok = false;
        ok = listener_.canTransform("map", "fixed_marker_frame", ros::Time(0));
        if (ok)
        {
            fixed_marker_received = true;
            set_fixed_marker_msg();
        }
    }

    if (ros::Time::now().toSec() - callback_time_.toSec() > 0.2)
    {
        camera_to_marker_ok = false;
    }
    else
    {
        camera_to_marker_ok = true;
    }

    if (!camera_to_marker_ok)
    {
        ROS_WARN_STREAM_THROTTLE(5, "[Camera location] : "
                                        << "camera to fixed marker is not ok");
    }
}

void Camera_location::set_fixed_marker_msg()
{
    tf::StampedTransform m_p_;
    try
    {
        listener_.lookupTransform("map", "fixed_marker_frame", ros::Time(0), m_p_);
    }
    catch (const tf::TransformException &ex)
    {
        try
        {
            listener_.lookupTransform("map", "fixed_marker_frame", ros::Time(0), m_p_);
        }
        catch (const tf::TransformException &ex)
        {
            ROS_WARN_STREAM_THROTTLE(10, ex.what());
        }
    }
    fixed_marker_pose.position.x = m_p_.getOrigin().getX();
    fixed_marker_pose.position.y = m_p_.getOrigin().getY();
    fixed_marker_pose.position.z = m_p_.getOrigin().getZ();
    fixed_marker_pose.orientation.x = m_p_.getRotation().getX();
    fixed_marker_pose.orientation.y = m_p_.getRotation().getY();
    fixed_marker_pose.orientation.z = m_p_.getRotation().getZ();
    fixed_marker_pose.orientation.w = m_p_.getRotation().getW();
    // ROS_INFO("fixed marker pose : (%f %f %f)(%f %f %f %f)\n", fixed_marker_pose.getOrigin().getX(), fixed_marker_pose.getOrigin().getY(), fixed_marker_pose.getOrigin().getZ(), fixed_marker_pose.getRotation().getX(), fixed_marker_pose.getRotation().getY(), fixed_marker_pose.getRotation().getZ(), fixed_marker_pose.getRotation().getW());
}

void Camera_location::timerCallback(const ros::TimerEvent &e)
{
    check_data_ok();
    if (fixed_marker_received && camera_to_marker_ok)
    {
        Eigen::Isometry3d marker_pose_;
        tf::poseMsgToEigen(marker_pose, marker_pose_);
        Eigen::Isometry3d fixed_marker_pose_;
        tf::poseMsgToEigen(fixed_marker_pose, fixed_marker_pose_);

        marker_pose_ = marker_pose_.inverse();
        Eigen::Isometry3d trans = fixed_marker_pose_ * marker_pose_;

        geometry_msgs::Pose trans_;
        tf::poseEigenToMsg(trans, trans_);

        map_to_camera_trans.header.stamp = ros::Time::now();
        map_to_camera_trans.transform.translation.x = trans_.position.x;
        map_to_camera_trans.transform.translation.y = trans_.position.y;
        map_to_camera_trans.transform.translation.z = trans_.position.z;
        map_to_camera_trans.transform.rotation.x = trans_.orientation.x;
        map_to_camera_trans.transform.rotation.y = trans_.orientation.y;
        map_to_camera_trans.transform.rotation.z = trans_.orientation.z;
        map_to_camera_trans.transform.rotation.w = trans_.orientation.w;

        br_.sendTransform(map_to_camera_trans);
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