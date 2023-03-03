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

class Euler
{
public:
    double x;
    double y;
    double z;
    double pitch;
    double roll;
    double yaw;
};

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

    bool marker_frame_ok;     // make sure static tf (map->central_marker_frame) is ok
    bool camera_to_marker_ok; // make sure the topic "/aruco_single/pose" is ok
    void check_data();

    tf::TransformListener listener_;
    tf::TransformBroadcaster br_;
    geometry_msgs::TransformStamped trans;

    // Euler marker_pose; // the pose of the marker in camera_ref_frame
    tf::Pose marker_pose; // the pose of the marker in camera_ref_frame
};

Camera_location::Camera_location(ros::NodeHandle &nh)
{
    nh_ = nh;
    initialize();
}

void Camera_location::initialize()
{
    marker_frame_ok = false;
    camera_to_marker_ok = false;
    control_frequency = 50;

    trans.header.frame_id = "map";
    trans.child_frame_id = "camera_frame";

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency), &Camera_location::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency), false);
    timer_.start();
}

void Camera_location::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &p_)
{
    if (!camera_to_marker_ok)
        camera_to_marker_ok = true;

    marker_pose.setOrigin(tf::Vector3(p_->pose.position.x, p_->pose.position.y, p_->pose.position.z));
    marker_pose.setRotation(tf::Quaternion(p_->pose.orientation.x, p_->pose.orientation.y, p_->pose.orientation.z, p_->pose.orientation.w));
}

void Camera_location::check_data()
{
    if (!marker_frame_ok)
    {
        ROS_WARN_STREAM_THROTTLE(5, "[Camera location : ]"
                                        << "map frame to marker frame is not ok");
        bool ok = false;
        std::string *error_msg;
        ok = listener_.canTransform("map", "central_marker_frame", ros::Time(0), error_msg);
        if (ok)
            marker_frame_ok = true;
    }

    if (!camera_to_marker_ok)
    {
        ROS_WARN_STREAM_THROTTLE(5, "[Camera location : ]"
                                        << "camera frame to marker frame is not ok");
    }
}

void Camera_location::timerCallback(const ros::TimerEvent &e)
{
    if (marker_frame_ok && camera_to_marker_ok)
    {
        // look_up tf from map to marker
        tf::StampedTransform map_to_central_marker;
        try
        {
            listener_.lookupTransform("map", "central_marker_frame", ros::Time(0), map_to_central_marker);
        }
        catch (const tf::TransformException &ex)
        {
            try
            {
                listener_.lookupTransform("map", "central_marker_frame", ros::Time(0), map_to_central_marker);
            }
            catch (const tf::TransformException &ex)
            {
                ROS_WARN_STREAM_THROTTLE(10, ex.what());
            }
        }

        // calculate the translate tf from map to camera_frame store in the "trans_tf"
        tf::Pose trans_tf = map_to_central_marker * marker_pose.inverse();

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
    else
    {
        check_data();
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