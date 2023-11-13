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

    ros::Time callback_time_;

    bool fixed_marker_received; // make sure fixed_marker pose param is ok
    bool camera_to_marker_ok;   // make sure the topic "/aruco_fixed_marker/pose" is ok
    void check_data_ok();

    tf::TransformListener listener_;
    tf::TransformBroadcaster br_;                                 // map->camera_frame broadcaster
    tf::StampedTransform map_to_camera_trans, past_map_to_camera; // map->camera_frame broadcast msg
    tf::Quaternion last_m_2_;                                     // store last time fixed marker (relative to camera_frame) orientation (for filter)

    bool first_in; // first time go into timercallback

    // fixed marker pose param
    void Load_Param();
    double fixed_marker_position_x_;
    double fixed_marker_position_y_;
    double fixed_marker_position_z_;
    double fixed_marker_orientation_x_;
    double fixed_marker_orientation_y_;
    double fixed_marker_orientation_z_;
    double fixed_marker_orientation_w_;
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
    first_in = true;
    control_frequency = 50;

    // map->camera_frame broadcast msg
    map_to_camera_trans.frame_id_ = "map";
    map_to_camera_trans.child_frame_id_ = "camera_frame";

    // load param
    Load_Param();

    // timer init
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency), &Camera_location::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency), false);
    timer_.start();
}

void Camera_location::Load_Param()
{
    bool ifparamGet;
    ifparamGet = ros::param::get("fixed_marker_position_x", fixed_marker_position_x_);
    ifparamGet = ros::param::get("fixed_marker_position_y", fixed_marker_position_y_);
    ifparamGet = ros::param::get("fixed_marker_position_z", fixed_marker_position_z_);
    ifparamGet = ros::param::get("fixed_marker_orientation_x", fixed_marker_orientation_x_);
    ifparamGet = ros::param::get("fixed_marker_orientation_y", fixed_marker_orientation_y_);
    ifparamGet = ros::param::get("fixed_marker_orientation_z", fixed_marker_orientation_z_);
    ifparamGet = ros::param::get("fixed_marker_orientation_w", fixed_marker_orientation_w_);

    if (ifparamGet)
        ROS_ERROR("[Camera_location]: get fixed marker param failure !!!");
}

void Camera_location::check_data_ok()
{
    /*check param of fixed marker exist*/
    if (!fixed_marker_received)
    {
        bool ifhasParam = false;
        ifhasParam = ros::param::has("fixed_marker_position_x");
        ifhasParam = ros::param::has("fixed_marker_position_y");
        ifhasParam = ros::param::has("fixed_marker_position_z");
        ifhasParam = ros::param::has("fixed_marker_orientation_x");
        ifhasParam = ros::param::has("fixed_marker_orientation_y");
        ifhasParam = ros::param::has("fixed_marker_orientation_z");
        ifhasParam = ros::param::has("fixed_marker_orientation_w");
        if (ifhasParam)
        {
            fixed_marker_received = true;
        }
        else
        {
            ROS_WARN_STREAM_THROTTLE(5, "[Camera location] : "
                                            << "fixed marker param is not ok");
            Load_Param();
        }
    }

    /*check tf from camera to marker exist*/
    if (!camera_to_marker_ok)
    {
        ROS_WARN_STREAM_THROTTLE(5, "[Camera location] : "
                                        << "camera to fixed marker is not ok");
        bool ok = false;
        std::string err;
        ok = listener_.canTransform("fixed_marker_frame", "camera_frame", ros::Time(0), &err);
        std::cout << "err: " << err << std::endl;
        if (ok)
        {
            camera_to_marker_ok = true;
        }
    }
}

void Camera_location::timerCallback(const ros::TimerEvent &e)
{
    check_data_ok();
    if (fixed_marker_received && camera_to_marker_ok)
    {
        // set tf from map to fixed_marker_frame
        tf::StampedTransform m_1_, m_2_;
        m_1_.setOrigin(tf::Vector3(fixed_marker_position_x_, fixed_marker_position_y_, fixed_marker_position_z_));
        m_1_.setRotation(tf::Quaternion(fixed_marker_orientation_x_, fixed_marker_orientation_y_, fixed_marker_orientation_z_, fixed_marker_orientation_w_));
        ROS_INFO("[Camera Location: fixed markerpose]: %f %f %f %f %f %f %f\n", fixed_marker_position_x_, fixed_marker_position_y_, fixed_marker_position_z_, fixed_marker_orientation_x_, fixed_marker_orientation_y_, fixed_marker_orientation_z_, fixed_marker_orientation_w_);

        // look up tf from fixed_marker_frame to camera_frame
        try
        {
            listener_.lookupTransform("fixed_marker_frame", "camera_frame", ros::Time(0), m_2_);
        }
        catch (const tf::TransformException &ex)
        {
            try
            {
                listener_.lookupTransform("fixed_marker_frame", "camera_frame", ros::Time(0), m_2_);
            }
            catch (const tf::TransformException &ex)
            {
                ROS_WARN_STREAM_THROTTLE(10, ex.what());
            }
        }

        if (first_in)
            last_m_2_ = m_2_.getRotation();

        // make sure cur quaternion and last quaternion in the same mode. If not, convert.
        if (last_m_2_.dot(m_2_.getRotation()) < 0)
            m_2_.setRotation(tf::Quaternion(m_2_.getRotation().x() * -1, m_2_.getRotation().y() * -1, m_2_.getRotation().z() * -1, m_2_.getRotation().w() * -1));

        // filter for aruco_ros msg: fixed marker orientation (relative to camera)
        double filter_portion = 0.99; // since it shouldn't change, set a very high portion
        tfScalar xx_ = filter_portion * last_m_2_.x() + (1 - filter_portion) * m_2_.getRotation().x();
        tfScalar yy_ = filter_portion * last_m_2_.y() + (1 - filter_portion) * m_2_.getRotation().y();
        tfScalar zz_ = filter_portion * last_m_2_.z() + (1 - filter_portion) * m_2_.getRotation().z();
        tfScalar ww_ = filter_portion * last_m_2_.w() + (1 - filter_portion) * m_2_.getRotation().w();
        m_2_.setRotation(tf::Quaternion(xx_, yy_, zz_, ww_));

        // translate two tf relationship
        // return tt is tf from map to camera_frame
        tf::Transform tt = m_1_ * m_2_;
        if (first_in)
        {
            past_map_to_camera.setOrigin(tt.getOrigin());
            past_map_to_camera.setRotation(tt.getRotation());
            // std::cout << "first time" << std::endl;
            first_in = false;
        }

        // filter for tf from map to camera_frame (position)
        tf::Vector3 pp;
        pp.setX(filter_portion * past_map_to_camera.getOrigin().getX() + (1 - filter_portion) * tt.getOrigin().getX());
        pp.setY(filter_portion * past_map_to_camera.getOrigin().getY() + (1 - filter_portion) * tt.getOrigin().getY());
        pp.setZ(filter_portion * past_map_to_camera.getOrigin().getZ() + (1 - filter_portion) * tt.getOrigin().getZ());
        tt.setOrigin(pp);

        // make sure two quaternion in the same mode. If not, convert.
        if (past_map_to_camera.getRotation().dot(tt.getRotation()) < 0)
        {
            tt.setRotation(tf::Quaternion(tt.getRotation().x() * -1, tt.getRotation().y() * -1, tt.getRotation().z() * -1, tt.getRotation().w() * -1));
        }
        // filter for tf from map to camera_frame (orientation)
        tfScalar x_ = (1 - filter_portion) * tt.getRotation().getX() + filter_portion * past_map_to_camera.getRotation().getX();
        tfScalar y_ = (1 - filter_portion) * tt.getRotation().getY() + filter_portion * past_map_to_camera.getRotation().getY();
        tfScalar z_ = (1 - filter_portion) * tt.getRotation().getZ() + filter_portion * past_map_to_camera.getRotation().getZ();
        tfScalar w_ = (1 - filter_portion) * tt.getRotation().getW() + filter_portion * past_map_to_camera.getRotation().getW();
        tt.setRotation(tf::Quaternion(x_, y_, z_, w_));

        map_to_camera_trans.stamp_ = ros::Time::now();
        map_to_camera_trans.setData(tt);
        br_.sendTransform(map_to_camera_trans);

        // store last data
        past_map_to_camera.setOrigin(tt.getOrigin());
        past_map_to_camera.setRotation(tt.getRotation());
        last_m_2_ = m_2_.getRotation();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_loc_plus");
    ros::NodeHandle nh("");
    Camera_location Camera_location(nh);
    while (ros::ok())
    {
        ros::spinOnce();
    }
}