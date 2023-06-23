// for ros
#include <ros/ros.h>
#include <ros/package.h>

// ros msg
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>

// cpp
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
using namespace std;

class Mission_publish
{
public:
    Mission_publish(ros::NodeHandle &nh);
    void initialize();

private:
    ros::NodeHandle nh_;

    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent &e);
    double control_frequency;

    void flagCallback(const std_msgs::Bool::ConstPtr &flag);
    ros::Subscriber flag_sub = nh_.subscribe("mission_flag", 10, &Mission_publish::flagCallback, this);
    bool flag;

    void read_CSV();
    void printmissionList();

    vector<vector<int>> missionList;
    vector<int> row;
    string line, word;
    string packagepath;
    int howmanyrobots;

    ros::Publisher mission1_pub = nh_.advertise<geometry_msgs::PoseStamped>("robot1_goal", 10);
    ros::Publisher mission2_pub = nh_.advertise<geometry_msgs::PoseStamped>("robot2_goal", 10);
    ros::Publisher mission3_pub = nh_.advertise<geometry_msgs::PoseStamped>("robot3_goal", 10);

    void reached1Callback(const std_msgs::Bool::ConstPtr &r);
    void reached2Callback(const std_msgs::Bool::ConstPtr &r);
    void reached3Callback(const std_msgs::Bool::ConstPtr &r);
    ros::Subscriber reached_sub_1 = nh_.subscribe("robot1_reached", 10, &Mission_publish::reached1Callback, this);
    ros::Subscriber reached_sub_2 = nh_.subscribe("robot2_reached", 10, &Mission_publish::reached2Callback, this);
    ros::Subscriber reached_sub_3 = nh_.subscribe("robot3_reached", 10, &Mission_publish::reached3Callback, this);

    bool robot1_push_n;
    bool robot2_push_n;
    bool robot3_push_n;

    geometry_msgs::PoseStamped get_mission_pose(int missionNum);
    geometry_msgs::PoseStamped m1_pose, m2_pose, m3_pose, m4_pose, m5_pose,
        m6_pose, m7_pose, m8_pose, m9_pose, m10_pose, m11_pose, m12_pose;

    void m1Callback(const geometry_msgs::PoseStamped p);
    ros::Subscriber mission_pose_sub_1 = nh_.subscribe("/mission1_pose", 10, &Mission_publish::m1Callback, this);
    void m2Callback(const geometry_msgs::PoseStamped p);
    ros::Subscriber mission_pose_sub_2 = nh_.subscribe("/mission2_pose", 10, &Mission_publish::m2Callback, this);
    void m3Callback(const geometry_msgs::PoseStamped p);
    ros::Subscriber mission_pose_sub_3 = nh_.subscribe("/mission3_pose", 10, &Mission_publish::m3Callback, this);
    void m4Callback(const geometry_msgs::PoseStamped p);
    ros::Subscriber mission_pose_sub_4 = nh_.subscribe("/mission4_pose", 10, &Mission_publish::m4Callback, this);
    void m5Callback(const geometry_msgs::PoseStamped p);
    ros::Subscriber mission_pose_sub_5 = nh_.subscribe("/mission5_pose", 10, &Mission_publish::m5Callback, this);
    void m6Callback(const geometry_msgs::PoseStamped p);
    ros::Subscriber mission_pose_sub_6 = nh_.subscribe("/mission6_pose", 10, &Mission_publish::m6Callback, this);
    void m7Callback(const geometry_msgs::PoseStamped p);
    ros::Subscriber mission_pose_sub_7 = nh_.subscribe("/mission7_pose", 10, &Mission_publish::m7Callback, this);
    void m8Callback(const geometry_msgs::PoseStamped p);
    ros::Subscriber mission_pose_sub_8 = nh_.subscribe("/mission8_pose", 10, &Mission_publish::m8Callback, this);
    void m9Callback(const geometry_msgs::PoseStamped p);
    ros::Subscriber mission_pose_sub_9 = nh_.subscribe("/mission9_pose", 10, &Mission_publish::m9Callback, this);
    void m10Callback(const geometry_msgs::PoseStamped p);
    ros::Subscriber mission_pose_sub_10 = nh_.subscribe("/mission10_pose", 10, &Mission_publish::m10Callback, this);
    void m11Callback(const geometry_msgs::PoseStamped p);
    ros::Subscriber mission_pose_sub_11 = nh_.subscribe("/mission11_pose", 10, &Mission_publish::m11Callback, this);
    void m12Callback(const geometry_msgs::PoseStamped p);
    ros::Subscriber mission_pose_sub_12 = nh_.subscribe("/mission12_pose", 10, &Mission_publish::m12Callback, this);
};

Mission_publish::Mission_publish(ros::NodeHandle &nh)
{
    nh_ = nh;
    initialize();
}

void Mission_publish::read_CSV()
{
    packagepath = ros::package::getPath("aruco_localization");
    fstream file(packagepath + "/csv/" + "/mission1.csv");
    if (file.is_open())
    {
        while (getline(file, line))
        {
            row.clear();
            stringstream str(line);
            while (getline(str, word, ','))
            {
                row.push_back(stoi(word));
            }
            missionList.push_back(row);
        }
        file.close();
        howmanyrobots = missionList.size();
        ROS_INFO("How many robot? %d robots\n", howmanyrobots);
    }
    else
    {
        ROS_INFO("Could not open the file\n");
    }
    // printmissionList();
}

void Mission_publish::printmissionList()
{
    for (int i = 0; i < missionList.size(); i++)
    {
        for (int j = 0; j < missionList[i].size(); j++)
        {
            ROS_INFO("%d", missionList[i][j]);
        }
    }
}

void Mission_publish::flagCallback(const std_msgs::Bool::ConstPtr &f)
{
    if (f)
    {
        flag = true;
        if (howmanyrobots == 2)
        {
            robot1_push_n = true;
            robot2_push_n = true;
        }
        else if (howmanyrobots == 3)
        {
            robot1_push_n = true;
            robot2_push_n = true;
            robot3_push_n = true;
        }
    }
}

void Mission_publish::initialize()
{
    control_frequency = 50;
    flag = false;
    robot1_push_n = false;
    robot2_push_n = false;
    robot3_push_n = false;
    read_CSV();

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency), &Mission_publish::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency), false);
    timer_.start();
}

void Mission_publish::reached1Callback(const std_msgs::Bool::ConstPtr &r)
{
    if (r)
        robot1_push_n = true;
}
void Mission_publish::reached2Callback(const std_msgs::Bool::ConstPtr &r)
{
    if (r)
        robot2_push_n = true;
}
void Mission_publish::reached3Callback(const std_msgs::Bool::ConstPtr &r)
{
    if (r)
        robot3_push_n = true;
}

void Mission_publish::m1Callback(const geometry_msgs::PoseStamped p)
{
    m1_pose = p;
}
void Mission_publish::m2Callback(const geometry_msgs::PoseStamped p)
{
    m2_pose = p;
}
void Mission_publish::m3Callback(const geometry_msgs::PoseStamped p)
{
    m3_pose = p;
}
void Mission_publish::m4Callback(const geometry_msgs::PoseStamped p)
{
    m4_pose = p;
}
void Mission_publish::m5Callback(const geometry_msgs::PoseStamped p)
{
    m5_pose = p;
}
void Mission_publish::m6Callback(const geometry_msgs::PoseStamped p)
{
    m6_pose = p;
}
void Mission_publish::m7Callback(const geometry_msgs::PoseStamped p)
{
    m7_pose = p;
}
void Mission_publish::m8Callback(const geometry_msgs::PoseStamped p)
{
    m8_pose = p;
}
void Mission_publish::m9Callback(const geometry_msgs::PoseStamped p)
{
    m9_pose = p;
}
void Mission_publish::m10Callback(const geometry_msgs::PoseStamped p)
{
    m10_pose = p;
}
void Mission_publish::m11Callback(const geometry_msgs::PoseStamped p)
{
    m11_pose = p;
}
void Mission_publish::m12Callback(const geometry_msgs::PoseStamped p)
{
    m12_pose = p;
}

geometry_msgs::PoseStamped Mission_publish::get_mission_pose(int missionNum)
{
    geometry_msgs::PoseStamped p;
    switch (missionNum)
    {
    case 1:
        p = m1_pose;
        break;
    case 2:
        p = m2_pose;
        break;
    case 3:
        p = m3_pose;
        break;
    case 4:
        p = m4_pose;
        break;
    case 5:
        p = m5_pose;
        break;
    case 6:
        p = m6_pose;
        break;
    case 7:
        p = m7_pose;
        break;
    case 8:
        p = m8_pose;
        break;
    case 9:
        p = m9_pose;
        break;
    case 10:
        p = m10_pose;
        break;
    case 11:
        p = m11_pose;
        break;
    case 12:
        p = m12_pose;
        break;
    }
    return p;
}

void Mission_publish::timerCallback(const ros::TimerEvent &e)
{
    if (flag)
    {
        if (robot1_push_n)
        {
            if (missionList[0].size() != 0)
            {
                int m = missionList[0].front();
                missionList[0].erase(missionList[0].begin());
                geometry_msgs::PoseStamped p = get_mission_pose(m);
                mission1_pub.publish(p);
                ROS_INFO("robot 1 mission num is : %d\n", m);
                if (missionList[0].size() == 0)
                {
                    ROS_INFO("robot1 mission finished\n");
                }
            }
            robot1_push_n = false;
        }
        if (robot2_push_n)
        {
            if (missionList[1].size() != 0)
            {
                int m = missionList[1].front();
                missionList[1].erase(missionList[1].begin());
                geometry_msgs::PoseStamped p = get_mission_pose(m);
                mission2_pub.publish(p);
                if (missionList[1].size() == 0)
                {
                    ROS_INFO("robot2 mission finished\n");
                }
            }
            robot2_push_n = false;
        }
        if (robot3_push_n)
        {
            if (missionList[2].size() != 0)
            {
                int m = missionList[2].front();
                missionList[2].erase(missionList[2].begin());
                geometry_msgs::PoseStamped p = get_mission_pose(m);
                mission3_pub.publish(p);
                if (missionList[2].size() == 0)
                {
                    ROS_INFO("robot3 mission finished\n");
                }
            }
            robot3_push_n = false;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_publish");
    ros::NodeHandle nh("");
    Mission_publish mission_publish(nh);
    while (ros::ok())
    {
        ros::spinOnce();
    }
}