
#ifndef _ODOM_ZED_H_
#define _ODOM_ZED_H_

#include <Eigen/Core>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TransformStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <velo_with_zed_rs/zedAction.h>

namespace VeloWithZedRs
{
    class ZedQuaternion
    {
    private:

        bool success;
        ros::Subscriber sub_odom;
        nav_msgs::Odometry odom;

        actionlib::SimpleActionServer<velo_with_zed_rs::zedAction> server;
        velo_with_zed_rs::zedFeedback feedback;
        velo_with_zed_rs::zedResult result;
        velo_with_zed_rs::zedGoalConstPtr goal;


        void set_R(
            Eigen::Matrix3f &R,
            const float roll,
            const float pitch,
            const float yaw);

        void set_ts_zed(
            const tf2::Vector3 &translation,
            const tf2::Quaternion &rotation,
            geometry_msgs::TransformStamped &ts);

        void set_qt(
            geometry_msgs::TransformStamped &ts);

        void odom_cb(
            const nav_msgs::Odometry::ConstPtr& msg);

        void goal_cb();
        void preempt_cb();

    public:

        ZedQuaternion(ros::NodeHandle &nh);
        ~ZedQuaternion();
        void run();
    };
}

#endif
