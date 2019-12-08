#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <actionlib/server/simple_action_server.h>
#include <velo_with_zed_rs/zedAction.h>

#include "./../include/odom_zed.h"

namespace VeloWithZedRs
{
    ZedQuaternion::ZedQuaternion(ros::NodeHandle &nh) :
        success(false),
        server(nh, "zed", false),
        sub_odom(nh.subscribe("/zed/zed_node/odom", 10, &ZedQuaternion::odom_cb, this))
    {
        server.registerGoalCallback(boost::bind(&ZedQuaternion::goal_cb, this));
        server.registerPreemptCallback(boost::bind(&ZedQuaternion::preempt_cb, this));
        server.start();
    } 


    ZedQuaternion::~ZedQuaternion()
    {
    }


    void ZedQuaternion::set_ts_zed(
        const tf2::Vector3 &translation,
        const tf2::Quaternion &rotation,
        geometry_msgs::TransformStamped &ts)
    {
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "map";
        ts.child_frame_id = "zed";

	    ts.transform.translation.x = translation.x();
	    ts.transform.translation.y = translation.y();
	    ts.transform.translation.z = translation.z();

	    ts.transform.rotation.x = rotation.x();
	    ts.transform.rotation.y = rotation.y();
	    ts.transform.rotation.z = rotation.z();
	    ts.transform.rotation.w = rotation.w();
    }


    void ZedQuaternion::odom_cb(
        const nav_msgs::OdometryConstPtr& msg)
    {
        odom.pose = msg->pose;
        success = true;
    }

    void ZedQuaternion::set_qt(
        geometry_msgs::TransformStamped &ts)
    {
        if(success)
        {
            // Camera position in map frame
            tf2::Vector3 t;
                double x = odom.pose.pose.position.x;
                double y = odom.pose.pose.position.y;
                double z = odom.pose.pose.position.z;
                t.setValue(x, y, z);

            // Orientation quaternion
            tf2::Quaternion q(
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w);

            geometry_msgs::TransformStamped ts;

            set_ts_zed(t, q, ts);
        }
        else
        {
            ROS_WARN("Could not odometry in ZED");
        }
    }


    void ZedQuaternion::goal_cb()
    {
        if(server.isNewGoalAvailable())
        {
          goal = server.acceptNewGoal();
        }

        geometry_msgs::TransformStamped ts;

        while(ros::ok())
        {
            set_qt(ts);

            if(success)
            {
                success = false;
                result.success_zed.data = true;
                result.ts_zed = ts;
                server.setSucceeded(result, "zed_ts");
                break;
            }
            else
            {
                ros::Duration(1.0);
                ros::spinOnce();
                continue;
            }
        }
    }


    void ZedQuaternion::preempt_cb()
    {
        result.success_zed.data = false; 
        server.setPreempted(result, "zed_preempted");
    }

    void ZedQuaternion::run()
    {
        ros::spin();
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_zed");
    ros::NodeHandle nh;
    
    VeloWithZedRs::ZedQuaternion *zed_server;
    zed_server = new VeloWithZedRs::ZedQuaternion(nh);
    zed_server->run();
    delete zed_server;

    return 0;
}
