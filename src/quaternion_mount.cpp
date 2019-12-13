#include <Eigen/Core>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <geometry_msgs/TransformStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <velo_with_zed_rs/mountAction.h>

#include "./../include/quaternion_mount.h"

namespace VeloWithZedRs
{

    Mount::Mount(ros::NodeHandle &nh) :
        success(false),
        server(nh, "mount", false),
        degree_sub(nh.subscribe("degree", 1, &Mount::degree_cb, this))
    {
        ros::param::get("/rotation_broadcaster/x", x);
        ros::param::get("/rotation_broadcaster/y", y);
        ros::param::get("/rotation_broadcaster/z", z);

        server.registerGoalCallback(boost::bind(&Mount::goal_callback, this));
        server.registerPreemptCallback(boost::bind(&Mount::preempt_callback, this));
        server.start();
    } 


    Mount::~Mount()
    {
    }

    // 雲台から角度を得るコールバック関数
    void Mount::degree_cb(const std_msgs::Float32 &pitch_cb)
    {
	float degree;
	ros::param::get("/calib_velo/degree", degree);
        
	   
	pitch = pitch_cb.data;
        success = true;
    }

    // 回転行列Rの初期位置を設定
    void Mount::set_R(
                Eigen::Matrix3f &R,
                const float roll,
                const float pitch,
                const float yaw)
    {
        Eigen::Matrix3f R_tmp;
        Eigen::Matrix3f Rpi;
        Eigen::Matrix3f Rya;
        Eigen::Matrix3f Rro;

        Rro << 1, 0, 0,  
               0, cos(roll), -sin(roll),  
               0, sin(roll), cos(roll); 

        Rya << cos(yaw), 0, sin(yaw), 
               0, 1, 0, 
               -sin(yaw), 0, cos(yaw);

        Rpi << cos(pitch), -sin(pitch), 0, 
               sin(pitch), cos(pitch), 0, 
               0, 0, 1;

        R = Rro * Rpi * Rya;
    }


    Eigen::Vector3f Mount::set_diff_xyz(
        const float roll,
        const float pitch,
        const float yaw,
        const float x,
        const float y,
        const float z)
    {
        Eigen::Matrix3f R_3x3;
        Eigen::Vector3f t;
        t << x, y, z;

        set_R(R_3x3, roll, pitch, yaw);
        t = R_3x3*t;

        return t;
    }
	

    void Mount::set_ts(
        const float roll,
        const float pitch,
        const float yaw,
        geometry_msgs::TransformStamped &ts)
    {
        Eigen::Vector3f t;
        t = set_diff_xyz(x, y, z, roll, pitch, yaw);

        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "map";
        ts.child_frame_id = "velodyne";

        tf2::Vector3 translation;
	translation.setValue(t(0), t(1), t(2));
	ts.transform.translation.x = translation.x();
	ts.transform.translation.y = translation.y();
	ts.transform.translation.z = translation.z();

	tf2::Quaternion rotation;
	rotation.setRPY(0, pitch, 0);
	ts.transform.rotation.x = rotation.x();
	ts.transform.rotation.y = rotation.y();
	ts.transform.rotation.z = rotation.z();
	ts.transform.rotation.w = rotation.w();
    }


    void Mount::goal_callback()
    {
        if(server.isNewGoalAvailable())
        {
          goal = server.acceptNewGoal();
        }

        geometry_msgs::TransformStamped ts;

        while(ros::ok())
        {
            if(success)
            {
                set_ts(0.0, pitch, 0.0, ts);
                success = false;
                break;
            }
            ros::Duration(1.0);
            ros::spinOnce();
        }
        
        result.success_mount.data = true;
        result.ts_mount = ts;
        server.setSucceeded(result, "R");
    }


    void Mount::preempt_callback()
    {
        result.success_mount.data = false; 
        server.setPreempted(result, "preempted");
    }


    void Mount::run()
    {
        ros::spin();
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_mount");
    ros::NodeHandle nh;

    VeloWithZedRs::Mount *server;
    server = new VeloWithZedRs::Mount(nh);
    server->run();
    delete server;

    return 0;
}
