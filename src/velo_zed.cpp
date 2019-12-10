#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <actionlib/client/simple_action_client.h>
#include <velo_with_zed_rs/mountAction.h>
#include <velo_with_zed_rs/zedAction.h>

#include "./../include/velo_zed.h"

namespace VeloWithZedRs
{
    VeloTransform::VeloTransform(ros::NodeHandle &nh) : 
        cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        cloud_frame ("/velodyne_points"),
        count (0),
        client_mount ("mount", true),
        client_zed ("zed", true),
        degree_pub (nh.advertise<std_msgs::Int16>("servo", 10)),
        cloud_sub(nh.subscribe(cloud_frame, 1, &VeloTransform::getpc2_cb, this))
    {
        get_params();
        if(!client_mount.waitForServer(ros::Duration(10.0)))
        {
            ROS_INFO("Could not find server : mount");
            ros::shutdown();
        }
        if(!client_zed.waitForServer(ros::Duration(10.0)))
        {
            ROS_INFO("Could not find server : zed");
            ros::shutdown();
        }
    }

    VeloTransform::~VeloTransform()
    {
    }

    void VeloTransform::get_params()
    {
        ros::param::get("/calib_velodyne/dir_path", dir_path);
        ros::param::get("/calib_velodyne/file_name", file_name);
        ros::param::get("/calib_velodyne/degree", degree);
    }


    void VeloTransform::set_ts_initialized(
        geometry_msgs::TransformStamped &ts)
    {
            ts.transform.rotation.x = 0.0;
            ts.transform.rotation.y = 0.0;
            ts.transform.rotation.z = 0.0;
            ts.transform.rotation.w = 1.0;
            ts.transform.translation.x = 0.0;
            ts.transform.translation.y = 0.0;
            ts.transform.translation.z = 0.0;
    }


    void ts_to_vec_quaternion(
        const geometry_msgs::TransformStamped &ts,
        tf2::Vector3 &t,
        tf2::Quaternion &q)
    {
        double t_x = ts.transform.translation.x; 
        double t_y = ts.transform.translation.y; 
        double t_z = ts.transform.translation.z; 
        t.setValue(t_x, t_y, t_z);

        double q_x = ts.transform.rotation.x; 
        double q_y = ts.transform.rotation.y; 
        double q_z = ts.transform.rotation.z; 
        double q_w = ts.transform.rotation.w; 

        q.setX(q_x);
        q.setY(q_y);
        q.setZ(q_z);
        q.setW(q_w);
    }


    void VeloTransform::get_mount_ts(
        geometry_msgs::TransformStamped &ts)
    {
        goal_mount.enable_mount.data = true;
        client_mount.sendGoal(goal_mount);

        bool finished = client_mount.waitForResult(ros::Duration(2.0));
        if(!finished)
        {
            ROS_WARN("rotation broadcaster TimeOut");
            success_set_ts = false;
            set_ts_initialized(ts);
        }
        else
        {
            success_set_ts = true;
            result_mount = client_mount.getResult();
            ts = result_mount->ts_mount;
        }
    }


    void VeloTransform::get_zed_ts(
        geometry_msgs::TransformStamped &ts)
    {
        goal_zed.enable_zed.data = true;
        client_zed.sendGoal(goal_zed);

        bool finished = client_zed.waitForResult(ros::Duration(2.0));
        if(!finished)
        {
            ROS_WARN("rotation broadcaster TimeOut");
            success_set_ts = false;
            set_ts_initialized(ts);
        }
        else
        {
            success_set_ts = true;
            result_zed = client_zed.getResult();
            ts = result_zed->ts_zed;
        }
    }


    void VeloTransform::getpc2_cb(
        const sensor_msgs::PointCloud2 &pc2)
    {
        sensor_msgs::PointCloud2 pc2_transformed;
        geometry_msgs::TransformStamped ts;
        geometry_msgs::TransformStamped ts_mount;
        geometry_msgs::TransformStamped ts_zed;

        tf2::Vector3 t;
        tf2::Vector3 t_mount;
        tf2::Vector3 t_zed;
        tf2::Quaternion q;
        tf2::Quaternion q_mount;
        tf2::Quaternion q_zed;

        Eigen::Matrix4f R;

        if(!send_degree)
        {
            ROS_INFO("Degree : %d", degree);
        }
        else
        {
            ROS_INFO("Degree : %d", degree);

            get_mount_ts(ts_mount);
            get_zed_ts(ts_zed);
            if(!success_set_ts)
            {
                ROS_WARN("Could not set ts");
            }
            else
            {
                R = tf2::transformToEigen(ts.transform).matrix().cast<float>();

                pcl_ros::transformPointCloud(R, 
                                             pc2, 
                                             pc2_transformed);

                pcl::fromROSMsg(pc2_transformed, *cloud);

                std::string format = ".pcd";
                std::string savename = dir_path 
                                     + file_name 
                                     + std::to_string(count) 
                                     + format; 
        
                pcl::io::savePCDFileASCII(savename, *cloud);
                count++;
            }
        }
    }

       
    void VeloTransform::run()
    {
        ros::Rate rate(1.0);
        pub_degree.data = 1000.0;

        while(ros::ok())
        {
            ros::param::get("/calib_velo/degree", degree);
            if(degree != pub_degree.data)
            {
                pub_degree.data = degree;
                degree_pub.publish(pub_degree);
                ros::Duration(7.0);
                send_degree = true;
            }
            else
            {
                ros::Duration(2.0);
                send_degree = false;
                continue;
            }
            rate.sleep();
            ros::spinOnce();
        }
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "velo_with_zed_rs");
    ros::NodeHandle nh;

    VeloWithZedRs::VeloTransform *get_pcl;
    get_pcl = new VeloWithZedRs::VeloTransform(nh);
    get_pcl->run();

    delete get_pcl; 
   
    return 0;
}
