#ifndef VELO_ZED_H
#define VELO_ZED_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pcl/point_types.h>

#include <actionlib/client/simple_action_client.h>
#include <velo_with_zed_rs/mountAction.h>
#include <velo_with_zed_rs/zedAction.h>

namespace VeloWithZedRs
{
    class VeloTransform
    {

    private:

        actionlib::SimpleActionClient<velo_with_zed_rs::mountAction> client_mount;
        velo_with_zed_rs::mountResultConstPtr result_mount;
        velo_with_zed_rs::mountGoal goal_mount;

        actionlib::SimpleActionClient<velo_with_zed_rs::zedAction> client_zed;
        velo_with_zed_rs::zedResultConstPtr result_zed;
        velo_with_zed_rs::zedGoal goal_zed;

        unsigned int count;
        bool send_degree;
        bool success_set_ts;
        int degree;

        std::string dir_path;
        std::string file_name;

        std::string cloud_frame;

        ros::Subscriber cloud_sub;
        ros::Publisher degree_pub;
        std_msgs::Int16 pub_degree;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

        void get_params();

        void set_ts_initialized(
            geometry_msgs::TransformStamped &ts);
        
        void set_all_ts(
            const tf2::Vector3 &t,
            const tf2::Quaternion &q,
            geometry_msgs::TransformStamped &ts);

        void add_tlanslate_and_quaternion(
            const tf2::Vector3 &t_mount,
            const tf2::Vector3 &t_zed,
            tf2::Vector3 &t,
            const tf2::Quaternion &q_mount,
            const tf2::Quaternion &q_zed,
            tf2::Quaternion &q);

        void ts_to_vec_quaternion(
            const geometry_msgs::TransformStamped &ts,
            tf2::Vector3 &t,
            tf2::Quaternion &q);

        /**
         *  mountAction 
         **/
        void get_mount_ts(
            geometry_msgs::TransformStamped &ts);

        /**
         *  zedAction
         **/
        void get_zed_ts(
            geometry_msgs::TransformStamped &ts);

        /**
         *  get pointcloud2 from velodyne
         **/
        void getpc2_cb(
            const sensor_msgs::PointCloud2 &cloud_msgs);

    public:

        VeloTransform(ros::NodeHandle &nh);
        ~VeloTransform();
        void run();

    };
}

#endif
