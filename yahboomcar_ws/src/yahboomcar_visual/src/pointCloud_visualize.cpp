//
// Created by yahboom on 2021/8/21.
//

#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
using namespace std;
class cloudHandler {
public:
    cloudHandler()
            : viewer("Cloud Viewer") {
        // 创建订阅者
        // Create a subscriber
        pcl_sub = nh.subscribe("/cloud_topic", 10, &cloudHandler::cloudCB, this);
        // 创建计时器
        // Create a timer
        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input) {
        // 创建点云
        // Create a point cloud
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        // 从话题中订阅的数据转成点云
        // Convert the subscribed data from the topic into a point cloud
        pcl::fromROSMsg(input, cloud);
        //  使用PCL可视化工具显示出来
        // Use the PCL visualization tool to display it
        viewer.showCloud(cloud.makeShared());
    }

    void timerCB(const ros::TimerEvent &) {
        // 关闭可视化工具时，关闭ROS节点
        // Close the ROS node when closing the visualization tool
        if (viewer.wasStopped()) {
            ros::shutdown();
        }
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;
};

main(int argc, char **argv) {
    ros::init(argc, argv, "pcl_visualize");
    cloudHandler handler;
    ros::spin();
    return 0;
}
