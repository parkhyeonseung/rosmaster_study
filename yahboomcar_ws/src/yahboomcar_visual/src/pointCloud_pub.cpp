//
// Created by yahboom on 2021/4/29.
//

#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv) {
    //初始化了一个节点
    // Initializes a node
    ros::init(argc, argv, "pub_pcl");
    ros::NodeHandle n;
    //建立了一个点云发布者
    // Create a point cloud publisher
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/color_cloud", 1000);
    //点云更新频率2Hz
    // Point cloud update frequency 2Hz
    ros::Rate rate(2);
    //点云大小为100
    // Point cloud size is 100
    unsigned int num_points = 100;
    //建立了一个pcl的点云（不能直接发布）
    // Create a PCL point cloud (cannot publish directly)
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    //点云初始化
    // Point cloud initialization
    cloud.points.resize(num_points);
    //建立一个可以直接发布的点云
    // Create a point cloud that can be published directly
    sensor_msgs::PointCloud2 output_msg;
    while (ros::ok()) {
        output_msg.header.stamp = ros::Time::now();
        for (int i = 0; i < num_points; i++) {
            //点云中每个点位于一个10*10*10的方块内随机分布，颜色也随机
            // rand () / (RAND_MAX + 1.0f)为[0.1)   rand () / (RAND_MAX )为[0.1]
            // Each point in the point cloud is randomly distributed in a 10*10*10 square, and the color is also random
            // rand () / (RAND_MAX + 1.0f) is [0.1)   rand () / (RAND_MAX ) is [0.1]
            cloud.points[i].x = 10 * rand() / (RAND_MAX + 1.0f);
            cloud.points[i].y = 10 * rand() / (RAND_MAX + 1.0f);
            cloud.points[i].z = 10 * rand() / (RAND_MAX + 1.0f);
            //(rand() % (b-a+1))+ a
            cloud.points[i].r = (rand() % (255 + 1));
            cloud.points[i].g = (rand() % (255 + 1));
            cloud.points[i].b = (rand() % (255 + 1));
        }
        //将点云转化为消息才能发布
        // Convert the point cloud to a message before publishing
        pcl::toROSMsg(cloud, output_msg);
        //frame_id为map，rviz中就不用改了
        //frame_id is map, rviz does not need to change
        output_msg.header.frame_id = "map";
        //发布出去
        // Publish
        pub.publish(output_msg);
        ROS_INFO("Published ... ");
        rate.sleep();
    }
    ros::spin();
    return 0;
}
