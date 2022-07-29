//
// Created by yahboom on 2022/2/7.
// 程序作用：可使用rqt_image_view工具查看日图像
//

#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
image_transport::Publisher pub;

void imgCallBack(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat img;
    cv::normalize(cv_ptr->image, img, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv_bridge::CvImage cv_img(cv_ptr->header, sensor_msgs::image_encodings::MONO8, img);
    pub.publish(cv_img.toImageMsg());
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "IR_transform", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    const image_transport::Subscriber &sub = it.subscribe("/camera/ir/image", 1, imgCallBack);
    pub = it.advertise("/camera/ir/image_mono8", 1);
    ros::spin();
    return 0;
}



