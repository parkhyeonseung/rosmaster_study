/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include <chrono>
#include <ctime>
#include <climits>
#include <Eigen/Core>
#include <Eigen/Geometry>  // Eigen 几何模块
#include <opencv2/highgui/highgui.hpp>
#include "PCLMapper.h"
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
////    	cout<<RED<<"compute the intensity of features ..."<<WHITE<<endl;
using namespace std;

namespace Mapping {

/*
 * 
 * @ 设置点云分辨率
 */
    PointCloudMapper::PointCloudMapper()
            : nh("~"), spinner(0), it(nh) {
        float fx_, fy_, cx_, cy_, resolution_, depthfactor_;
        int queueSize_;
        bool mbuseExact_;

        mbuseCompressed = false;
        lastKeyframeSize = 0;
        mGlobalPointCloudID = 0; //点云ID
        mLastGlobalPointCloudID = 0;
        queueSize = 10;

        std::string topicColor, topicDepth, topicTcw, topicPointCloud;

        topicColor = nh.param<std::string>("topicColor", "/RGBD/RGB/Image");
        topicDepth = nh.param<std::string>("topicDepth", "/RGBD/Depth/Image");
        topicTcw = nh.param<std::string>("topicTcw", "/RGBD/CameraPose");
        local_frame_id = nh.param<std::string>("local_frame_id", "camera");
        global_frame_id = nh.param<std::string>("global_frame_id", "camera");
        mNodePath = nh.param<std::string>("node_path", "./");
        use_viewer=nh.param<bool>("use_viewer", false);
        std::cout << "mNodePath: " << mNodePath << std::endl;

        nh.param<float>("fx", fx_, 517.306408);
        nh.param<float>("fy", fy_, 516.469215);
        nh.param<float>("cx", cx_, 318.643040);
        nh.param<float>("cy", cy_, 255.313989);
        nh.param<float>("resolution", resolution_, 0.05);
        nh.param<float>("depthfactor", depthfactor_, 1.0);

        nh.param<int>("queueSize", queueSize_, 10);
        nh.param<bool>("buseExact", mbuseExact_, true);

        mbuseExact = mbuseExact_;  //
        queueSize = queueSize_;
        mcx = cx_;
        mcy = cy_;
        mfx = fx_;
        mfy = fy_;
        mresolution = resolution_;
        mDepthMapFactor = depthfactor_;

        image_transport::TransportHints hints(mbuseCompressed ? "compressed" : "raw");
        subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
        subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
        tcw_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, topicTcw, queueSize);
        
        cout << "use_viewer: " << use_viewer << endl;
        cout << "topicColor: " << topicColor << endl;
        cout << "topicDepth: " << topicDepth << endl;
        cout << "topicTcw: " << topicTcw << endl;
        cout << "local_frame_id: " << local_frame_id << endl;
        cout << "global_frame_id: " << global_frame_id << endl;

        cout << "fx: " << mfx << endl;
        cout << "fy: " << mfy << endl;
        cout << "cx: " << mcx << endl;
        cout << "cy: " << mcy << endl;
        cout << "resolution: " << mresolution << endl;
        cout << "DepthMapFactor: " << mDepthMapFactor << endl;
        cout << "queueSize: " << queueSize << endl;
        cout << "mbuseExact: " << mbuseExact << endl;

        //接受RGB DepTh 位姿数据
        if (mbuseExact) {
            syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor,
                                                                           *subImageDepth, *tcw_sub);
            syncExact->registerCallback(boost::bind(&PointCloudMapper::callback, this, _1, _2, _3));
        } else {
            syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize),
                                                                                       *subImageColor, *subImageDepth,
                                                                                       *tcw_sub);
            syncApproximate->registerCallback(boost::bind(&PointCloudMapper::callback, this, _1, _2, _3));
        }

        voxel.setLeafSize(mresolution, mresolution, mresolution);
        globalMap = PointCloud::Ptr(new PointCloud());
        localMap = PointCloud::Ptr(new PointCloud());

        pub_global_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("Global/PointCloudOutput", 1);
        pub_local_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("Local/PointCloudOutput", 10);

    }

    PointCloudMapper::~PointCloudMapper() {
        shutdown();
    }


// 由外部函数调用，每生成一个关键帧调用一次该函数
    void PointCloudMapper::insertKeyFrame(cv::Mat &color, cv::Mat &depth, geometry_msgs::PoseStamped &T) {
        unique_lock<mutex> lck(keyframeMutex);
        // 已测试接受到的数据没有问题
//        cout<< BLUE<<"--------------------------------T:\n"<<T.matrix()<<WHITE<<endl;
        mvGlobalPointCloudsPose.push_back(T);

        colorImgs.push_back(color.clone());
        depthImgs.push_back(depth.clone());


        //mLastGlobalPointCloudID=mGlobalPointCloudID;
        mGlobalPointCloudID++;
        mbKeyFrameUpdate = true;

        cout << GREEN << "--------------------------------receive a keyframe, id = " << mGlobalPointCloudID << WHITE
             << endl;
    }

/**
 * @function 更加关键帧生成点云、并对点云进行滤波处理
 * 备注：点云生成函数在　台式机上调用时间在0.1ｓ 左右
 */
    pcl::PointCloud<PointCloudMapper::PointT>::Ptr
    PointCloudMapper::generatePointCloud(cv::Mat &color, cv::Mat &depth, Eigen::Isometry3d &T) {
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        PointCloud::Ptr tmp(new PointCloud());// point cloud is null ptr

        for (int m = 0; m < depth.rows; m += 3) {
            for (int n = 0; n < depth.cols; n += 3) {
                float d = depth.ptr<float>(m)[n] / mDepthMapFactor;
                if (d < 0.01 || d > 10) {
                    continue;
                }
                PointT p;
                p.z = d;
                p.x = (n - mcx) * p.z / mfx;
                p.y = (m - mcy) * p.z / mfy;

                p.b = color.ptr<uchar>(m)[n * 3];
                p.g = color.ptr<uchar>(m)[n * 3 + 1];
                p.r = color.ptr<uchar>(m)[n * 3 + 2];

                tmp->points.push_back(p);
            }
        }

        PointCloud::Ptr cloud_voxel_tem(new PointCloud);
        tmp->is_dense = false;
        voxel.setInputCloud(tmp);
        voxel.setLeafSize(mresolution, mresolution, mresolution);
        voxel.filter(*cloud_voxel_tem);

        PointCloud::Ptr cloud1(new PointCloud);

        // 这里对点云进行变换
        pcl::transformPointCloud(*cloud_voxel_tem, *cloud1, T.matrix());

        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

        cout << GREEN << "generate point cloud from  kf-ID:" << mLastGlobalPointCloudID << ", size="
             << cloud1->points.size() << " cost time: " << time_used.count() * 1000 << " ms ." << WHITE << endl;
        mLastGlobalPointCloudID++;
        return cloud1;
    }


/*
 *
 * 由于随着尺寸的增加以后,显示函数会异常退出
 */
    void PointCloudMapper::viewer() {
        int N = 0, i = 0;
        bool KFUpdate = false;
        ros::AsyncSpinner spinner(2); // Use 1threads
        spinner.start();
        if (use_viewer) {
            pcl::visualization::CloudViewer pcl_viewer("viewer");
            while (ros::ok()) {
                {
                    unique_lock<mutex> lck_shutdown(shutDownMutex);
                    if (shutDownFlag) {
                        break;
                    }
                }
                // keyframe is updated
                KFUpdate = false;
                {
                    unique_lock<mutex> lck(keyframeMutex);
                    N = (int) mvGlobalPointCloudsPose.size();
                    KFUpdate = mbKeyFrameUpdate;
                    mbKeyFrameUpdate = false;
//                std::cout << "---------------------------lck mbKeyFrameUpdate N:" << mbKeyFrameUpdate << std::endl;
                }
                if (KFUpdate) {
                    std::cout << "---------------------------KFUpdate N:" << N << std::endl;
                    for (i = lastKeyframeSize; i < N && i < (lastKeyframeSize + 5); i++) {
                        if ((mvGlobalPointCloudsPose.size() != colorImgs.size()) ||
                            (mvGlobalPointCloudsPose.size() != depthImgs.size()) ||
                            (depthImgs.size() != colorImgs.size())) {
                            cout << " depthImgs.size != colorImgs.size()  " << endl;
                            continue;
                        }
                        PointCloud::Ptr tem_cloud1(new PointCloud());
                        cout << "i: " << i << "  mvPosePointClouds.size(): " << mvGlobalPointCloudsPose.size() << endl;
                        //生成一幅点云大约在0.1s左右 点云数据
                        Eigen::Isometry3d transform = convert2Eigen(mvGlobalPointCloudsPose[i]);
                        //
                        tem_cloud1 = generatePointCloud(colorImgs[i], depthImgs[i], transform);

                        if (tem_cloud1->empty()) continue;

                        *globalMap += *tem_cloud1;

                        sensor_msgs::PointCloud2 local;
                        pcl::toROSMsg(*tem_cloud1, local);// 转换成ROS下的数据类型 最终通过topic发布
                        local.header.stamp = ros::Time::now();
                        local.header.frame_id = local_frame_id;
                        pub_local_pointcloud.publish(local);
                    }
                    {
                        int buff_length = 150;
                        if (i > (buff_length + 5)) {
                            cout << "i =" << i << endl;
                            // shutdown();
                            // break;
                        }
                    }
                    lastKeyframeSize = i;
                    sensor_msgs::PointCloud2 output;
                    pcl::toROSMsg(*globalMap, output);
                    output.header.stamp = ros::Time::now();
                    output.header.frame_id = global_frame_id;
                    pub_global_pointcloud.publish(output);
                    pcl_viewer.showCloud(globalMap);
                    cout << "show global map, size=" << globalMap->points.size() << endl;
                }
            }

        } else{
            while (ros::ok()) {
                {
                    unique_lock<mutex> lck_shutdown(shutDownMutex);
                    if (shutDownFlag) {
                        break;
                    }
                }
                // keyframe is updated
                KFUpdate = false;
                {
                    unique_lock<mutex> lck(keyframeMutex);
                    N = (int) mvGlobalPointCloudsPose.size();
                    KFUpdate = mbKeyFrameUpdate;
                    mbKeyFrameUpdate = false;
//                std::cout << "---------------------------lck mbKeyFrameUpdate N:" << mbKeyFrameUpdate << std::endl;
                }
                if (KFUpdate) {
                    std::cout << "---------------------------KFUpdate N:" << N << std::endl;
                    for (i = lastKeyframeSize; i < N && i < (lastKeyframeSize + 5); i++) {
                        if ((mvGlobalPointCloudsPose.size() != colorImgs.size()) ||
                            (mvGlobalPointCloudsPose.size() != depthImgs.size()) ||
                            (depthImgs.size() != colorImgs.size())) {
                            cout << " depthImgs.size != colorImgs.size()  " << endl;
                            continue;
                        }
                        PointCloud::Ptr tem_cloud1(new PointCloud());
                        cout << "i: " << i << "  mvPosePointClouds.size(): " << mvGlobalPointCloudsPose.size() << endl;
                        //生成一幅点云大约在0.1s左右 点云数据
                        Eigen::Isometry3d transform = convert2Eigen(mvGlobalPointCloudsPose[i]);
                        //
                        tem_cloud1 = generatePointCloud(colorImgs[i], depthImgs[i], transform);

                        if (tem_cloud1->empty()) continue;

                        *globalMap += *tem_cloud1;

                        sensor_msgs::PointCloud2 local;
                        pcl::toROSMsg(*tem_cloud1, local);// 转换成ROS下的数据类型 最终通过topic发布
                        local.header.stamp = ros::Time::now();
                        local.header.frame_id = local_frame_id;
                        pub_local_pointcloud.publish(local);
                    }
                    {
                        int buff_length = 150;
                        if (i > (buff_length + 5)) {
                            cout << "i =" << i << endl;
                            // shutdown();
                            // break;
                        }
                    }
                    lastKeyframeSize = i;
                    sensor_msgs::PointCloud2 output;
                    pcl::toROSMsg(*globalMap, output);
                    output.header.stamp = ros::Time::now();
                    output.header.frame_id = global_frame_id;
                    pub_global_pointcloud.publish(output);
                    cout << "show global map, size=" << globalMap->points.size() << endl;
                }
            }
        }
    }


    Eigen::Matrix4f PointCloudMapper::cvMat2Eigen(const cv::Mat &cvT) {
        Eigen::Matrix<float, 4, 4> T;
        T << cvT.at<float>(0, 0), cvT.at<float>(0, 1), cvT.at<float>(0, 2), cvT.at<float>(0, 3),
                cvT.at<float>(1, 0), cvT.at<float>(1, 1), cvT.at<float>(1, 2), cvT.at<float>(1, 3),
                cvT.at<float>(2, 0), cvT.at<float>(2, 1), cvT.at<float>(2, 2), cvT.at<float>(2, 3),
                0, 0, 0, 1;

        return T;
    }
    /**
   * @ 将深度图像映射成彩色图
   *
   */
    void PointCloudMapper::dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue) {
        cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
        const uint32_t maxInt = 255;

        for (int r = 0; r < in.rows; ++r) {
            const uint16_t *itI = in.ptr<uint16_t>(r);
            uint8_t *itO = tmp.ptr<uint8_t>(r);

            for (int c = 0; c < in.cols; ++c, ++itI, ++itO) {
                *itO = (uint8_t) std::min((*itI * maxInt / maxValue), 255.0f);
            }
        }

        cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
    }

    void PointCloudMapper::callback(const sensor_msgs::Image::ConstPtr msgRGB,
                                    const sensor_msgs::Image::ConstPtr msgD,
                                    const geometry_msgs::PoseStamped::ConstPtr tcw) {

        std::cout << "----------------------------callback" << std::endl;

        cv::Mat color, depth, depthDisp;
        geometry_msgs::PoseStamped Tcw = *tcw;
        cv_bridge::CvImageConstPtr pCvImage;

        pCvImage = cv_bridge::toCvShare(msgRGB, "bgr8");
        pCvImage->image.copyTo(color);
        pCvImage = cv_bridge::toCvShare(msgD, msgD->encoding); //imageDepth->encoding
        pCvImage->image.copyTo(depth);
        // IR image input
        if (color.type() == CV_16U) {
            cv::Mat tmp;
            color.convertTo(tmp, CV_8U, 0.02);
            cv::cvtColor(tmp, color, CV_GRAY2BGR);
        }
        if (depth.type() != CV_32F)
            depth.convertTo(depth, CV_32F);

        insertKeyFrame(color, depth, Tcw);
    }

    Eigen::Isometry3d PointCloudMapper::convert2Eigen(geometry_msgs::PoseStamped &Tcw) {
        Eigen::Quaterniond q = Eigen::Quaterniond(Tcw.pose.orientation.w, Tcw.pose.orientation.x,
                                                  Tcw.pose.orientation.y, Tcw.pose.orientation.z);
        Eigen::AngleAxisd V6(q);
        //  V6.fromRotationMatrix<double,3,3>(q.toRotationMatrix());
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();  // 三维变换矩阵
        T.rotate(V6);  // 旋转部分赋值
        T(0, 3) = Tcw.pose.position.x;
        T(1, 3) = Tcw.pose.position.y;
        T(2, 3) = Tcw.pose.position.z;
        return T;
    };

    void PointCloudMapper::getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &outputMap) {
        unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
        outputMap = globalMap;
    }

// 复位点云显示模块
    void PointCloudMapper::reset() {
        mvGlobalPointCloudsPose.clear();
        mvGlobalPointClouds.clear();
        mGlobalPointCloudID = 0;
        mLastGlobalPointCloudID = 0;
    }

    void PointCloudMapper::shutdown() {
        {
            unique_lock<mutex> lck(shutDownMutex);
            shutDownFlag = true;
        }
        string save_path = mNodePath + "/resultPointCloudFile.pcd";
        pcl::io::savePCDFile(save_path, *globalMap, true);
        cout << "save pcd files to :  " << save_path << endl;
    }
// -----end of namespace
}
