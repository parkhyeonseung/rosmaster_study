/**
 * @function 接受RGB图像 Depth深度图像  相机位姿Tcw
 *
 * @param topicColor
 * @param topicDepth
 * @param topicTcw
 * @param cameraParamFile  
 * 修改于高翔发布的点云构建线程
 * cenruping@vip.qq.com
 */

#include <iostream>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include "PCLMapper.h"

using namespace std;


int main(int argc, char **argv) {

    std::string cameraParamFile;

    ros::init(argc, argv, "pointcloud_mapping", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    if (!ros::ok()) {
        cout << "ros init error..." << endl;
        return 0;
    }
    ros::start();
    Mapping::PointCloudMapper mapper;
    mapper.viewer();
    cout << "ros shutdown ..." << endl;
    ros::waitForShutdown();
    ros::shutdown();
    return 0;
}