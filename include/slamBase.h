/*************************************************************************
	> File Name: slamBase.h
	> Author: Liu Hengli
	> Mail: 745189913@qq.com
	> Created Time: 2016年12月21日 星期三 21时40分09秒
 ************************************************************************/

#ifndef _SLAMBASE_H
#define _SLAMBASE_H
#endif

//各种头文件
//Ｃ++标准库
#include <fstream>
#include <vector>

using namespace std;

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//定义类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//相机内参
struct CMAERA_INTRINSIC_PARAMETERS
{
    double cx, cy, fx, fy, scale;
};

//函数接口
//image2PointCloud将rgb转换为点云
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, 
CMAERA_INTRINSIC_PARAMETERS& camera );

//point2dTo3d将单个点从图像坐标转换为空间坐标
//input: 3维点Point3f (u, v, d)
cv::Point3f point2dTo3d(cv::Point3f& point, CMAERA_INTRINSIC_PARAMETERS& camera);


