/*************************************************************************
	> File Name: generatePointCloud.cpp
	> Author: Hou
	> Mail: 745189913@qq.com
	> Created Time: 2016年12月21日 星期三 11时11分37秒
 ************************************************************************/

//c++标准库
#include <iostream>
#include <string>

using namespace std;

//OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//PCL库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//定义运点类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//相机内参
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

//主函数
int main( int argc, char** argv )
{
    //读取　./data/rgb.png和./data/depth.png，并转化为云点
    //图像矩阵
    cv::Mat rgb, depth;
    //使用cv::imread()来读取图像
    rgb = cv::imread("./data/rgb.png");
    cv::imshow("rgb", rgb);
    cv::waitKey(10);
    //rgb 图像为８UC3的彩色图像
    //depth 是16UC1的单通道图像，主要flags设置为-1，表示读取原始数据不做任何修改
    depth = cv::imread("./data/depth.png", -1);

    //点云变量
    //使用智能指针，创建一个空点云。这种指针用完后会自动释放
    PointCloud::Ptr cloud (new PointCloud);
    //遍历深度图
    for (int m = 0; m < depth.rows; m++)
    {
        for (int n = 0; n < depth.cols; n++)
        {
            //获取深度图中（m, n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            //d可能没有值，若如此，跳过此点
            if( d == 0 )
            {
                continue;
            }
            //d存在值，则向点云增加一个点
            PointT p;

            //计算这个点的空间坐标
            p.z = double(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            //从rgb图像中获取它的颜色
            //rgb是三通道的BGR格式图，所以按照下面发的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            //把p加入到点云中
            cloud->points.push_back( p );
        }
    }
    //设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"Point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile("./pointcloud.pcd", *cloud);
    //清除数据并退出
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;
}
