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
#include <map>

using namespace std;

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//定义类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//参数文件读取类
class ParameterReader
{
public:
	ParameterReader( string filename = "./parameters.txt")
	{
		ifstream fin(filename.c_str());
		if(!fin)
		{
			cerr<<"parameter file does not exist."<<endl;
			return;
		}
		while(!fin.eof())
		{
			string str;
			getline(fin, str);
			if(str[0] == '#')
			{
				//以"#"开头的是注释
				continue;
			}

			int pos = str.find("=");
			if(pos == -1)
			{
				continue;
			}
			string key = str.substr(0, pos);
			string value = str.substr(pos+1, str.length());
			data[key] = value;
			
			if(!fin.good())
			{
				break;
			}
		}	
	}
	string getData(string key)
	{
		map<string, string>::iterator iter = data.find(key);
		if(iter == data.end())
		{
			cerr<<"Parameter name "<<key<<"not found!"<<endl;
			return string("NOT_FOUND");
		}
		return iter->second;
	}
public:
	map<string, string> data;
};

//相机内参
struct CAMERA_INTRINSIC_PARAMETERS
{
    double cx, cy, fx, fy, scale;
};

//图像帧结构
struct FRAME
{
	cv::Mat rgb, depth;//该帧对应的彩色图像和深度图像
	cv::Mat desp; //特征描述子
	vector<cv::KeyPoint> kp;//关键点
};

//PnP求解结果
struct RESULT_OF_PNP
{
	cv::Mat rvec, tvec;//旋转向量和平移向量
	int inliers;
};

//函数接口
//image2PointCloud将rgb转换为点云
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, 
CAMERA_INTRINSIC_PARAMETERS& camera );

//point2dTo3d将单个点从图像坐标转换为空间坐标
//input: 3维点Point3f (u, v, d)
cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera);

//同时提取关键点与特征描述子
void computeKeyPointsAndDesp(FRAME& frame, string detector);

//估计两帧图像之间的运动
RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera);


