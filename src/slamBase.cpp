/*************************************************************************
    > File Name: src/slamBase.cpp
    > Author: Liu Hengli
    > Implementation of slamBase.h
    > Created Time: 2016年12月21日
 ************************************************************************/

#include "slamBase.h"

PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    cv::Point3f p; // 3D 点
    p.z = double( point.z ) / camera.scale;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}

//computeKeyPointsAndDesp 同时提取关键点和特征描述子
//这里使用SIFT
//这里使用的是Opencv3.1.0与OpenCV2.X不同
void computeKeyPointsAndDesp(FRAME& frame)
{
    cv::Ptr<cv::Feature2D> _detector;
    _detector = cv::xfeatures2d::SIFT::create();
    if(!_detector)
    {
        cerr<<"Unknown detector type !"<<endl;
        return;
    }
   // _detector->detectAndCompute(frame.rgb, noArray(), frame.kp, frame.desp);//一次性计算关键点和描述子
   _detector->detect(frame.rgb, frame.kp);//计算关键点
   _detector->compute(frame.rgb, frame.kp, frame.desp);//计算描述子

   return;
}

//estimateMotion计算两个帧之间的运动
RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    static ParameterReader pd;
    vector<cv::DMatch> matches;
    cv::FlannBasedMatcher matcher;
    matcher.match(frame1.desp, frame2.desp, matches);

    cout<<"Find total"<<matches.size()<<"matches."<<endl;
    vector<cv::DMatch> goodMatches;
    double minDis = 9999;
    double good_match_threshold = atof(pd.getData("good_match_threshold").c_str());
    for(size_t i=0; i<matches.size();i++)
    {
        if(matches[i].distance < minDis)
        {
            minDis = matches[i].distance;
        }
    }
    for(size_t i=0; i<matches.size(); i++)
    {
        if(matches[i].distance < good_match_threshold * minDis)
        {
            goodMatches.push_back(matches[i]);
        }
    }

    cout<<"good matches:"<<goodMatches.size()<<endl;
    //第一帧的三维点
    vector<cv::Point3f> pts_obj;
    //第二帧的图像点
    vector<cv::Point2f> pts_img;

    //相机内参
    for(size_t i=0; i<goodMatches.size(); i++)
    {
        //query是第一帧，train是第二帧
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        //获取d是需要小心！ｘ是向右，ｙ是向下的，所以ｙ是行，ｘ是列
        ushort d = frame1.depth.ptr<ushort>(int(p.y))[int(p.x)];
        if(d == 0)
        {
            continue;
        }
        pts_img.push_back(cv::Point2f(frame2.kp[goodMatches[i].trainIdx].pt));

        //将(u,v,d)转成(x,y,z)
        cv::Point3f pt(p.x, p.y, d);
        cv::Point3f pd = point2dTo3d(pt, camera);
        pts_obj.push_back(pd);
    }

    //定义相机参数矩阵
    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    cout<<"开始求解PnP"<<endl;
    //构造相机矩阵
    cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
    cv::Mat rvec, tvec, inliers;
    //求解PnP
    const int iterCount = 100;
    const float maxDistance = 1.0;
    const double confidence = 0.95;//这个参数与opencv2.x不同，该参数的范围在(0,1)
    cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 
    iterCount, maxDistance, confidence, inliers);

    //存入结果
    RESULT_OF_PNP result;
    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    return result;
}