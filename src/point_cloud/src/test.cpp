#include "test.hpp"
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> // 里面是pcl库点云的数据类型
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

cv::Mat point_cloud_box::pointcloud_box(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, const int offset_x, const int offset_y, const float &pass_z)
{

    cv::Mat BEV(BEV_height, BEV_width, CV_8UC1, cv::Scalar(0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_out(new pcl::PointCloud<pcl::PointXYZ>); // 建立一个点云指针

    pcl::copyPointCloud(*cloud_in, *clouds_out);

    point_filter(clouds_out, offset_x - Box_width / 2, offset_x + Box_width / 2, "x", false);
    point_filter(clouds_out, offset_y - Box_height / 2, offset_y + Box_height / 2, "y", false);
    point_filter(clouds_out, -pass_z, pass_z, "z", false);

    for (int i = 0; i < clouds_out->points.size(); i++)
    {
        int y_img = int((clouds_out->points[i].x + (Box_width / 2 - offset_x)) * BEV_height / Box_height);
        int x_img = int((clouds_out->points[i].y + (Box_height / 2 - offset_y)) * BEV_width / Box_width);
        if (x_img < BEV_width && y_img < BEV_height)
            BEV.at<uchar>(x_img, y_img) = scale_to_255(clouds_out->points[i].z, -pass_z, pass_z);

        // std::cout << x_img << "," << y_img << "  ";
    }
    std::cout << "pointcloud_box" << std::endl;
    cv::flip(BEV, BEV, 1);

    return BEV;
}

int point_cloud_box::scale_to_255(const float &H, const float &min, const float &max)
{
    return int(((H - min) / (max - min)) * 255);
}

// 图像旋转
///@ angle 要旋转的角度
void point_cloud_box::Rotate(const cv::Mat &srcImage, cv::Mat &destImage, double angle)
{
    cv::Point2f center(srcImage.cols / 2, srcImage.rows / 2);                       //中心
    cv::Mat M = cv::getRotationMatrix2D(center, angle, 1);                          //计算旋转的仿射变换矩阵
    cv::warpAffine(srcImage, destImage, M, cv::Size(srcImage.cols, srcImage.rows)); //仿射变换
}

void point_cloud_box::point_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const int min, const int max, std::string axis, bool setFilterLimitsNegative)
{
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setFilterFieldName(axis);

    filter.setFilterLimits(min, max);

    if (setFilterLimitsNegative == true)
    {
        filter.setFilterLimitsNegative(true);
    }

    filter.filter(*cloud);
}

void point_cloud_box::cloudCallback(const sensor_msgs::PointCloud2::Ptr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_curr(new pcl::PointCloud<pcl::PointXYZ>);           // 建立一个点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>); // 建立一个点云指针
    pcl::fromROSMsg(*cloud_msg, *pc_curr);

    float theta = -M_PI / 4;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX())); //同理，UnitX(),绕X轴；UnitY(),绕Y轴
    pcl::transformPointCloud(*pc_curr, *transformed_cloud, transform);

    cv::Mat BEV = pointcloud_box(pc_curr, 10, 0, 2.0);

    std::cout << "save" << std::endl;

    cv::applyColorMap(BEV, BEV, cv::COLORMAP_JET);

    Rotate(BEV, BEV, angle_);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", BEV).toImageMsg();

    obj_pub.publish(msg);
}

void point_cloud_box::createROSPubSub()
{
    image_transport::ImageTransport it(nh_);

    obj_pub = it.advertise("image", 100);

    Img_sub = nh_.subscribe("/livox_full_cloud", 100, &point_cloud_box::cloudCallback, this); // image_raw
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_box");

    point_cloud_box pcb;

    pcb.createROSPubSub();

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return (0);
}