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
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "h264_decoder.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include "PracticalSocket.h" // For UDPSocket and SocketException
#include <cstdlib>           // For atoi()
#include "config.h"

#define BUF_LEN 65540 // Larger than maximum UDP packet size
#define SERV_PORT 9999

int sock_fd;
int len;
bool set_rect = false;
std::string command;

int hdr_stat = 0;
int flip_stat = 0;
int detect_stat = 0;
int contrastValue = 50; // 对比度
int brightValue = 50;   // 亮度值

int sendSocketData()
{
    controlData tempData;
    tempData.use_flip = (bool)flip_stat;
    tempData.use_ssr = (bool)hdr_stat;
    tempData.use_detect = (bool)detect_stat;
    tempData.contrast = contrastValue;
    tempData.bright = brightValue;

    struct sockaddr_in send_addr;
    memset(&send_addr, 0, sizeof(struct sockaddr_in));
    send_addr.sin_family = AF_INET;
    send_addr.sin_port = htons(9999);
    send_addr.sin_addr.s_addr = inet_addr("192.168.1.128");
    if (sendto(sock_fd, (controlData *)&tempData, sizeof(tempData), 0, (sockaddr *)&send_addr, sizeof(send_addr)) <= 0)
    {
        printf("send data error");
    }
}

int getSocketData(targetInfo temp)
{
    int recv_num;
    // char recv_buf[20];
    struct sockaddr_in addr_client;
    recv_num = recvfrom(sock_fd, (targetInfo *)&temp, sizeof(temp), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);

    if (recv_num < 0)
    {
        perror("recvfrom error:");
        exit(1);
        return 0;
    }
    if (temp.target_header == 0xFFEEAABB)
    {
        std::cout << "recv data: " << std::endl;

        std::cout << temp.target_id << ", ";
        std::cout << temp.target_x << ", ";
        std::cout << temp.target_y << ", ";
        std::cout << temp.target_w << ", ";
        std::cout << temp.target_h << ", ";

        std::cout << std::endl;
        return 1;
    }
}

int initSocketData()
{
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }

    struct sockaddr_in addr_serv;

    memset(&addr_serv, 0, sizeof(struct sockaddr_in));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_port = htons(SERV_PORT);
    addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);
    len = sizeof(addr_serv);

    if (bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
    {
        perror("bind error:");
        exit(1);
    }
    else
        return 0;
}

void send_bbox_message(targetInfo temp)
{
    autoware_msgs::DetectedObjectArray out_message;

    out_message.objects.clear();

    autoware_msgs::DetectedObject obj;

    obj.x = int(temp.target_x);
    obj.y = int(temp.target_y);
    obj.width = int(temp.target_w);
    obj.height = int(temp.target_h);
    if (obj.x < 0)
        obj.x = 0;
    if (obj.y < 0)
        obj.y = 0;
    if (obj.width < 0)
        obj.width = 0;
    if (obj.width < 0)
        obj.height = 0;

    if (temp.target_class == 0)
    {
        obj.label = "person";
    }
    else if (temp.target_class == 1)
    {
        obj.label = "car";
    }
    else if (temp.target_class == 2)
    {
        obj.label = "cyclist";
    }
    else
    {
        obj.label = "unknow";
    }
    obj.score = 1;
    obj.valid = true;
    std::cout << obj.x << ", " << obj.y << ", " << obj.width << ", " << obj.label << std::endl;
    out_message.objects.push_back(obj);
}

cv::Mat point_cloud_box::pointcloud_box(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, double scale)
{
    cv::Mat BEV(BEV_height, BEV_width, CV_8UC1, cv::Scalar(0));
    // pcl::PointXYZ min; //
    // pcl::PointXYZ max; //

    pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_out(new pcl::PointCloud<pcl::PointXYZ>); // 建立一个点云指针

    pcl::copyPointCloud(*cloud_in, *clouds_out);
    // pcl::getMinMax3D(*clouds_out, min, max);

    point_filter(clouds_out, offset_x - scale * (Box_width / 2), offset_x + scale * (Box_width / 2), "x", false);
    point_filter(clouds_out, offset_y - scale * (Box_height / 2), offset_y + scale * (Box_height / 2), "y", false);
    point_filter(clouds_out, min_z_, pass_z_, "z", false);

    for (int i = 0; i < clouds_out->points.size(); i++)
    {
        int x_img = int((clouds_out->points[i].x + (scale * (Box_width / 2) - offset_x)) * BEV_width / (scale * Box_width));
        int y_img = int((clouds_out->points[i].y + (scale * (Box_height / 2) - offset_y)) * BEV_height / (scale * Box_height));
        if (x_img < BEV_width && y_img < BEV_height)
            BEV.at<uchar>(y_img, x_img) = scale_to_255(clouds_out->points[i].z, min_z_, pass_z_);

        // std::cout << x_img << "," << y_img << "  ";
    }
    std::cout << "pointcloud_box" << std::endl;
    cv::flip(BEV, BEV, 1);

    cv::applyColorMap(BEV, BEV, cv::COLORMAP_JET);

    Rotate(BEV, BEV, angle_);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", BEV).toImageMsg();

    obj_pub.publish(msg);

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
    double start_time = ros::Time::now().toSec();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_curr(new pcl::PointCloud<pcl::PointXYZ>);           // 建立一个点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>); // 建立一个点云指针
    pcl::fromROSMsg(*cloud_msg, *pc_curr);

    // float theta = 0;

    // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX())); //同理，UnitX(),绕X轴；UnitY(),绕Y轴
    // pcl::transformPointCloud(*pc_curr, *transformed_cloud, transform);

    cv::Mat BEV = pointcloud_box(pc_curr, 10);

    targetInfo temp;

    temp.target_x = 100;
    temp.target_y = 100;
    temp.target_w = 200;
    temp.target_h = 200;
    temp.target_class = 0;

    send_bbox_message(temp);

    double end_time = ros::Time::now().toSec();

    printf("deal this frame takes %f ms\n\n", (end_time - start_time) * 1000);
}

void point_cloud_box::createROSPubSub()
{
    image_transport::ImageTransport it(nh_);

    obj_pub = it.advertise("image", 100);

    Img_sub = nh_.subscribe("/livox/lidar", 100, &point_cloud_box::cloudCallback, this); // image_raw
}
