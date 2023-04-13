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
PointCloudXYZI pl_full;
PointCloudXYZI pl_buff[128]; //maximum 128 line lidar

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

void point_cloud_BEV::point_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const double min, const double max, std::string axis, bool setFilterLimitsNegative)
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

void point_cloud_BEV::RGB_point_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const double min, const double max, std::string axis, bool setFilterLimitsNegative)
{
    pcl::PassThrough<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud);
    filter.setFilterFieldName(axis);

    filter.setFilterLimits(min, max);

    if (setFilterLimitsNegative == true)
    {
        filter.setFilterLimitsNegative(true);
    }

    filter.filter(*cloud);
}

void point_cloud_BEV::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    pl_full.clear();
    int plsize = msg->point_num;
    pl_full.resize(plsize);

    //清空缓存内的点云并预留足够空间
    for(int i=0; i<6; i++)
    {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
    }
    omp_set_num_threads(12); //加速
#pragma omp parallel for
    for(uint i=1; i<plsize; i++)
    {
      if((msg->points[i].line < 6) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points

        std::cout << "intensity: " << pl_full[i].intensity << std::endl;

        bool is_new = false;
        //与前一点间距太小则忽略该点，间距太小不利于特征提取
        if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
            || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
            || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
        {
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }
}

void point_cloud_BEV::transform_cloud(const pcl::PointCloud<PointType>::Ptr &cloud_in, const pcl::PointCloud<PointType>::Ptr &transformed_cloud, const position target_pos, const angle target_angle)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(target_angle.yaw, Eigen::Vector3f::UnitZ())); //同理，UnitX(),绕X轴；UnitY(),绕Y轴
    transform.rotate(Eigen::AngleAxisf(target_angle.pitch, Eigen::Vector3f::UnitX())); //同理，UnitX(),绕X轴；UnitY(),绕Y轴
    transform.rotate(Eigen::AngleAxisf(target_angle.roll, Eigen::Vector3f::UnitY())); //同理，UnitX(),绕X轴；UnitY(),绕Y轴
    transform.translation() << target_pos.x, target_pos.y, target_pos.z;
    pcl::transformPointCloud(*cloud_in, *transformed_cloud, transform);
}

cv::Mat point_cloud_BEV::Point_cloud_BEV(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, double scale, double offset_x, double offset_y, double offset_z, std::vector<box> BBoxs)
{
    cv::Mat BEV(BEV_height, BEV_width, CV_8UC1, cv::Scalar(0));
    pcl::PointXYZ min; //
    pcl::PointXYZ max; //

    pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_out(new pcl::PointCloud<pcl::PointXYZ>); // 建立一个点云指针

    pcl::copyPointCloud(*cloud_in, *clouds_out);

    pcl::getMinMax3D(*clouds_out, min, max);

    point_filter(clouds_out, offset_x - scale * (Box_width / 2), offset_x + scale * (Box_width / 2), "x", false);
    point_filter(clouds_out, offset_y - scale * (Box_height / 2), offset_y + scale * (Box_height / 2), "y", false);
    point_filter(clouds_out, min_z_, pass_z_, "z", false);

    omp_set_num_threads(12); //加速
#pragma omp parallel for
    for (int i = 0; i < clouds_out->points.size(); i++)
    {
        int x_img = int((clouds_out->points[i].x + (scale * (Box_width / 2) - offset_x)) * BEV_width / (scale * Box_width));
        int y_img = int((clouds_out->points[i].y + (scale * (Box_height / 2) - offset_y)) * BEV_height / (scale * Box_height));
        // std::cout << x_img << "," << y_img << "  ";
        if (x_img < BEV_width && y_img < BEV_height && x_img > 0 && y_img > 0)
            BEV.at<uchar>(y_img, x_img) = scale_to_255(clouds_out->points[i].z, min_z_, pass_z_);

        // std::cout << x_img << "," << y_img << "  ";
    }
    
    // 画框
    // for (int i = 0; i < BBoxs.size(); i++)
    // {
    //     int x1 = int((BBoxs[i].x1 + (scale * (Box_width / 2) - offset_x)) * BEV_width / (scale * Box_width));
    //     int y1 = int((BBoxs[i].y1 + (scale * (Box_height / 2) - offset_y)) * BEV_height / (scale * Box_height));
    //     int x2 = int((BBoxs[i].x2 + (scale * (Box_width / 2) - offset_x)) * BEV_width / (scale * Box_width));
    //     int y2 = int((BBoxs[i].y2 + (scale * (Box_height / 2) - offset_y)) * BEV_height / (scale * Box_height));
    //     int x3 = int((BBoxs[i].x3 + (scale * (Box_width / 2) - offset_x)) * BEV_width / (scale * Box_width));
    //     int y3 = int((BBoxs[i].y3 + (scale * (Box_height / 2) - offset_y)) * BEV_height / (scale * Box_height));
    //     int x4 = int((BBoxs[i].x4 + (scale * (Box_width / 2) - offset_x)) * BEV_width / (scale * Box_width));
    //     int y4 = int((BBoxs[i].y4 + (scale * (Box_height / 2) - offset_y)) * BEV_height / (scale * Box_height));
    //     cv::line(BEV, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 3);
    //     cv::line(BEV, cv::Point(x2, y2), cv::Point(x3, y3), cv::Scalar(0, 0, 255), 3);
    //     cv::line(BEV, cv::Point(x3, y3), cv::Point(x4, y4), cv::Scalar(0, 0, 255), 3);
    //     cv::line(BEV, cv::Point(x4, y4), cv::Point(x1, y1), cv::Scalar(0, 0, 255), 3);
    // }

    cv::flip(BEV, BEV, 1);

    cv::applyColorMap(BEV, BEV, cv::COLORMAP_JET);

    Rotate(BEV, BEV, angle_);

    cv::rectangle(BEV, cv::Rect(620, 350, 40, 20), cv::Scalar(0, 0, 255),3);

    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", BEV).toImageMsg();

    // obj_pub.publish(msg);

    return BEV;
}

cv::Mat point_cloud_BEV::cloud_to_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, double scale, double offset_x, double offset_y, double offset_z)
{
    cv::Mat BEV(BEV_height, BEV_width, CV_8UC3, cv::Scalar(0));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds_out(new pcl::PointCloud<pcl::PointXYZRGB>); // 建立一个点云指针

    pcl::copyPointCloud(*cloud_in, *clouds_out);

    RGB_point_filter(clouds_out, offset_x - scale * (Box_width / 2), offset_x + scale * (Box_width / 2), "x", false);
    RGB_point_filter(clouds_out, offset_y - scale * (Box_height / 2), offset_y + scale * (Box_height / 2), "y", false);

    for (int i = 0; i < clouds_out->points.size(); i++)
    {
        int x_img = int((clouds_out->points[i].x + (scale * (Box_width / 2) - offset_x)) * BEV_width / (scale * Box_width));
        int y_img = int((clouds_out->points[i].y + (scale * (Box_height / 2) - offset_y)) * BEV_height / (scale * Box_height));
        // std::cout << x_img << "," << y_img << "  ";
        if (x_img < BEV_width && y_img < BEV_height && x_img > 0 && y_img > 0)
        {
            
            // BEV.at<uchar>(y_img, x_img) = scale_to_255(clouds_out->points[i].z, min_z_, pass_z_);
        }
        
        // std::cout << x_img << "," << y_img << "  ";
    }

    return BEV;

}

int point_cloud_BEV::scale_to_255(const float &H, const float &min, const float &max)
{
    return int(((H - min) / (max - min)) * 255);
}

// 图像旋转
///@ angle 要旋转的角度
void point_cloud_BEV::Rotate(const cv::Mat &srcImage, cv::Mat &destImage, double angle)
{
    cv::Point2f center(srcImage.cols / 2, srcImage.rows / 2);                       //中心
    cv::Mat M = cv::getRotationMatrix2D(center, angle, 1);                          //计算旋转的仿射变换矩阵
    cv::warpAffine(srcImage, destImage, M, cv::Size(srcImage.cols, srcImage.rows)); //仿射变换
}

void point_cloud_BEV::cloudCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    double start_time = ros::Time::now().toSec();
    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>); // 建立一个点云指针
    sensor_msgs::PointCloud2 laserCloudMap;
    position target_pos;
    angle target_angle;

    livox_pcl_cbk(msg);

    transform_cloud(pl_full.makeShared(), transformed_cloud, target_pos, target_angle);

    // 计算俯视图
    // cv::Mat BEV = Point_cloud_BEV(pc_curr, scale_, offset_x_, offset_y_, offset_z_, detect_BBoxs);

    // targetInfo temp;

    // temp.target_x = 100;
    // temp.target_y = 100;
    // temp.target_w = 200;
    // temp.target_h = 200;
    // temp.target_class = 0;

    // send_bbox_message(temp);
    pcl::toROSMsg(*transformed_cloud, laserCloudMap);
    laserCloudMap.header.frame_id = "livox_frame";
    publidarcloud.publish(laserCloudMap);

    double end_time = ros::Time::now().toSec();

    printf("deal this frame takes %f ms\n\n", (end_time - start_time) * 1000);
}

void point_cloud_BEV::gpsHandler(const nav_msgs::Odometry::ConstPtr &gpsMsg)
{
    offset_x_ = gpsMsg->pose.pose.position.x;
    offset_y_ = gpsMsg->pose.pose.position.y;
    offset_z_ = gpsMsg->pose.pose.position.z;
}

void point_cloud_BEV::controlHandler(const std_msgs::Float32MultiArray::ConstPtr &controlMsg)
{
    offset_x_ = offset_x_ + controlMsg->data[0];
    offset_y_ = offset_y_ + controlMsg->data[1];
    scale_ = controlMsg->data[2];
}

void point_cloud_BEV::detectHandler(const autoware_msgs::DetectedObjectArray::ConstPtr &input_detections)
{
    std::vector<box> detect_BBoxs;
    box detect_box;
    double half_l, half_w;
    tf::Quaternion RQ2;
    double roll, pitch, yaw;
    Eigen::Vector3d bottom_quad[4];
    for (int obj = 0; obj < input_detections->objects.size(); ++obj)
    {
        half_l = input_detections->objects[obj].dimensions.x / 2;
        half_w = input_detections->objects[obj].dimensions.y / 2;
        Eigen::Vector3d center(input_detections->objects[obj].pose.position.x,
                               input_detections->objects[obj].pose.position.y,
                               input_detections->objects[obj].pose.position.z);

        tf::quaternionMsgToTF(input_detections->objects[obj].pose.orientation, RQ2);
        tf::Matrix3x3(RQ2).getRPY(roll, pitch, yaw);

        Eigen::Vector3d ldir(cos(yaw), sin(yaw), 0);
        Eigen::Vector3d odir(-ldir[1], ldir[0], 0);
        bottom_quad[0] = center + ldir * -half_l + odir * -half_w; // A(-half_l, -half_w)
        bottom_quad[1] = center + ldir * -half_l + odir * half_w;  // B(-half_l, half_w)
        bottom_quad[2] = center + ldir * half_l + odir * half_w;   // C(half_l, half_w)
        bottom_quad[3] = center + ldir * half_l + odir * -half_w;  // D(half_l, -half_w)

        detect_box.x1 = bottom_quad[0](0);
        detect_box.y1 = bottom_quad[0](1);
        detect_box.x2 = bottom_quad[1](0);
        detect_box.y2 = bottom_quad[1](1);
        detect_box.x3 = bottom_quad[2](0);
        detect_box.y3 = bottom_quad[2](1);
        detect_box.x4 = bottom_quad[3](0);
        detect_box.y4 = bottom_quad[3](1);
        detect_BBoxs.push_back(detect_box);
    }
}

cv::Mat point_cloud_BEV::point_to_rgbimage(const pcl::PointCloud<PointType>::Ptr &cloud_in, double scale, position target_pos, angle target_angle)
{
    std::vector<box> BBoxs;   // 传入2D的目标框
    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>); // 建立一个点云指针
    transform_cloud(cloud_in, transformed_cloud, target_pos, target_angle);
    cv::Mat BEV;
    
    return BEV;
}

point_cloud_BEV::point_cloud_BEV()
{
    image_transport::ImageTransport it(nh_);

    obj_pub = it.advertise("image", 100);

    lidar_sub = nh_.subscribe("/livox/lidar", 100, &point_cloud_BEV::cloudCallback, this); // image_raw

    subGPS = nh_.subscribe<nav_msgs::Odometry>(gpsTopic, 200, &point_cloud_BEV::gpsHandler, this, ros::TransportHints().tcpNoDelay());

    subcontrol = nh_.subscribe<std_msgs::Float32MultiArray>(controlTopic, 200, &point_cloud_BEV::controlHandler, this, ros::TransportHints().tcpNoDelay());

    subDetect = nh_.subscribe<autoware_msgs::DetectedObjectArray>(detectTopic, 200, &point_cloud_BEV::detectHandler, this, ros::TransportHints().tcpNoDelay());

    publidarcloud = nh_.advertise<sensor_msgs::PointCloud2>("pcl_cloud", 100000);
}
