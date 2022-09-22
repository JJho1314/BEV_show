#include "test.hpp"
#include <fstream>
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
#include <cv_bridge/cv_bridge.h>
#include <string>

image_transport::Publisher obj_pub;

void readInToMatrix(std::fstream &in, std::string FilePath, std::vector<position> path)
{
    in.open(FilePath, ios::in); //打开一个file
    if (!in.is_open())
    {
        cout << "Can not find " << FilePath << endl;
        system("pause");
    }
    std::string buff;
    int i = 0; //行数i
    while (getline(in, buff))
    {
        std::vector<double> nums;
        position pos;
        // string->char *
        char *s_input = (char *)buff.c_str();
        const char *split = "，";
        // 以‘，’为分隔符拆分字符串
        char *p = strtok(s_input, split);
        double a;
        while (p != NULL)
        {
            // char * -> int
            a = atof(p);
            // cout << a << endl;
            nums.push_back(a);
            p = strtok(NULL, split);
        } // end while
        pos.x = nums[0];
        pos.y = nums[1];
        pos.z = nums[2];
        path.push_back(pos);
    } // end while
    in.close();
    std::cout << "get  data" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_box");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    obj_pub = it.advertise("image", 100);

    initSocketData();

    point_cloud_box pcb;

    // pcb.createROSPubSub();

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);

    sensor_msgs::PointCloud2 output;

    // pcb.createROSPubSub();

    pcl::PointCloud<pcl::PointXYZ> cloud;

    pcl::io::loadPCDFile("/home/jjho/workspace/data/outdoor-208.pcd", cloud); //修改自己pcd文件所在路径

    pcl::toROSMsg(cloud, output);

    output.header.frame_id = "livox_frame";

    std::vector<position> path;
    std::fstream in;
    readInToMatrix(in, "/home/jjho/workspace/data/picking_list.txt", path);
    std::cout << "save" << std::endl;
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        for (int i = 0; i < path.size(); i++)
        {
            float x = path[i].x;
            float y = path[i].y;
            cv::Mat BEV = pcb.pointcloud_box(cloud.makeShared(), 10, x, y);

            ROS_INFO("i = %d \n", i);

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", BEV).toImageMsg();

            obj_pub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return (0);
}