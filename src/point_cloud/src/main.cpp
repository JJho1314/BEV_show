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
#include <sstream>


#define PI 3.1415926

image_transport::Publisher obj_pub;

void readInToMatrix(std::fstream &in, std::string FilePath, std::vector<position> &path)
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
        // char *s_input = (char *)buff.c_str();
        // const char *split = "，";
        // // 以‘，’为分隔符拆分字符串
        // char *p = strtok(s_input, split);
        // double a;
        // while (p != NULL)
        // {
        //     // char * -> int
        //     a = atof(p);
        //     nums.push_back(a);
        //     p = strtok(NULL, split);
        // } // end while

        for (int i = 0; i < buff.size(); ++i)
        {
            if (buff[i] == ',')
            {
                buff[i] = ' ';
            }
        }
        std::istringstream out(buff);
        std::string str;
        while (out >> str)
        {
            nums.push_back(std::stof(str));
        }

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
    ros::init(argc, argv, "point_cloud_BEV");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    initSocketData();
    point_cloud_BEV pcb;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>); // 建立一个点云指针

    // std::vector<box> BBoxs;   // 传入2D的目标框
    // position target_pos;     //  鼠标传入的目标位置
    // angle target_angle;      // 鼠标传入的目标姿态角度

    // pcb.createROSPubSub();
    
    ros::spin();

    // while (ros::ok())
    // {
    //     ROS_INFO("ok");

    //     target_pos.x = 0;
    //     target_pos.y = 0;
    //     target_pos.z = 1;
    //     target_angle.yaw = PI;
        
    //     // pcb.transform_cloud(cloud.makeShared(), transformed_cloud, target_angle, target_pos);
    //     // cv::Mat BEV = pcb.Point_cloud_BEV(transformed_cloud, 8, 0, 0, 0, BBoxs);

    //     // cv::Mat BEV = pcb.point_to_rgbimage(cloud.makeShared(), 8, target_pos, target_angle);

    //     // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", BEV).toImageMsg();

    //     // obj_pub.publish(msg);
    //     ros::Duration(0.2).sleep();
        

    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }


    return (0);
}