#include <pcl/io/pcd_io.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <pcl/point_types.h> // 里面是pcl库点云的数据类型
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

class point_cloud_box
{
public:
    void createROSPubSub();

private:
    void cloudCallback(const sensor_msgs::PointCloud2::Ptr &cloud_msg);

    cv::Mat pointcloud_box(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, const int offset_x, const int offset_y, const float &pass_z);

    int scale_to_255(const float &H, const float &min, const float &max);

    void point_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const int min, const int max, std::string axis, bool setFilterLimitsNegative);

    // 图像旋转
    ///@ angle 要旋转的角度
    void Rotate(const cv::Mat &srcImage, cv::Mat &destImage, double angle);

    ros::NodeHandle nh_;
    image_transport::Publisher obj_pub;
    ros::Subscriber Img_sub;

    int Box_height = 30;
    int Box_width = 30;
    int BEV_height = 120;
    int BEV_width = 120;
    int angle_ = 270;
    int offset_x;
    int offset_y;
    int offset_z;
    int position[2];
};