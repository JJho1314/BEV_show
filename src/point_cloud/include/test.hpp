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

struct position
{
    float x;
    float y;
    float z;
};

typedef struct
{
    int target_header;

    int target_id;
    int target_x;
    int target_y;
    int target_w;
    int target_h;
    int target_class;
    int target_prob;
    int target_num;
    float target_velocity;
} targetInfo;

typedef struct
{
    bool use_ssr;
    bool use_flip;
    bool use_detect;
    int contrast;
    int bright;
} controlData; //下发的command

int sendSocketData();

int getSocketData(targetInfo temp);

int initSocketData();

class point_cloud_box
{
public:
    void createROSPubSub();

    cv::Mat pointcloud_box(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, double scale, double offset_x, double offset_y);

private:
    void cloudCallback(const sensor_msgs::PointCloud2::Ptr &cloud_msg);

    int scale_to_255(const float &H, const float &min, const float &max);

    void point_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const double min, const double max, std::string axis, bool setFilterLimitsNegative);

    // 图像旋转
    ///@ angle 要旋转的角度
    void Rotate(const cv::Mat &srcImage, cv::Mat &destImage, double angle);

    ros::NodeHandle nh_;
    image_transport::Publisher obj_pub;
    ros::Subscriber Img_sub;

    float min_z_ = -2.0;
    float pass_z_ = 0.5;
    int Box_height = 9;
    int Box_width = 16;
    int BEV_height = 720;
    int BEV_width = 1280;
    int angle_ = 0;
    int offset_x_ = 0;
    int offset_y_ = 0;
    int offset_z_ = 0;
};