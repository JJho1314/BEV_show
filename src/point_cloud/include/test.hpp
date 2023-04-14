#include <pcl/io/pcd_io.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <pcl/point_types.h> // 里面是pcl库点云的数据类型
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float32MultiArray.h>
#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/DetectedObject.h"
#include <livox_ros_driver/CustomMsg.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<pcl::PointXYZINormal> PointCloudXYZI;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

struct position
{
    float x;
    float y;
    float z;
};

struct angle
{
    float yaw;
    float pitch;
    float roll;
};

struct box
{
    float x1;
    float y1;
    float x2;
    float y2;
    float x3;
    float y3;
    float x4;
    float y4;
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

class point_cloud_BEV
{
public:
    point_cloud_BEV();

    cv::Mat Point_cloud_BEV(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, double scale, double offset_x, double offset_y, double offset_z, std::vector<box> BBoxs);
    
    cv::Mat point_to_rgbimage(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_in, double scale, position target_pos, angle target_angle);

    void cloudCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg);
private:

    void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    
    void gpsHandler(const nav_msgs::Odometry::ConstPtr &gpsMsg);

    void controlHandler(const std_msgs::Float32MultiArray::ConstPtr &controlMsg);

    void detectHandler(const autoware_msgs::DetectedObjectArray::ConstPtr &input_detections);

    int scale_to_255(const float &H, const float &min, const float &max);

    void point_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const double min, const double max, std::string axis, bool setFilterLimitsNegative);

    void RGB_point_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const double min, const double max, std::string axis, bool setFilterLimitsNegative);

    void transform_cloud(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_in, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &transformed_cloud, const position target_pos, const angle target_angle);

    cv::Mat cloud_to_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, double scale);
    // 图像旋转
    ///@ angle 要旋转的角度
    void Rotate(const cv::Mat &srcImage, cv::Mat &destImage, double angle);

    ros::NodeHandle nh_;
    image_transport::Publisher obj_pub;
    ros::Subscriber lidar_sub;
    ros::Subscriber subGPS;
    ros::Subscriber subcontrol;
    ros::Subscriber subDetect;

    ros::Publisher publidarcloud;

    std::string gpsTopic = "odom";
    std::string controlTopic = "control_map";
    std::string detectTopic = "/detection/final_result/objects";

    std::vector<box> detect_BBoxs;

    float min_z_ = -10.0;
    float pass_z_ = 10.0;
    float scale_ = 10;
    int Box_height = 10;   // 场景长宽比例
    int Box_width = 16;    // 场景长宽比例
    int BEV_height = 720;  // 视频尺寸
    int BEV_width = 1280;  // 视频尺寸
    int angle_ = 0;
    int offset_x_ = 0;
    int offset_y_ = 0;
    int offset_z_ = 0;
};