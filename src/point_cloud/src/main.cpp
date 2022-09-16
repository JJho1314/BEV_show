#include "test.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_box");
    initSocketData();

    point_cloud_box pcb;

    pcb.createROSPubSub();

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return (0);
}