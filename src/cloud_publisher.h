//
// Created by howstar on 18-6-6.
//

#ifndef PROJECT_CLOUDSHOW_H
#define PROJECT_CLOUDSHOW_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

namespace cloud_show{

    extern ros::Publisher pub_obs;
    extern ros::Publisher pub_curb;
    extern ros::Publisher pub_box;
    extern ros::Publisher pub;

    void init_pub();

    void all_publish(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obs,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ori);
}

#endif //PROJECT_CLOUDSHOW_H