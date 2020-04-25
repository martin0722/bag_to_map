#ifndef __POINTMAP_LOADER_H__
#define __POINTMAP_LOADER_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

class PointMapLoader
{
    public:
        PointMapLoader();

    private:
        ros::NodeHandle mNodeHandle;
        ros::Publisher mPubMap;
        std::string mMapFileDirectory;
        pcl::PointCloud<pcl::PointXYZI> mMap;
};

#endif // __POINTMAP_LOADER_H__
