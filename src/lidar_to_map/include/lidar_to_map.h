#ifndef __LIDAR_TO_MAP_H__
#define __LIDAR_TO_MAP_H__

#include <Eigen/Dense>
#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <vector>

class LidarToMap
{
public:
    LidarToMap(const std::string & filename);
    ~LidarToMap(){ mBag.close(); }

protected:
    void CreateTf();
    void TransformCloud();
    void CreateMap(const std::string & outputFolder);
    void LookupTransform(
        const std_msgs::Header & thisHeader,
        std::shared_ptr<Eigen::Matrix4f> & eigenTransform);

private:
    rosbag::Bag mBag;
    tf::Transformer mTransformer;
    pcl::PointCloud<pcl::PointXYZI> mMap;
};

#endif // __LIDAR_TO_MAP_H__
