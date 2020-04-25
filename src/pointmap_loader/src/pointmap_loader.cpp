#include <experimental/filesystem>
#include <pcl/common/common.h>
#include <pcl_ros/io/pcd_io.h>
#include <pointmap_loader.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace fs = std::experimental::filesystem;

PointMapLoader::PointMapLoader()
    : mNodeHandle()
    , mPubMap()
    , mMapFileDirectory("/home/mxwu/tf_lidar/devel/lib/lidar_to_map/out")
    , mMap()
{
    mPubMap = mNodeHandle.advertise<sensor_msgs::PointCloud2>(
        "/points_map", 1u, true);

    pcl::PCDReader reader;
    for (auto & inputFile : fs::directory_iterator(mMapFileDirectory))
    {
        PCL_WARN("processing %s\n",
            fs::path(inputFile).filename().string().c_str());
        pcl::PointCloud<pcl::PointXYZI> cloud;
        if (reader.read(fs::path(inputFile).string(), cloud) == -1)
            PCL_ERROR("Couldn't read file %s\n",
                fs::path(inputFile).filename().string().c_str());
        else
            mMap += cloud;
    }
    std::cout << "map size: " << mMap.size() << std::endl;

    sensor_msgs::PointCloud2 mapToPub;
    pcl::toROSMsg(mMap, mapToPub);
    mapToPub.header.frame_id = "/map";
    mPubMap.publish(mapToPub);

    static tf2_ros::StaticTransformBroadcaster staticBroadcaster;
    geometry_msgs::TransformStamped staticTransformStamped;
    staticTransformStamped.header.stamp = ros::Time::now();
    staticTransformStamped.header.frame_id = "/map";
    staticTransformStamped.child_frame_id = "/base_link";
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D(mMap, minPt, maxPt);
    staticTransformStamped.transform.translation.x =
        0.5f * (minPt.x + maxPt.x); 
    staticTransformStamped.transform.translation.y =
        0.5f * (minPt.y + maxPt.y); 
    staticTransformStamped.transform.translation.z = 0.0f;
    tf2::Quaternion quat;
    quat.setRPY(0.0f, 0.0f, 0.0f);
    staticTransformStamped.transform.rotation.x = quat.x();
    staticTransformStamped.transform.rotation.y = quat.y();
    staticTransformStamped.transform.rotation.z = quat.z();
    staticTransformStamped.transform.rotation.w = quat.w();
    staticBroadcaster.sendTransform(staticTransformStamped);

    ros::spin();
}
