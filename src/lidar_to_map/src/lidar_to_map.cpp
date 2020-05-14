#include <lidar_to_map.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <memory>
#include <rosbag/view.h>
#include <tf/transform_datatypes.h>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

LidarToMap::LidarToMap(const std::string & filename)
    : mBag()
    , mTransformer(true, ros::Duration(10000.0))
    , mMap()
{
    mBag.open(filename);
    CreateTf();
    TransformCloud();
    const std::string outputFolder = "out";
    if (fs::exists(outputFolder.c_str()) &&
        fs::is_directory(outputFolder.c_str()))
        fs::remove_all(outputFolder.c_str());
    fs::create_directory(outputFolder.c_str());
    CreateMap(outputFolder);
}

void LidarToMap::CreateTf()
{
    std::vector<std::string> topics;
    topics.push_back(std::string("/tf_static"));
    std::vector<geometry_msgs::TransformStamped> tfStatic;
    for(rosbag::MessageInstance const m : rosbag::View(
        mBag, rosbag::TopicQuery(topics)))
    {
        auto msg = m.instantiate<tf2_msgs::TFMessage>();
        if (msg != nullptr)
        {
            for (const auto & tf : msg->transforms)
                tfStatic.push_back(tf);
        }
    }

    topics.clear();
    topics.push_back(std::string("/tf"));
    for(rosbag::MessageInstance const m : rosbag::View(
        mBag, rosbag::TopicQuery(topics)))
    {
        auto msg = m.instantiate<tf2_msgs::TFMessage>();
        if (msg != nullptr)
        {
            tf::StampedTransform tftf;
            for (const auto & tf : msg->transforms)
            {
                tf::transformStampedMsgToTF(tf, tftf);
                mTransformer.setTransform(tftf);
                for (auto tf_static : tfStatic)
                {
                    tf_static.header.stamp = tf.header.stamp;
                    tf::transformStampedMsgToTF(tf_static, tftf);
                    mTransformer.setTransform(tftf);
                }
            }
        }
    }
}

void LidarToMap::TransformCloud()
{
    std::vector<std::string> topics;
    topics.push_back(std::string("/LidarFront/Raw"));
    for(rosbag::MessageInstance const m : rosbag::View(
        mBag, rosbag::TopicQuery(topics)))
    {
        auto msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (msg != nullptr)
        {
            pcl::PointCloud<pcl::PointXYZI> pclCloud;
            pcl::fromROSMsg(*msg, pclCloud);
            if (!pclCloud.empty())
            {
                std::shared_ptr<Eigen::Matrix4f> transform;
                LookupTransform(msg->header, transform);
                if (transform)
                {
                    pcl::PointCloud<pcl::PointXYZI> pclCloudT;
                    std::cout << "transform cloud at: " <<
                        msg->header.stamp << std::endl;
                    pcl::transformPointCloud(pclCloud, pclCloudT, *transform);
                    mMap += pclCloudT;
                }
            }
        }
    }
}

void LidarToMap::CreateMap(const std::string & outputFolder)
{
    pcl::PCDWriter writer;
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D(mMap, minPt, maxPt);
    const auto subMapSize = 100.0;
    auto ix = 0u;
    std::cout << "x:" << minPt.x << "~" << maxPt.x << std::endl;
    std::cout << "y:" << minPt.y << "~" << maxPt.y << std::endl;
    for (auto x{ minPt.x }; x < maxPt.x; x += subMapSize)
    {
        pcl::PassThrough<pcl::PointXYZI> filterX;
        filterX.setInputCloud(mMap.makeShared());
        filterX.setFilterFieldName("x");
        filterX.setFilterLimits(x, x + subMapSize);
        pcl::PointCloud<pcl::PointXYZI> cloudX;
        filterX.filter(cloudX);
        auto iy = 0u;
        for (auto y{ minPt.y }; y < maxPt.y; y += subMapSize)
        {
            pcl::PassThrough<pcl::PointXYZI> filterY;
            filterY.setInputCloud(cloudX.makeShared());
            filterY.setFilterFieldName("y");
            filterY.setFilterLimits(y, y + subMapSize);
            pcl::PointCloud<pcl::PointXYZI> cloud;
            filterY.filter(cloud);
            const auto filename = outputFolder + "/sub_map_" +
                std::to_string(ix) + "_" + std::to_string(iy ++) + ".pcd";
            if (!cloud.empty())
            {
                std::cout << "write ... " << filename << std::endl;
                writer.writeBinaryCompressed(filename, cloud);
            }
            else
                std::cout << "write ... " << filename <<
                    " ... empty" << std::endl;
        }
        ix ++;
    }
}

void LidarToMap::LookupTransform(
    const std_msgs::Header & thisHeader,
    std::shared_ptr<Eigen::Matrix4f> & eigenTransform)
{
    try
    {
        tf::StampedTransform tf;
        mTransformer.lookupTransform(
            "/map", thisHeader.frame_id, thisHeader.stamp, tf);
        eigenTransform = std::make_shared<Eigen::Matrix4f>();
        pcl_ros::transformAsMatrix(tf, *eigenTransform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }
}

