#include "youngil.h"

//ros::Publisher pub;
ros::Publisher pub1;
ros::Publisher pub2;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& inputcloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obst(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    
    pcl::fromROSMsg(*inputcloud, *cloud);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.08);
    
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    for(int index : inliers->indices)
    {
        cloud_plane->points.push_back(cloud->points[index]);
    }


    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_obst);

    // Define sensor_msgs::PointCloud2 for publishing
    sensor_msgs::PointCloud2 cloud_obst1;
    sensor_msgs::PointCloud2 cloud_plane2;

    pcl::PCLPointCloud2 * cloud_o1(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2 * cloud_p2(new pcl::PCLPointCloud2);

    pcl::toPCLPointCloud2(*cloud_obst, *cloud_o1);
    pcl::toPCLPointCloud2(*cloud_plane, *cloud_p2);
    pcl_conversions::fromPCL(*cloud_o1, cloud_obst1);
    pcl_conversions::fromPCL(*cloud_p2, cloud_plane2);
    cloud_obst1.header.frame_id = "velodyne";           // hearder setting is matter from rviz to visualize
    cloud_plane2.header.frame_id = "velodyne";
    pub1.publish(cloud_obst1);
    pub2.publish(cloud_plane2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "segmentation");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("output1", 1, cloud_cb);

    pub1 = nh.advertise<sensor_msgs::PointCloud2>("Obst1", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("Plane2", 1);


    ros::spin();

    return 0;
}
