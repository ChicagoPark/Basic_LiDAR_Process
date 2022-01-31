#include "youngil.h"

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& inputcloud)
{
    // Creates pointers that will capture point cloud.
    pcl::PCLPointCloud2 * cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2 * cloud_filtered(new pcl::PCLPointCloud2);

    
    
    // Creates sensor_msgs for the point clouds of publisher
    sensor_msgs::PointCloud2 cloud_output;

    // Converts sensor_msgs type to PCLPointCloud2 type
    pcl_conversions::toPCL(*inputcloud, *cloud);
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;

    pcl::PCLPointCloud2ConstPtr cloud_Ptr(cloud);

    vg.setInputCloud(cloud_Ptr);
    vg.setLeafSize(0.13, 0.13, 0.13);
    vg.filter(*cloud_filtered);

    //Set the cloudRegion to cut unnecessary areas.
    pcl::PCLPointCloud2 * cloud_Region(new pcl::PCLPointCloud2);

    pcl::PCLPointCloud2ConstPtr cloud_Ptr1(cloud_filtered);

    pcl::CropBox<pcl::PCLPointCloud2> region(true);     // we're dealing with points inside that CropBox
    region.setMin(Eigen::Vector4f(-20, -10, -0.8,1));
    region.setMax(Eigen::Vector4f(20,10,1.2,1));
    region.setInputCloud(cloud_Ptr1);
    region.filter(*cloud_Region);                       // gets the remained point cloud

    pcl_conversions::fromPCL(*cloud_Region, cloud_output);

    //Publish the data
    pub.publish(cloud_output);
}

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "filtering");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/livox_merge", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output1", 1);

    ros::spin();

    return 0;
}
