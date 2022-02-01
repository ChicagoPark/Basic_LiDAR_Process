#include "youngil.h"

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI> ());

void cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZI>* source_cloud = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::PointCloud<pcl::PointXYZI>* transformed_cloud = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::fromROSMsg (*cloud_msg, *source_cloud);

Eigen::Translation3f init_translation(-0.0468, -0.266, 0.0);
Eigen::AngleAxisf init_rotation_x( 0.0, Eigen::Vector3f::UnitX());
Eigen::AngleAxisf init_rotation_y(0.0, Eigen::Vector3f::UnitY());
Eigen::AngleAxisf init_rotation_z(-0.35, Eigen::Vector3f::UnitZ());                   // unit is radian

Eigen::Matrix4f m = (init_translation * init_rotation_x*init_rotation_y* init_rotation_z).matrix();

  pcl::transformPointCloud (*source_cloud, *transformed_cloud, m);
  pcl::copyPointCloud(*transformed_cloud, *cloud1);

  delete source_cloud;
  delete transformed_cloud;

}

void cloud_cb1 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg2)
{
  pcl::fromROSMsg (*cloud_msg2, *cloud2);
  pcl::PointCloud<pcl::PointXYZI>* merge_cloud = new pcl::PointCloud<pcl::PointXYZI>;

  pcl::PCLPointCloud2* cloud1_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* cloud2_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* merge_cloud_pc = new pcl::PCLPointCloud2;
  pcl::toPCLPointCloud2(*cloud1, *cloud1_pc);
  pcl::toPCLPointCloud2(*cloud2, *cloud2_pc);
  pcl::toPCLPointCloud2(*merge_cloud, *merge_cloud_pc);

  pcl::concatenatePointCloud(*cloud1_pc, *cloud2_pc, *merge_cloud_pc);

  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(*merge_cloud_pc, output);
  pub.publish(output);
  delete merge_cloud;
  delete cloud1_pc;
  delete cloud2_pc;
  delete merge_cloud_pc;

}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "merge");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe ("/livox/lidar_1HDDH1200104181", 1, cloud_cb1);
  ros::Subscriber sub2 = nh.subscribe ("/livox/lidar_1HDDH3200106141", 1, cloud_cb2);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("livox_merge", 1);
 
  // Spin
  ros::spin ();
}
