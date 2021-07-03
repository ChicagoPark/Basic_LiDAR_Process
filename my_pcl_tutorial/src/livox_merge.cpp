#include "youngil.h"

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI> ());

void 
cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZI>* source_cloud = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::PointCloud<pcl::PointXYZI>* transformed_cloud = new pcl::PointCloud<pcl::PointXYZI>;
//  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
//  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::fromROSMsg (*cloud_msg, *source_cloud);


//  Eigen::Affine3f b = Eigen::Affine3f::Identity();
  // Define a translation of 2.5 meters on the x axis.
//  b.translation() << 0.2, 0.0, 0.0;
/*
  Eigen::Matrix4f m;
  m<<     1,   0,  0,  -0,	// x

          0,   1,  0,  -0.2,	// y

          0,   0,  1,  -0.0,	// z

          0,   0,  0,  1;

  double a, b, c;
  c = 0 * 3.14 / 180;  //roll (x축기준 회전) 좌우 기우리짐
  b = 0 * 3.14 / 180;  //pitch (y축기준 회전) 앞뒤 쏠림
  a = -20.8 * 3.14 / 180;  //yaw (xy좌표계 변환!!) 단순 회전  (- 우측 이동)

  m(0,0) = cos(a)*cos(b);
  m(0,1) = cos(a)*sin(b)*sin(c) - sin(a)*cos(c);
  m(0,2) = cos(a)*sin(b)*cos(c) + sin(a)*sin(c);

  m(1,0) = sin(a)*cos(b);
  m(1,1) = sin(a)*sin(b)*sin(c) + cos(a)*cos(c);
  m(1,2) = sin(a)*sin(b)*cos(c) - cos(a)*sin(c);

  m(2,0) = -sin(b);
  m(2,1) = cos(b)*sin(c);
  m(2,2) = cos(b)*cos(c);
*/
Eigen::Translation3f init_translation(-0.0468, -0.266, 0.0);
Eigen::AngleAxisf init_rotation_x( 0.0, Eigen::Vector3f::UnitX());
Eigen::AngleAxisf init_rotation_y(0.0, Eigen::Vector3f::UnitY());
Eigen::AngleAxisf init_rotation_z(-0.35, Eigen::Vector3f::UnitZ()); // 라디안으로 계산해준다.

Eigen::Matrix4f m = (init_translation * init_rotation_x*init_rotation_y* init_rotation_z).matrix();

  pcl::transformPointCloud (*source_cloud, *transformed_cloud, m);
  pcl::copyPointCloud(*transformed_cloud, *cloud1);

  delete source_cloud;
  delete transformed_cloud;

}

void 
cloud_cb1 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg2)
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