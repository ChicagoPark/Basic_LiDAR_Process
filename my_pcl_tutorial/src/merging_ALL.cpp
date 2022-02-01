#include "youngil.h"

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ns1 (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ns2 (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ns3 (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_livox_1 (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_livox_2 (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_livox_m (new pcl::PointCloud<pcl::PointXYZI> ());

int a, b, c, d, e;

void livox_translation_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg1)
{
    pcl::PointCloud<pcl::PointXYZI>* source_cloud = new pcl::PointCloud<pcl::PointXYZI>;
    pcl::PointCloud<pcl::PointXYZI>* transformed_cloud = new pcl::PointCloud<pcl::PointXYZI>;
    pcl::fromROSMsg (*cloud_msg1, *source_cloud);
    Eigen::Translation3f init_translation(-0.0468, -0.266, 0.0);
    Eigen::AngleAxisf init_rotation_x( 0.0, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(0.0, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(-0.35, Eigen::Vector3f::UnitZ()); // 라디안으로 계산해준다.

    Eigen::Matrix4f m = (init_translation * init_rotation_x*init_rotation_y* init_rotation_z).matrix();

    pcl::transformPointCloud (*source_cloud, *transformed_cloud, m);
    pcl::copyPointCloud(*transformed_cloud, *cloud_livox_1);
    

    delete source_cloud;
    delete transformed_cloud;
    
    a = 1;
}

void livox_merge_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg2)
{
  pcl::fromROSMsg (*cloud_msg2, *cloud_livox_2);

  pcl::PointCloud<pcl::PointXYZI>* merge_cloud = new pcl::PointCloud<pcl::PointXYZI>;

  pcl::PCLPointCloud2* cloud1_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* cloud2_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* merge_cloud_pc = new pcl::PCLPointCloud2;
  pcl::toPCLPointCloud2(*cloud_livox_1, *cloud1_pc);
  pcl::toPCLPointCloud2(*cloud_livox_2, *cloud2_pc);
  pcl::toPCLPointCloud2(*merge_cloud, *merge_cloud_pc);

  pcl::concatenatePointCloud(*cloud1_pc, *cloud2_pc, *merge_cloud_pc);
  pcl::fromPCLPointCloud2(*merge_cloud_pc, *cloud_livox_2);
  Eigen::Translation3f init_translation(0.0234, +0.133, 0.0);
  Eigen::AngleAxisf init_rotation_x( 0.0, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(0.0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(0.175, Eigen::Vector3f::UnitZ()); // 라디안으로 계산해준다.

  Eigen::Matrix4f m = (init_translation * init_rotation_x*init_rotation_y* init_rotation_z).matrix();

  pcl::PointCloud<pcl::PointXYZI>* transformed_cloud = new pcl::PointCloud<pcl::PointXYZI>;

  pcl::transformPointCloud (*cloud_livox_2, *transformed_cloud, m);
  pcl::copyPointCloud(*transformed_cloud, *cloud_livox_m);
  
  // for memory efficiency
  delete merge_cloud;
  delete cloud1_pc;
  delete cloud2_pc;
  delete merge_cloud_pc;
  b = 1;
}


void cloud_cb3 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg3)
{
    pcl::PointCloud<pcl::PointXYZI>* source_cloud = new pcl::PointCloud<pcl::PointXYZI>;
    pcl::PointCloud<pcl::PointXYZI>* transformed_cloud = new pcl::PointCloud<pcl::PointXYZI>;
    pcl::fromROSMsg (*cloud_msg3, *source_cloud);
    Eigen::Translation3f init_translation(-0.05, 0.52, -0.09);
    Eigen::AngleAxisf init_rotation_x( 0.0, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(0.0, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(0.0, Eigen::Vector3f::UnitZ()); // 라디안으로 계산해준다.

    Eigen::Matrix4f m = (init_translation * init_rotation_x*init_rotation_y* init_rotation_z).matrix();

    pcl::transformPointCloud (*source_cloud, *transformed_cloud, m);
    pcl::copyPointCloud(*transformed_cloud, *cloud_ns1);


    delete source_cloud;
    delete transformed_cloud;

    c = 1;
}

void cloud_cb4 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg4)
{
    pcl::PointCloud<pcl::PointXYZI>* source_cloud = new pcl::PointCloud<pcl::PointXYZI>;
    pcl::PointCloud<pcl::PointXYZI>* transformed_cloud = new pcl::PointCloud<pcl::PointXYZI>;
    pcl::fromROSMsg (*cloud_msg4, *source_cloud);
    Eigen::Translation3f init_translation(-0.05, -0.52, -0.09);
    Eigen::AngleAxisf init_rotation_x( 0.0, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(0.0, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(0.0, Eigen::Vector3f::UnitZ()); // unit is radian

    Eigen::Matrix4f m = (init_translation * init_rotation_x*init_rotation_y* init_rotation_z).matrix();

    pcl::transformPointCloud (*source_cloud, *transformed_cloud, m);
    pcl::copyPointCloud(*transformed_cloud, *cloud_ns3);

    delete source_cloud;
    delete transformed_cloud;

    d = 1;
}

void cloud_cb5 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg5)
{
  pcl::fromROSMsg (*cloud_msg5, *cloud_ns2);
  pcl::PCLPointCloud2* cloud_ns1_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* cloud_ns2_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* cloud_ns3_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* cloud_livox_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* merge_cloud_pc1 = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* merge_cloud_pc2 = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* merge_cloud_pc3 = new pcl::PCLPointCloud2;
  pcl::toPCLPointCloud2(*cloud_ns1, *cloud_ns1_pc);
  pcl::toPCLPointCloud2(*cloud_ns2, *cloud_ns2_pc);
  pcl::toPCLPointCloud2(*cloud_ns3, *cloud_ns3_pc);
  pcl::toPCLPointCloud2(*cloud_livox_m, *cloud_livox_pc);

  pcl::concatenatePointCloud(*cloud_ns1_pc, *cloud_ns2_pc, *merge_cloud_pc1);
  pcl::concatenatePointCloud(*cloud_ns3_pc, *merge_cloud_pc1, *merge_cloud_pc2);
  pcl::concatenatePointCloud(*cloud_livox_pc, *merge_cloud_pc2, *merge_cloud_pc3);


  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(*merge_cloud_pc3, output);
  pub.publish(output);
    
  //delete merge_cloud;
  delete cloud_ns1_pc;
  delete cloud_ns2_pc;
  delete cloud_ns3_pc;
  delete cloud_livox_pc;
  delete merge_cloud_pc1;
  delete merge_cloud_pc2;
  delete merge_cloud_pc3;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "merge");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe ("/livox/lidar_1HDDH3200106141", 1, livox_translation_cb);
  ros::Subscriber sub2 = nh.subscribe ("/livox/lidar_1HDDH1200104181", 1, livox_merge_cb);
  ros::Subscriber sub3 = nh.subscribe ("/ns1/velodyne_points", 1, cloud_cb3);
  ros::Subscriber sub4 = nh.subscribe ("/ns3/velodyne_points", 1, cloud_cb4);
  
  ros::Subscriber sub5 = nh.subscribe ("/ns2/velodyne_points", 1, cloud_cb5);
  

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("livox_merge", 1);
 
  // Spin
  ros::spin ();
}
