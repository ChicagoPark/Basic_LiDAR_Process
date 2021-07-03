/*
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
*/

#include "youngil.h"

//바운딩 박스를 만들어주기위한 구조체 추가

struct Box
{
  float x_min;
  float y_min;
  float z_min;
  float x_max;
  float y_max;
  float z_max;
};

typedef pcl::PointXYZ PointT;

//clusters 를 다른 색으로 담고있는 clusters를 퍼블리싱하기 위한 pub1
ros::Publisher pub1;
//bounding box를 visualization 하기위한 퍼블리셔
ros::Publisher pub2;
//This publisher is for multi ransac. 
ros::Publisher pub_ransac;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& inputcloud)
{
  //ROS message 변환
  //왜 PointXYZ로 선언을 해줄까? => 이후 clusters 에 대해 다른 색으로 표현해주기 위해서,clustering 이후 intensity value를 정해주기 위해서이다.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*inputcloud, *cloud);

  //visualizing에 필요한 marker 선언
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  //Clustering 을 위한 KD-tree를 선언해준다.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  //각각의 clusters를 받아줄 vector를 만들어준다.
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.15); // 2cm

  ec.setMinClusterSize (25);
  ec.setMaxClusterSize (10000);
  //ec.setMinClusterSize (30);
  //ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;

  pcl::PointCloud<pcl::PointXYZI> TotalCloud;
  int j = 0;
  //각 cluster들을 각각 고려하여 intensity를 표시해주기위한 작업
  //아래 for문은 각각의 cluster에 대해 접근해준다.
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZI> eachCloud;
    float a = cluster_indices.size();
    //아래 for문은 점하나하나에 접근
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
        //개별 클라우드를 받기위한 포인트 클라우드 선언
        pcl::PointXYZ pt = cloud->points[*pit];
        pcl::PointXYZI pt2;
        pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
        pt2.intensity = (float)(j + 1);   //다른 clusters에 대해 다른 intensity를 넣어주기위해
        eachCloud.push_back(pt2);
        TotalCloud.push_back(pt2);
     }
        //minPoint와 maxPoint 받아오기
        pcl::PointXYZI minPoint, maxPoint;
        pcl::getMinMax3D(eachCloud, minPoint, maxPoint);
        Box box;
        box.x_min = minPoint.x;
        box.y_min = minPoint.y;
        box.z_min = minPoint.z;
        box.x_max = maxPoint.x;
        box.y_max = maxPoint.y;
        box.z_max = maxPoint.z;

        //각 축에 대한 x, y, z 축 사이즈 고려하기위한 size 변수를 선언한다.
        float xsize = std::abs(box.x_max - box.x_min);
        float ysize = std::abs(box.y_max - box.y_min);
        float zsize = std::abs(box.z_max - box.z_min);
        float volume = xsize * ysize * zsize;

        float loc_x = (box.x_max - box.x_min)/2 + box.x_min;
        float loc_y = (box.y_max - box.y_min)/2 + box.y_min;
        float loc_z = (box.z_max - box.z_min)/2 + box.z_min;

        //if (0.5 < xsize && xsize < 10 && 1.2 < ysize && ysize < 4 && 0.15<zsize && zsize<3 && volume > 1.5)  this conditional sentense is for real vehicles
        if (0.05 < xsize && xsize < 0.6 && 0.05 < ysize && ysize < 0.6 && 0.4<zsize && zsize<3)// && volume > 0.2)
        {
            marker.header.frame_id = "velodyne";
            marker.header.stamp = ros::Time();
            marker.ns = a;
            marker.id = j;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = (box.x_min+box.x_max)/2;
            marker.pose.position.y = (box.y_min+box.y_max)/2;
            marker.pose.position.z = (box.z_min+box.z_max)/2;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = xsize;
            marker.scale.y = ysize;
            marker.scale.z = zsize;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.lifetime = ros::Duration(0.1);
            marker_array.markers.push_back(marker);
          }
        j++;
        float distance = std::sqrt(loc_x * loc_x + loc_y *loc_y);
        distance = std::sqrt(distance * distance + loc_z * loc_z);
        std::cout << "DISTANCE FROM BOX : " << distance << std::endl;

  }
  // Convert To ROS data type
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "velodyne";


  pub1.publish(output);
  pub2.publish(marker_array);
  //pub_ransac.publish(output);

  

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clustering");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("Obst1", 1, cloud_cb);

  pub1 = nh.advertise<sensor_msgs::PointCloud2>("clust1", 1);
  pub2 = nh.advertise<visualization_msgs::MarkerArray>("/BoundingBox", 1);

  //pub_ransac = nh.advertise<sensor_msgs::PointCloud2>("output1", 1);

  ros::spin();

  return 0;
}
