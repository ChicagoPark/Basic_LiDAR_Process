#include "youngil.h"

typedef pcl::PointXYZ PointT;

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& inputcloud)
{
  //Convert ROSMsg(Sensor msg) to PointXYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*inputcloud, *cloud);

  //Declare a KD-tree for clustering.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.5); // 2cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (20000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;

  pcl::PointCloud<pcl::PointXYZI> TotalCloud;
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
        pcl::PointXYZ pt = cloud->points[*pit];
        pcl::PointXYZI pt2;
        pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
        pt2.intensity = (float)(j + 1);           // assign different intensity about different clusters

        TotalCloud.push_back(pt2);
    }
    j++;
  }

  // Convert To ROS data type
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "velodyne";
  pub.publish(output);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clustering");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("Obst1", 1, cloud_cb);

  pub = nh.advertise<sensor_msgs::PointCloud2>("clust1", 1);

  ros::spin();

  return 0;
}
