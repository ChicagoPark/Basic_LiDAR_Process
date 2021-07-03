#include "youngil.h"

//ros::Publisher pub;
ros::Publisher pub1;
ros::Publisher pub2;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane_main(new pcl::PointCloud<pcl::PointXYZI>);

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& inputcloud)
{
    //pcl을 담아줄 클라우드 포인터를 만들어준다.
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obst(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);

    //sensor_msgs::PointCloud2 cloud_output_obst;
    //sensor_msgs::PointCloud2 cloud_output_plane;

    pcl::fromROSMsg(*inputcloud, *cloud);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.1);

    //const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_Ptr(cloud);
    //pcl::PCLPointCloud2 * cloud_center(new pcl::PCLPointCloud2);
    //pcl::toPCLPointCloud2(*cloud, *cloud_center);
    //pcl::PCLPointCloud2ConstPtr cloud_Ptr(cloud_center);
    //pcl::PCLPointCloud2ConstPtr cloud_Ptr(cloud_center);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)            // 0일때, 우리는 우리 데이터에 적합한 모델을 찾지 못했다는 것을 의미한다.
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    for(int index : inliers->indices)       //inliers가 가지는 모든 인덱스 값에 해당하는 cloud값을 planeCloud에 넣어준다.
    {
        cloud_plane->points.push_back(cloud->points[index]);
    }

    if(cloud_plane->points.size() >= 2000)
    {
        //pcl::concatenatePointCloud(*cloud_plane, *cloud_ns2_pc, *merge_cloud_pc1);

        *cloud_plane_main = *cloud_plane_main + *cloud_plane;

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud);       //이 reference cloud 에서 inliers에 해당하는 모든 포인트가 사라져서 obstCloud 만 남게된다.
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_obst);
    }

    /* original
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);       //이 reference cloud 에서 inliers에 해당하는 모든 포인트가 사라져서 obstCloud 만 남게된다.
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_obst);
    */

    //pcl::PointCloud 클래스를 pcl::PCLPointCloud2 로 변환 한다.
    //pcl::PCLPointCloud2 * cloud_p1(new pcl::PCLPointCloud2);
    //pcl::PCLPointCloud2 * cloud_p2(new pcl::PCLPointCloud2);

    //pcl::toPCLPointCloud2(*cloud_obst, *cloud_p1);
    //pcl::toPCLPointCloud2(*cloud_plane, *cloud_p2);

    //pcl_conversions::fromPCL(*cloud_p1, cloud_output_obst);
    //pcl_conversions::fromPCL(*cloud_p2, cloud_output_plane);

    // 아웃풋을 위한 sensor_msgs::PointCloud2 선언
    sensor_msgs::PointCloud2 cloud_obst1;
    sensor_msgs::PointCloud2 cloud_plane2;

    //toPCL을 하여 fromPCL을 사용하기위한 선언
    pcl::PCLPointCloud2 * cloud_o1(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2 * cloud_p2(new pcl::PCLPointCloud2);

    pcl::toPCLPointCloud2(*cloud_obst, *cloud_o1);
    pcl::toPCLPointCloud2(*cloud_plane_main, *cloud_p2);
    pcl_conversions::fromPCL(*cloud_o1, cloud_obst1);
    pcl_conversions::fromPCL(*cloud_p2, cloud_plane2);
    cloud_obst1.header.frame_id = "velodyne";     //헤더에 대한 설정을 해주지않으면 rviz에 띄워지지 않는다.
    cloud_plane2.header.frame_id = "velodyne";    //헤더에 대한 설정을 해주지않으면 rviz에 띄워지지 않는다.
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
