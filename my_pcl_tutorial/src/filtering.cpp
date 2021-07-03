#include "youngil.h"

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& inputcloud)
{
    //pcl을 담아줄 클라우드 포인터를 만들어준다.
    pcl::PCLPointCloud2 * cloud(new pcl::PCLPointCloud2);   //PCLPointCloud2는 template을 취급하지 않는다.
    pcl::PCLPointCloud2 * cloud_filtered(new pcl::PCLPointCloud2);

    //이후 sensor_msgs로 변환하기위한 container 또한 만들어준다.
    sensor_msgs::PointCloud2 cloud_output;

    //sensor_msgs 타입을 pcl을 위한 타입으로 변환해준다.
    pcl_conversions::toPCL(*inputcloud, *cloud);
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;

    pcl::PCLPointCloud2ConstPtr cloud_Ptr(cloud);    //cloudPtr을 선언하여 변환 시키지 않을 cloud를 담아두고 이 후 처리속도를 돕는다.

    vg.setInputCloud(cloud_Ptr);        //voxel처리와 cropbox 처리를 하는데에 들어가는 인풋은 반드시 const pointer로 정의되어있어야한다.
    vg.setLeafSize(0.13, 0.13, 0.13);
    //vg.setLeafSize(0.09, 0.09, 0.09);
    vg.filter(*cloud_filtered); //.filter 의 경우 괄호에 레퍼런스 변수의 형태를 요구하므로 *를 붙여준다.

    //우리가 불필요한 지역을 자르기 위해 cloudRegion을 정해준다.
    pcl::PCLPointCloud2 * cloud_Region(new pcl::PCLPointCloud2);

    pcl::PCLPointCloud2ConstPtr cloud_Ptr1(cloud_filtered);

    pcl::CropBox<pcl::PCLPointCloud2> region(true);     // we're dealing with points inside that CropBox
    region.setMin(Eigen::Vector4f(-20, -10, -0.8,1));
    region.setMax(Eigen::Vector4f(20,10,1.2,1));
    region.setInputCloud(cloud_Ptr1);
    region.filter(*cloud_Region);           //우리가 원하는 부분을 자른 포인트클라우드를 받아왔다.

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
    ros::Subscriber sub = nh.subscribe("/livox_merge", 1, cloud_cb);   //"points_calibrated"라는 토픽을 사용하며, que size는 real time 처리를 위해 1로 설정해준다.

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output1", 1);     //내어 보내주는 토픽의 이름은 "output1" 으로 설정을 해주는 것을 확인할 수 있다.

    ros::spin();

    return 0;
}
