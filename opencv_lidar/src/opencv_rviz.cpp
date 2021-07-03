#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

//rviz 에 띄우기 위한 sensor_msgs 헤더 추가
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_opencv");
  ros::NodeHandle nh;

  cout <<"OpenCV version : " << CV_VERSION << endl;
  cout <<"Major version : " << CV_MAJOR_VERSION << endl;

  sensor_msgs::Image opencv_sensor_image;
  cv_bridge::CvImage cv_bridge;
  ros::Publisher opencv_sensor_image_publish = nh.advertise<sensor_msgs::Image>("image", 100);

  std_msgs::Header header;  //sensor_msgs::Image 의 메시지에 포함되어있던 헤더를 채우기위해 선언
  ros::Rate loop_rate(60);  //loop 주기는 60hz

  ROS_INFO("opencv_rviz_node open");

  double width;
  double height;
  double exposure;
  double brightness;

  Mat img;                //행렬 선언

  VideoCapture vc(0);     //카메라 번호 선언 (0번으로 하면 0번 카메라가 뜨고, 1번으로 하면 두번 째로 연결한 카메라가 잡힌다.)

  width = vc.get(CAP_PROP_FRAME_WIDTH);
  height = vc.get(CAP_PROP_FRAME_HEIGHT);
  exposure = vc.get(CAP_PROP_EXPOSURE);
  brightness = vc.get(CAP_PROP_BRIGHTNESS);   //카메라 정보 얻기

  cout << "width : " << width << endl;
  cout << "height : " << height << endl;
  cout << "exposure : " << exposure << endl;
  cout << "brightness : " << brightness << endl;

  while(ros::ok()){
    vc >> img;      // vc의 카메라 이미지를 img 행렬에 대입

    flip(img, img, 1);  // 영상 이미지 좌우 반전

    //rviz 상에서 이미지를 보기위해 imshow 코드를 토픽을 퍼블리싱하는 코드로 수정해주도록 하겠다.
    //imshow("video", img); //영상 출력

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img).toImageMsg();   //msg를 sensor_msgs::ImagePtr msg 와 같이 포인터로 표현할 것￣
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "velodyne";

    opencv_sensor_image_publish.publish(msg);

    if(waitKey(10) == 27) break;    //esc 누르면 종료
    ros::spinOnce();    //spin의 경우 msg가 수신될 경우 콜백함수가 호출되게 된다.
    loop_rate.sleep();

  }
  return 0;
}
