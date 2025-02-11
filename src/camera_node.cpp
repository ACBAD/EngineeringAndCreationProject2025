#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera", 1);
  ROS_INFO("Init ok");
  cv::VideoCapture cap(1); // 打开默认摄像头
  if (!cap.isOpened()) {
    ROS_ERROR("Cannot open camera");
    return -1;
  }
  ROS_INFO("Camera open");
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(30);
  ROS_INFO("Start capture");
  while (nh.ok()) {
    cap >> frame;
    if (!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      ros::spinOnce();
    }
    loop_rate.sleep();
  }
  return 0;
}
