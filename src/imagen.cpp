#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
  int i = 8;
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  image_transport::Publisher pub = it.advertise("camera/image", 1);
  ros::Rate loop_rate(1);

  while (nh.ok())
  {
    cv::Mat image = cv::imread("/home/alejandro/pipe_detection/src/pipe_detection/src/test1/out" + std::to_string(i) + ".jpg", CV_LOAD_IMAGE_COLOR);
    // Publico la imagen en un tópico para que el otro programa se suscriba e intente identificar la tuberia
    //cv::Mat image = cv::imread("/home/alejandro/pipe_detection/src/pipe_detection/src/out28.jpg", CV_LOAD_IMAGE_COLOR);
    //cv::imshow("hola", image);
    i = i + 1;
    if (i == 383)
      i = 8;
    cv::waitKey(3);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    image_transport::Publisher pub = it.advertise("camera/image", 1);
    ros::Rate loop_rate(5);

    while (nh.ok())
    {
      cv::Mat image = cv::imread("/home/alejandro/pipe_detection/src/pipe_detection/src/test1/out" + std::to_string(i) + ".jpg", CV_LOAD_IMAGE_COLOR);
      // Publico la imagen en un tópico para que el otro programa se suscriba e intente identificar la tuberia
      //cv::Mat image = cv::imread("/home/alejandro/pipe_detection/src/pipe_detection/src/out28.jpg", CV_LOAD_IMAGE_COLOR);
      //cv::imshow("hola", image);
      i = i + 1;
      if (i == 383)
        i = 8;
      cv::waitKey(3);
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}
