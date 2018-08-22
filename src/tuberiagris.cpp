#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
//#include <tf/transform_broadcaster.h>

class ImageProcessor
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher img_pub_;
 // tf::TransformBroadcaster tf_br_;
public:
  ImageProcessor(ros::NodeHandle& n):
    nh_(n),
    it_(nh_)
  {
    img_sub_ = it_.subscribe("/camera/image", 1, &ImageProcessor::image_callback, this);
    img_pub_ = it_.advertise("/output_image", 1);
    
  }

  ~ImageProcessor() {}

   void image_callback(const sensor_msgs::ImageConstPtr& msg)
   {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
    }
    
   cv::Mat img = cv_ptr->image;
   //cv::Mat img = cv::imread("/home/alejandro/pipe_detection/src/pipe_detection/src/out28.jpg", CV_LOAD_IMAGE_COLOR);
  // cv::waitkey(30);
  // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
   
   

    // Do something with img and store the result in send
  ROS_INFO("Callback");

    
    
    cv::Mat hsv;
    cvtColor(img,hsv, CV_BGR2HSV);
    
    cv::Mat img_inrange;
    cv::inRange(hsv, cv::Scalar(90,10,100) , cv::Scalar(130,70,180),img_inrange);
   
    cv::Mat img_hsv;
    cvtColor(hsv,img_hsv, CV_HSV2BGR);

    cv::Mat img_inrange_bgr;
    cv::cvtColor(img_inrange,img_inrange_bgr,  CV_GRAY2BGR);
   
    cv::Mat img_eroded;
    cv::Mat element= cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10));
    cv::erode(img_inrange,img_eroded,element);

    cv::Mat img_eroded_bgr;
    cv::cvtColor(img_eroded,img_eroded_bgr,  CV_GRAY2BGR);

    cv::Mat img_dilated;
    cv::dilate(img_eroded,img_dilated,element);
    
    cv::Mat img_dilated_bgr;
    cv::cvtColor(img_dilated,img_dilated_bgr,  CV_GRAY2BGR);

    std::vector<std::vector<cv:: Point> > contours, contours_poly;
    cv::findContours(img_dilated,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for(int i=0;i<contours.size();i++)
     {    
       cv::drawContours(img_hsv,contours,i,cv::Scalar(0,0,0),3);
         
       std::vector<cv:: Point> approx;
       cv::approxPolyDP(contours[i],approx,2,true);
       contours_poly.push_back(approx); 
       
       //ROS_INFO_STREAM("Contour size:" << contours[i].size() <<", approx"<< approx.size()); 
      
        
	

     
}
   
 
  //Publish image
    cv_bridge::CvImage send (cv_ptr->header, cv_ptr->encoding,img_inrange_bgr);
    img_pub_.publish(send.toImageMsg());

  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pipe_detection");
  ros::NodeHandle n("~");
  ImageProcessor im(n);
  ros::spin();
  return 0;
}
