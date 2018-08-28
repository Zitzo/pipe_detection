#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <rgbd_tools/segmentation/color_clustering/ColorClustering.h>
#include <rgbd_tools/segmentation/color_clustering/types/ColorSpaceHSV8.h>
#include <rgbd_tools/segmentation/color_clustering/types/ccsCreation.h>
//#include <tf/transform_broadcaster.h>


cv::Mat  src_gray;
cv::Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold=30;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
std::string window_name = "Edge Map";

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
    
   cv::Mat src = cv_ptr->image;
   //cv::Mat img = cv::imread("/home/alejandro/pipe_detection/src/pipe_detection/src/out28.jpg", CV_LOAD_IMAGE_COLOR);
  // cv::waitkey(30);
  // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
   
   

    // Do something with img and store the result in send
  ROS_INFO("Callback");

  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );
    
  cv::cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, cv::Size(5,5) );

  // /// Canny detector
  // Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  // dst = Scalar::all(0);
  cv::cvtColor(src,dst,CV_BGR2HSV);
  // src.copyTo( dst, detected_edges);
  std::vector<rgbd::ImageObject> objects;
  // BOViL::ColorClusterSpace *ccs = BOViL::CreateHSVCS_8c(255,255,255);
  rgbd::ColorClusterSpace *ccs = rgbd::createSingleClusteredSpace(
    90,130,
    10, 70,
    100,180,
    180,255,255,
    32
  );

  rgbd::ColorClustering<uchar>(dst.data,
                                dst.cols,
                                dst.rows,
                                30000,
                                objects,
                                *ccs);

  cv::cvtColor(dst,dst,CV_HSV2BGR);
  cv::Mat display = src.clone();
  for(auto &ob: objects){
    cv::Rect bb(
      ob.centroid().x - ob.width()/2,
      ob.centroid().y - ob.height()/2,
      ob.width(),
      ob.height()
    );
    cv::rectangle(display, bb, cv::Scalar(0,255,0),2);
  }
  imshow( window_name, dst );
  imshow( window_name+"_res", display );
  cv::waitKey(3);
 
  //Publish image
    cv_bridge::CvImage send (cv_ptr->header, cv_ptr->encoding, dst);
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