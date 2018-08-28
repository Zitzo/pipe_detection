#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <rgbd_tools/segmentation/color_clustering/ColorClustering.h>
#include <rgbd_tools/segmentation/color_clustering/types/ColorSpaceHSV8.h>
#include <rgbd_tools/segmentation/color_clustering/types/ccsCreation.h>

using namespace cv;

/// Global variables

Mat src, src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold=30;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
std::string window_name = "Edge Map";

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, Size(5,5) );

  // /// Canny detector
  // Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  // dst = Scalar::all(0);
  cv::cvtColor(src,dst,CV_BGR2HSV);
  // src.copyTo( dst, detected_edges);
  std::vector<rgbd::ImageObject> objects;
  // BOViL::ColorClusterSpace *ccs = BOViL::CreateHSVCS_8c(255,255,255);
  rgbd::ColorClusterSpace *ccs = rgbd::createSingleClusteredSpace(
    0,180,
    30, 90,
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
 }


/** @function main */
int main( int argc, char** argv )
{
  /// Load an image
  src = imread("/home/alejandro/pipe_detection/src/pipe_detection/src/out28.jpg", CV_LOAD_IMAGE_COLOR);

  if( !src.data )
  { return -1; }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );

  /// Convert the image to grayscale
  cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Create a window
  namedWindow( window_name, CV_WINDOW_FREERATIO );
  namedWindow( window_name+"_res", CV_WINDOW_FREERATIO );

  /// Create a Trackbar for user to enter threshold
  createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

  /// Show the image
  CannyThreshold(0, 0);

  /// Wait until user exit program by pressing a key
  waitKey(0);

  return 0;
  }