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
#include <rgbd_tools/state_filtering/ExtendedKalmanFilter.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ctime>
//#include <tf/transform_broadcaster.h>

using namespace std;
using namespace cv;
unsigned t0, t1, t2, t3;

// Function declarations for PCA
void drawAxis(Mat &, Point, Point, Scalar, const float);
double getOrientation(const vector<Point> &, vector<Point> &, Mat &);
double filterCentroid(const vector<Point> &, vector<Point> &, Mat &);
void drawAxis(Mat &img, Point p, Point q, Scalar colour, const float scale = 0.2)
{
  double angle;
  double hypotenuse;
  angle = atan2((double)p.y - q.y, (double)p.x - q.x); // angle in radians
  hypotenuse = sqrt((double)(p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
  //    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
  //    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
  // Here we lengthen the arrow by a factor of scale
  q.x = (int)(p.x - scale * hypotenuse * cos(angle));
  q.y = (int)(p.y - scale * hypotenuse * sin(angle));
  line(img, p, q, colour, 1, CV_AA);
  // create the arrow hooks
  p.x = (int)(q.x + 9 * cos(angle + CV_PI / 4));
  p.y = (int)(q.y + 9 * sin(angle + CV_PI / 4));
  line(img, p, q, colour, 1, CV_AA);
  p.x = (int)(q.x + 9 * cos(angle - CV_PI / 4));
  p.y = (int)(q.y + 9 * sin(angle - CV_PI / 4));
  line(img, p, q, colour, 1, CV_AA);
}
double getOrientation(const vector<Point> &pts, vector<Point> &pipeCentroid, Mat &img)
{
  //Construct a buffer used by the pca analysis
  int sz = static_cast<int>(pts.size());
  Mat data_pts = Mat(sz, 2, CV_64FC1);
  for (int i = 0; i < data_pts.rows; ++i)
  {
    data_pts.at<double>(i, 0) = pts[i].x;
    data_pts.at<double>(i, 1) = pts[i].y;
  }
  //Perform PCA analysis
  PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
  //Store the center of the object
  Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                     static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
  //Store the eigenvalues and eigenvectors
  vector<Point2d> eigen_vecs(2);
  vector<double> eigen_val(2);
  for (int i = 0; i < 2; ++i)
  {
    eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                            pca_analysis.eigenvectors.at<double>(i, 1));
    eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
  }
  // Draw the principal components
  circle(img, cntr, 3, Scalar(255, 0, 255), 2);
  Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
  Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
  drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
  drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);
  double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
  std::cout << "Centroid coordinates x,y: " << cntr.x << "," << cntr.y << std::endl;
  std::cout << "P1 x,y: " << p1.x << "," << p1.y << std::endl;
  std::cout << "P2 x,y: " << p2.x << "," << p2.y << std::endl;
  // Add cntr point and two eigen_vecs and eigen_val (p1 and p2)
  pipeCentroid.insert(pipeCentroid.end(), cntr.begin(), cntr.end());
  // for (int i = 0; i < 2; ++i)
  // {
  //   pipeCentroid.insert(pipeCentroid.end(), eigen_vecs[i].begin(), eigen_vecs[i].end());
  //   pipeCentroid.insert(pipeCentroid.end(), eigen_val[i].begin(), eigen_val[i].end());
  // }
  return angle;
}
double filterCentroid(const vector<Point> &pipeCentroid, vector<Point> &filteredpipeCentroid, cv::Mat &filteredImg)
{
  Eigen::Matrix<float, 3,1> z;    // New observation
  z <<    pipeCentroid[0],
          pipeCentroid[1];
          0;                      // 666: Altitude!!!!!!!!!
  double rate=0.2;
  ekf.stepEKF(z,rate);
  Eigen::Matrix<float,3,1> filteredX = ekf.state();

  circle(filteredImg, filteredCntr, 3, Scalar(255, 0, 255), 2);
  // Point p1 = filteredCntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
  // Point p2 = filteredCntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
  drawAxis(filteredImg, filteredCntr, p1, Scalar(0, 255, 0), 1);
  drawAxis(filteredImg, filteredCntr, p2, Scalar(255, 255, 0), 5);
  double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
  std::cout << "Filtered centroid coordinates x,y: " << cntr.x << "," << cntr.y << std::endl;
  // std::cout << "Filtered P1 x,y: " << p1.x << "," << p1.y << std::endl;
  // std::cout << "Filtered P2 x,y: " << p2.x << "," << p2.y << std::endl;
  return angle;
}

class AxisEKF : public rgbd::ExtendedKalmanFilter<float, 3, 3>
{
protected:
  void updateJf(const double _incT)
  {
    mJf.setIdentity();
    //mJf.block<2, 2>(0, 2) = Eigen::Matrix<float, 2, 2>::Identity() * _incT;
  }
  void updateHZk()
  {
    float fx=535.4;
    float fy=539.2;

    mHZk << fx/mXfa(0) , 0 , -fx*mXfa(0)/(mXfa(3)^2)
            0 , fy/mXfa(1) , -fy*mXfa(1)/(mXfa(3)^2)
            0 , 0 , 1;
  }
  void updateJh()
  {
    mJh.setIdentity();
  }
}

////////////////////////////
cv::Mat src_gray;
cv::Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold = 30;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
std::string window_name = "Edge Map";

class ImageProcessor
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  g_pub_;
  // tf::TransformBroadcaster tf_br_;
public:
  ImageProcessor(ros::NodeHandle &n, AxisEKF &_ekf) : nh_(n),
                                       it_(nh_)
  {
    image_transport::Publisher im
        img_sub_ = it_.subscribe("/camera/image", 1, &ImageProcessor::image_callback, this);
    img_pub_ = it_.advertise("/output_image", 1);
    Ekf=_Ekf;
  }

  ~ImageProcessor() {}

  void image_callback(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
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
    t0 = clock();
    /// Create a matrix of the same type and size as src (for dst)
    dst.create(src.size(), src.type());

    cv::cvtColor(src, src_gray, CV_BGR2GRAY);

    /// Reduce noise with a kernel 3x3
    blur(src_gray, detected_edges, cv::Size(5, 5));

    // /// Canny detector
    // Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
    // dst = Scalar::all(0);
    cv::cvtColor(src, dst, CV_BGR2HSV);
    // src.copyTo( dst, detected_edges);
    std::vector<rgbd::ImageObject> objects;
    // BOViL::ColorClusterSpace *ccs = BOViL::CreateHSVCS_8c(255,255,255);
    rgbd::ColorClusterSpace *ccs = rgbd::createSingleClusteredSpace(
        90, 130,
        10, 70,
        100, 180,
        180, 255, 255,
        32);

    rgbd::ColorClustering<uchar>(dst.data,
                                 dst.cols,
                                 dst.rows,
                                 30000,
                                 objects,
                                 *ccs);

    cv::cvtColor(dst, dst, CV_HSV2BGR);
    cv::Mat display = src.clone();
    for (auto &ob : objects)
    {
      cv::Rect bb(
          ob.centroid().x - ob.width() / 2,
          ob.centroid().y - ob.height() / 2,
          ob.width(),
          ob.height());
      cv::rectangle(display, bb, cv::Scalar(0, 255, 0), 2);
    }
    imshow(window_name, dst);
    imshow(window_name + "_res", display);
    cv::waitKey(3);

    //Publish image
    cv_bridge::CvImage send(cv_ptr->header, cv_ptr->encoding, dst);
    img_pub_.publish(send.toImageMsg());
    t1 = clock();
    /////////// PCA
    t2 = clock();
    Mat gray;
    cvtColor(dst, gray, COLOR_BGR2GRAY);
    // Convert image to binary
    Mat bw;
    threshold(gray, bw, 50, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    // Find all the contours in the thresholded image
    vector<Vec4i> hierarchy;
    vector<vector<Point>> contours;
    findContours(bw, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    for (size_t i = 0; i < contours.size(); ++i)
    {
      // Calculate the area of each contour
      double area = contourArea(contours[i]);
      // Ignore contours that are too small or too large, MODIFIED AS PIPE IS A VERY LARGE OBJECT!!!
      if (area < 1e4 || 1e8 < area)
        continue;
      // Draw each contour only for visualisation purposes
      drawContours(src, contours, static_cast<int>(i), Scalar(0, 0, 255), 2, 8, hierarchy, 0);
      // Find the orientation of each shape
      vector<Point> centroid;
      getOrientation(contours[i], centroid, src);
      // Filter orientation of each shape wit EKF
      cv::Mat filtered_src = src.clone();
      filterCentroid(centroid, filtered_src);
    }
    imshow("output1", src);
    imshow("output2", gray);
    imshow("output3", bw);
    imshow("output4", filtered_src);
    t3 = clock();
    double time1 = (double(t1 - t0) / CLOCKS_PER_SEC);
    cout << "Execution Time Bovil: " << time1 << endl;
    double time2 = (double(t3 - t2) / CLOCKS_PER_SEC);
    cout << "Execution Time PCA: " << time2 << endl;
  }
  
  AxisEKF ekf;

};

int main(int argc, char **argv)
{
  // Initialize EKF
  Eigen::Matrix<float, 3, 3> mQ; // State covariance
  mQ.setIdentity();
  mQ.block<3, 3>(0, 0) *= 0.01;
  mQ.block<3, 3>(1, 1) *= 0.01;
  mQ.block<3, 3>(2, 2) *= 0.01;
  Eigen::Matrix<float, 3, 3> mR; // Observation covariance
  mR.setIdentity();
  Eigen::Matrix<float, 3, 1> x0;
  x0 << 0, 0, 1; // (x,y)
  // Create EKF
  AxisEKF Axis_ekf;
  Axis_ekf.setUpEKF(mQ, mR, x0);

  ros::init(argc, argv, "pipe_detection");
  ros::NodeHandle n("~");
  ImageProcessor im(n,Axis_ekf);
  ros::spin();
  return 0;
}