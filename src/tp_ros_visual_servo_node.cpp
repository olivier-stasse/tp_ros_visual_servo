#include <iostream>
#include <unistd.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  int iLowH_,iHighH_;
  int iLowS_,iHighS_;
  int iLowV_,iHighV_;

public:
  ImageConverter()
    : it_(),iLowH_(15), iHighH_(83),
      iLowS_(190), iHighS_(226),
      iLowV_(0), iHighV_(255)
  {
    // Creates Node and subscribe to input video feed
    /*
     A COMPLETER
    */
    // Window to show the image
    cv::namedWindow(OPENCV_WINDOW);
    // Window named "Control" to display sliders
    cv::namedWindow("Control", WINDOW_AUTOSIZE);

    // Create trackbars in "Control" window
    cv::createTrackbar("LowH", "Control", &iLowH_, 179); //Hue (0 - 179)
    cv::createTrackbar("HighH", "Control", &iHighH_, 179);

    cv::createTrackbar("LowS", "Control", &iLowS_, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Control", &iHighS_, 255);
    cv::createTrackbar("LowV", "Control", &iLowV_, 255); //Value (0 - 255)
    cv::createTrackbar("HighV", "Control", &iHighV_, 255);

    // Create publisher to send to control
    /* A COMPLETER */
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    Mat imgHSV;
    cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV);

    Mat imgThresholded;
    inRange(imgHSV, Scalar(iLowH_, iLowS_, iLowV_),
	    Scalar(iHighH_, iHighS_, iHighV_), imgThresholded); //Threshold the image

    imshow("Thresholded Image", imgThresholded); //show the thresholded image
    double cx=0.0,cy=0.0;
    unsigned int nb_pts=0,idx=0;
    for(unsigned int j=0;j<imgThresholded.rows;j++)
      {
	for(unsigned int i=0;i<imgThresholded.cols;i++)
	  {
	    if (imgThresholded.data[idx]>124)
	      {
		cx+=(double)i;
		cy+=(double)j;
		nb_pts++;
	      }
	    idx+=1;
	  }
      }
    cx = cx/(double)nb_pts;
    cy = cy/(double)nb_pts;

    // Draw the CoG in the image
    cv::circle(cv_ptr->image, cv::Point((int)cx,(int)cy), 10, CV_RGB(0,0,255));
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);

    // Control filling
    /* A COMPLETER */
    cv::waitKey(3);
  }
};


int main( int argc, char** argv )
{
  /* Node initialization */
  /* A COMPLETER */
  ImageConverter ic;
  ros::spin();
}
