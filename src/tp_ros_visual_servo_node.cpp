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
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher cmd_vel_pub_;

  int iLowH_,iHighH_;
  int iLowS_,iHighS_;
  int iLowV_,iHighV_;

  // Commandes parameters are made with a tuple =
  // [Parameter name, current value of the parameter, default value of the parameter]
  typedef std::tuple<std::string, double, double> param_ltype;
  // Dictionnary of the parameters
  std::map<std::string, param_ltype> list_params;

  // Center of gravity.
  double cx_, cy_;
  unsigned int nb_pts_;

public:
  ImageConverter()
    : it_(nh_),iLowH_(15), iHighH_(83),
      iLowS_(190), iHighS_(226),
      iLowV_(0), iHighV_(255),
      cx_(0.0),cy_(0.0), nb_pts_(0)
  {
    // Creates subscriber to get the input video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw",1,&ImageConverter::imageCb,this);

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
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    list_params["Kp"]=std::make_tuple(std::string("/visual_servo/Kp"),0.001,0.001);
    list_params["dx_clamp"]=std::make_tuple(std::string("/visual_servo/dx_clamp"),10.0,10.0);
    list_params["wz_min"]=std::make_tuple(std::string("/visual_servo/wz_min"),0.01,0.01);
    list_params["wz_max"]=std::make_tuple(std::string("/visual_servo/wz_max"),1.7,1.7);

    initParameters();
  }

  void initParameters()
  {
    // For each parameters
    for ( auto it = list_params.begin(); it != list_params.end(); ++it )
      {
	// If the parameter is already set
	if (nh_.hasParam(std::get<0>(it->second)))
	  // Read the current value.
	  nh_.getParam(std::get<0>(it->second),std::get<1>(it->second));
	else
	  // Set the default value.
	  nh_.setParam(std::get<0>(it->second),std::get<1>(it->second));
      }
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  double computeCmdZ(double DeltaX)
  {
    double Kp = std::get<1>(list_params["Kp"]);
    double DeltaX_Clamp = std::get<1>(list_params["dx_clamp"]);
    double wz_min = std::get<1>(list_params["wz_min"]);
    double wz_max = std::get<1>(list_params["wz_max"]);

    // Compute simple linear relationship.
    double cmdz = -Kp*(DeltaX);

    ROS_INFO_STREAM("Kp: " << Kp);
    ROS_INFO_STREAM("DeltaX: " << DeltaX);
    ROS_INFO_STREAM("cmdz: " << cmdz);
    // Lowest saturation
    double signe_cmd = cmdz > 0.0 ? 1.0 : -1.0;

    // Control saturation
    if (fabs(cmdz)>wz_max)
      cmdz = signe_cmd*wz_max;
    // Set control to zero if the robot is near the target.
    else if (DeltaX_Clamp*Kp>fabs(cmdz))
      cmdz=0.0;
    // Minimal value if the robot is not near enough but needs to move
    else if (wz_min > fabs(cmdz))
      cmdz = signe_cmd*wz_min;

    return cmdz;
  }

  void processImages(const sensor_msgs::ImageConstPtr& msg,
			       cv_bridge::CvImagePtr & cv_ptr,
			       Mat & imgThresholded)
  {
    // From ros image to opencv
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Mat imgHSV;
    // From RGB to HSV
    cvtColor(cv_ptr->image, imgHSV, COLOR_RGB2HSV);

    // Analysis object and provides a Thresholded image
    inRange(imgHSV, Scalar(iLowH_, iLowS_, iLowV_),
	    Scalar(iHighH_, iHighS_, iHighV_), imgThresholded);
  }

  void displayImages(cv_bridge::CvImagePtr & cv_ptr,
		     Mat & imgThresholded)
  {
    if (nb_pts_>0)
      {
      	// Draw the CoG in the image
	cv::circle(cv_ptr->image, cv::Point((int)cx_,(int)cy_), 10, CV_RGB(0,0,255));
      }

    // Show the thresholded image
    imshow("Thresholded Image", imgThresholded);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);

  }

  void computeCoG(Mat & imgThresholded)
  {
    // Compute Center-Of-Gravity of the object.
    cx_=0.0,cy_=0.0;
    nb_pts_=0;
    unsigned long int idx=0;
    for(unsigned int j=0;j<imgThresholded.rows;j++)
      {
	for(unsigned int i=0;i<imgThresholded.cols;i++)
	  {
	    if (imgThresholded.data[idx]>124)
	      {
		cx_+=(double)i;
		cy_+=(double)j;
		nb_pts_++;
	      }
	    idx+=1;
	  }
      }
    if (nb_pts_>0)
      {
	cx_ = cx_/(double)nb_pts_;
	cy_ = cy_/(double)nb_pts_;
      }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& imgMsg)
  {

    // 1 - Update each parameters by iterating over list_params.
    for ( auto it = list_params.begin(); it != list_params.end(); ++it )
      nh_.getParam(std::get<0>(it->second),std::get<1>(it->second));

    // 2- Image processing
    // Convert sensors_msg::Image to opencv image
    cv_bridge::CvImagePtr cv_ptr;
    // Result image
    Mat imgThresholded;
    processImages(imgMsg,cv_ptr,imgThresholded);

    // Compute the Center-of-Gravity.
    computeCoG(imgThresholded);

    // 3 - Compute Control for base
    // Creating the ros message to be send for the robot's base.
    geometry_msgs::Twist cmd_vel_msg;
    // Linear part of the cmd
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
    // Angular part of the cmd
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    // Default behavior turn left to find an object.
    cmd_vel_msg.angular.z = -0.2;

    // This is necessary to avoid sending NaN to Gazebo.
    if (nb_pts_>0)
      cmd_vel_msg.angular.z = computeCmdZ(cx_-(double)imgThresholded.cols/2.0);

    // 4 - Send Control value to base
    // Publish Twist msg on topic.
    cmd_vel_pub_.publish(cmd_vel_msg);

    displayImages(cv_ptr, imgThresholded);

    // 5 - Opencv update
    cv::waitKey(3);
  }
};


int main( int argc, char** argv )
{
  ros::init(argc,argv,"tp_ros_visual_servo");
  ImageConverter ic;
  ros::spin();
}
