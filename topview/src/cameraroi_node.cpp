#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <math.h>
using namespace std; 
using namespace cv;

#define WIDTH 320
#define HEIGHT 240



class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher();
private:
  Mat front_high;
  Mat front_low;
  Mat behind_high;
  Mat behind_low;
  Mat front;
  Mat behind;
  Mat f_high_roi;
  Mat f_low_roi;
  Mat b_high_roi;
  Mat b_low_roi;
  void publishfront();
  void publishbehind();
  void fronthighcallback(sensor_msgs::msg::Image::SharedPtr msg);
  void frontlowcallback(sensor_msgs::msg::Image::SharedPtr msg);
  void behindhighcallback(sensor_msgs::msg::Image::SharedPtr msg);
  void behindlowcallback(sensor_msgs::msg::Image::SharedPtr msg);
  void PinP_point(const cv::Mat &srcImg, const cv::Mat &smallImg, const cv::Point2f p0, const cv::Point2f p1);
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr fronthighsubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frontlowsubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr behindhighsubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr behindlowsubscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frontimg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr behindimg_pub_;
};
ImagePublisher::ImagePublisher() : Node("cameraroi_node"){
  front=cv::Mat(HEIGHT*2+10,WIDTH,CV_8UC3);
  front.setTo(Scalar(0));
  behind=cv::Mat(HEIGHT*2+10,WIDTH,CV_8UC3);
  behind.setTo(Scalar(0));
  front_high=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  behind_low=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  behind_high=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  behind_low=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  f_high_roi=cv::Mat(front,cv::Rect(0,0,WIDTH,HEIGHT));
  f_low_roi=cv::Mat(front,cv::Rect(0,HEIGHT+10,WIDTH,HEIGHT));
  b_high_roi=cv::Mat(behind,cv::Rect(0,0,WIDTH,HEIGHT));
  b_low_roi=cv::Mat(behind,cv::Rect(0,HEIGHT+10,WIDTH,HEIGHT));
  declare_parameter("front_high","/video0/image_raw");
  declare_parameter("front_low","/video2/image_raw");
  declare_parameter("behind_high","/video4/image_raw");
  declare_parameter("behind_low","/video6/image_raw");
  std::string fronthighpath_=get_parameter("front_high").as_string();
  std::string frontlowpath_=get_parameter("front_low").as_string();
  std::string behindhighpath_=get_parameter("behind_high").as_string();
  std::string behindlowpath_=get_parameter("behind_low").as_string();
  fronthighsubscriber_=this->create_subscription<sensor_msgs::msg::Image>(fronthighpath_, 10,std::bind(&ImagePublisher::fronthighcallback,this,std::placeholders::_1));
  frontlowsubscriber_=this->create_subscription<sensor_msgs::msg::Image>(frontlowpath_, 10,std::bind(&ImagePublisher::frontlowcallback,this,std::placeholders::_1));
  behindhighsubscriber_=this->create_subscription<sensor_msgs::msg::Image>(behindhighpath_, 10,std::bind(&ImagePublisher::behindhighcallback,this,std::placeholders::_1));
  behindlowsubscriber_=this->create_subscription<sensor_msgs::msg::Image>(behindlowpath_, 10,std::bind(&ImagePublisher::behindlowcallback,this,std::placeholders::_1));
  frontimg_pub_ = create_publisher<sensor_msgs::msg::Image>("image_front",10);
  behindimg_pub_=create_publisher<sensor_msgs::msg::Image>("image_behind",10);
  timer1_ = this->create_wall_timer(std::chrono::milliseconds(1000 / 30), std::bind(&ImagePublisher::publishfront, this));
  timer2_ = this->create_wall_timer(std::chrono::milliseconds(1000 / 30), std::bind(&ImagePublisher::publishbehind, this));
}
void ImagePublisher::fronthighcallback(sensor_msgs::msg::Image::SharedPtr msg){
  auto cv_img=cv_bridge::toCvCopy(msg,"bgr8");
  front_high=cv_img->image;
  
}

void ImagePublisher::frontlowcallback(sensor_msgs::msg::Image::SharedPtr msg){
  auto cv_img=cv_bridge::toCvCopy(msg,"bgr8");
  front_low=cv_img->image;
}  

void ImagePublisher::behindhighcallback(sensor_msgs::msg::Image::SharedPtr msg){
  auto cv_img=cv_bridge::toCvCopy(msg,"bgr8");
  behind_high=cv_img->image;

  }

void ImagePublisher::behindlowcallback(sensor_msgs::msg::Image::SharedPtr msg){
  auto cv_img=cv_bridge::toCvCopy(msg,"bgr8");
  behind_low=cv_img->image;
}

void ImagePublisher::publishfront(){
  front.setTo(Scalar(0));
  Mat cpy_high=front_high;
  Mat cpy_low=front_low;
  if(front_high.cols>0 &&front_high.rows>0){
  add(cpy_high,f_high_roi,f_high_roi);
  }
  if(front_low.cols>0 && front_low.rows>0){
  add(cpy_low,f_low_roi,f_low_roi);
  }
  cv_bridge::CvImage cv_image;
  sensor_msgs::msg::Image pub_image;
  cv_image.encoding = "bgr8";
  cv_image.image=front;
  cv_image.toImageMsg(pub_image);
  frontimg_pub_->publish(pub_image);
}
void ImagePublisher::publishbehind(){
  behind.setTo(Scalar(0));
  Mat cpy_high=front_high;
  Mat cpy_low=front_low;
  if(behind_low.cols>0&&behind_high.rows>0){
  add(cpy_high,b_high_roi,b_high_roi);
  }
  if(behind_low.cols==WIDTH&&behind_low.rows==HEIGHT){
  add(cpy_low,b_low_roi,b_low_roi);
  }
  cv_bridge::CvImage cv_image;
  sensor_msgs::msg::Image pub_image;
  cv_image.encoding = "bgr8";
  cv_image.image=behind;
  cv_image.toImageMsg(pub_image);
  behindimg_pub_->publish(pub_image);
}
int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}