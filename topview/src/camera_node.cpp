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
  int front_y=WIDTH;
  int behind_y=WIDTH;
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
ImagePublisher::ImagePublisher() : Node("camera_node"){
  namedWindow("parameter", 1);
  createTrackbar("front_y","parameter",&front_y,WIDTH);
  createTrackbar("behind_y","parameter",&behind_y,WIDTH);
  front_high=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  behind_low=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  behind_high=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  behind_low=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
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
void ImagePublisher::PinP_point(const cv::Mat &srcImg, const cv::Mat &smallImg, const cv::Point2f p0, const cv::Point2f p1){
    cv::Mat dstImg;
    srcImg.copyTo(dstImg);
    vector<cv::Point2f> src, dst;
    src.push_back(cv::Point2f(0, 0));
    src.push_back(cv::Point2f(smallImg.cols, 0));
    src.push_back(cv::Point2f(smallImg.cols, smallImg.rows));
    dst.push_back(p0);
    dst.push_back(cv::Point2f(p1.x, p0.y));
    dst.push_back(p1);
    cv::Mat mat = cv::getAffineTransform(src, dst);
    cv::warpAffine(smallImg, dstImg, mat, dstImg.size(), CV_INTER_LINEAR, cv::BORDER_TRANSPARENT);
    dstImg.copyTo(srcImg);
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
  Mat cpyfront_high;
  rotate(front_high,cpyfront_high,cv::ROTATE_90_CLOCKWISE);
  cpyfront_high=cpyfront_high(Rect(0,0,HEIGHT,front_y)).clone();
  Mat front=cv::Mat(HEIGHT+int(WIDTH*WIDTH/HEIGHT),WIDTH,CV_8UC3);
  front.setTo(Scalar(0));
  Point2f fronthp0(0,0);
  Point2f fronthp1(WIDTH,int(front_y*WIDTH/HEIGHT));
  Point2f frontlp0(0,int(front_y*WIDTH/HEIGHT));
  Point2f frontlp1(WIDTH,HEIGHT+int(front_y*WIDTH/HEIGHT));
  if(front_high.cols>0&&front_high.rows>0){  
  PinP_point(front,cpyfront_high,fronthp0,fronthp1);
  }
  if(front_low.cols>0&&front_low.rows>0){
  PinP_point(front,front_low,frontlp0,frontlp1);
  }
  cv_bridge::CvImage cv_image;
  sensor_msgs::msg::Image pub_image;
  cv_image.encoding = "bgr8";
  cv_image.image=front;
  cv_image.toImageMsg(pub_image);
  frontimg_pub_->publish(pub_image);
  waitKey(1);
}
void ImagePublisher::publishbehind(){
  Mat cpybehind_high;
  rotate(behind_high,cpybehind_high,cv::ROTATE_90_CLOCKWISE);
  cpybehind_high=cpybehind_high(Rect(0,0,HEIGHT,behind_y)).clone();
  Mat behind=cv::Mat(HEIGHT+int(WIDTH*WIDTH/HEIGHT),WIDTH,CV_8UC3);
  behind.setTo(Scalar(0));
  Point2f behindhp0(0,0);
  Point2f behindhp1(WIDTH,int(behind_y*WIDTH/HEIGHT));
  Point2f behindlp0(0,int(behind_y*WIDTH/HEIGHT));
  Point2f behindlp1(WIDTH,int(behind_y*WIDTH/HEIGHT)+HEIGHT);
  if(behind_high.cols>0&&behind_high.rows>0){  
  PinP_point(behind,cpybehind_high,behindhp0,behindhp1);
  }
  if(front_low.cols>0&&front_low.rows>0){
  PinP_point(behind,behind_low,behindlp0,behindlp1);
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