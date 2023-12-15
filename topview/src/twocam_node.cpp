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
  int front_y;
  int behind_y;
  int front_y_low=0;
  int behind_y_low=0;
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
ImagePublisher::ImagePublisher() : Node("twocam_node"){
  front_high=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  behind_low=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  behind_high=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  behind_low=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  declare_parameter("front_high","/video2/image_raw");
  declare_parameter("front_low","/video4/image_raw");
  declare_parameter("behind_high","/video6/image_raw");
  declare_parameter("behind_low","/video8/image_raw");
  declare_parameter("front_y",WIDTH);
  declare_parameter("behind_y",WIDTH);
  std::string fronthighpath_=get_parameter("front_high").as_string();
  std::string frontlowpath_=get_parameter("front_low").as_string();
  std::string behindhighpath_=get_parameter("behind_high").as_string();
  std::string behindlowpath_=get_parameter("behind_low").as_string();
  front_y=get_parameter("front_y").as_int();
  behind_y=get_parameter("behind_y").as_int();
  if(front_y>WIDTH){
    front_y_low=front_y-WIDTH;
    front_y=WIDTH;
  }
  if(behind_y>WIDTH){
    behind_y_low=behind_y-WIDTH;
    behind_y=WIDTH;
  }
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
  Mat front=cv::Mat(HEIGHT+int(WIDTH*WIDTH/HEIGHT),WIDTH,CV_8UC3);
  front.setTo(Scalar(0));
  if(front_high.cols>0&&front_high.rows>0){
    Mat cpyfront_high; 
    rotate(front_high,cpyfront_high,cv::ROTATE_90_COUNTERCLOCKWISE);
    cpyfront_high=cpyfront_high(Rect(0,0,HEIGHT,front_y)).clone();
     Point2f fronthp0(0,front_y_low);
    Point2f fronthp1(WIDTH,int(front_y*WIDTH/HEIGHT)+front_y_low);
    PinP_point(front,cpyfront_high,fronthp0,fronthp1);
  }
  if(front_low.cols>0&&front_low.rows>0){
    Mat cpyfront_low;
    rotate(front_low,cpyfront_low,cv::ROTATE_180);
    cpyfront_low=cpyfront_low(Rect(0,front_y_low,WIDTH,HEIGHT-front_y_low)).clone();
    Point2f frontlp0(0,int(front_y*WIDTH/HEIGHT)+front_y_low);
    Point2f frontlp1(WIDTH,HEIGHT+int(front_y*WIDTH/HEIGHT));
    PinP_point(front,cpyfront_low,frontlp0,frontlp1);
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
  Mat behind=cv::Mat(HEIGHT+int(WIDTH*WIDTH/HEIGHT),WIDTH,CV_8UC3);
  behind.setTo(Scalar(0));
  if(behind_high.cols>0&&behind_high.rows>0){
    Mat cpybehind_high; 
    rotate(behind_high,cpybehind_high,cv::ROTATE_90_COUNTERCLOCKWISE);
    cpybehind_high=cpybehind_high(Rect(0,0,HEIGHT,behind_y)).clone(); 
    Point2f behindhp0(0,behind_y_low);
    Point2f behindhp1(WIDTH,int(behind_y*WIDTH/HEIGHT)+behind_y_low);
    PinP_point(behind,cpybehind_high,behindhp0,behindhp1);
  }
  if(front_low.cols>0&&front_low.rows>0){
  Mat cpybehind_low;
  cpybehind_low=behind_low(Rect(0,behind_y_low,WIDTH,HEIGHT-behind_y_low)).clone();
  Point2f behindlp0(0,int(behind_y*WIDTH/HEIGHT)+behind_y_low);
  Point2f behindlp1(WIDTH,int(behind_y*WIDTH/HEIGHT)+HEIGHT);
  PinP_point(behind,cpybehind_low,behindlp0,behindlp1);
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
