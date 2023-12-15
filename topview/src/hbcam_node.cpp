#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <math.h>
#include "opencv2/stitching.hpp"
using namespace std; 
using namespace cv;

#define WIDTH 320
#define HEIGHT 240
class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher();
private:
  Mat flont_high;
  Mat flont_low;
  Mat behind_high;
  Mat behind_low;
  Mat flontimg;
  Mat behindimg;
  void publishflont();
  void flonthighcallback(sensor_msgs::msg::Image::SharedPtr msg);
  void flontlowcallback(sensor_msgs::msg::Image::SharedPtr msg);
  void behindhighcallback(sensor_msgs::msg::Image::SharedPtr msg);
  void behindlowcallback(sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr flonthighsubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr flontlowsubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr behindhighsubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr behindlowsubscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr flontimg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr behindimg_pub_;

};
ImagePublisher::ImagePublisher() : Node("hdcam_node"){
  flont_high=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  behind_low=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  behind_high=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  behind_low=cv::Mat(HEIGHT,WIDTH,CV_8UC3);

  flonthighsubscriber_=this->create_subscription<sensor_msgs::msg::Image>("/video0/image_raw", 10,std::bind(&ImagePublisher::flonthighcallback,this,std::placeholders::_1));
  flontlowsubscriber_=this->create_subscription<sensor_msgs::msg::Image>("/video2/image_raw", 10,std::bind(&ImagePublisher::flontlowcallback,this,std::placeholders::_1));
  behindhighsubscriber_=this->create_subscription<sensor_msgs::msg::Image>("/video4/image_raw", 10,std::bind(&ImagePublisher::behindhighcallback,this,std::placeholders::_1));
  behindlowsubscriber_=this->create_subscription<sensor_msgs::msg::Image>("/video6/image_raw", 10,std::bind(&ImagePublisher::behindlowcallback,this,std::placeholders::_1));
  flontimg_pub_ = create_publisher<sensor_msgs::msg::Image>("image_flont",10);
  behindimg_pub_=create_publisher<sensor_msgs::msg::Image>("image_behind",10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / 30), std::bind(&ImagePublisher::publishflont, this));
}

void ImagePublisher::flonthighcallback(sensor_msgs::msg::Image::SharedPtr msg){
  auto cv_img=cv_bridge::toCvCopy(msg,"bgr8");
  flont_high=cv_img->image;
}

void ImagePublisher::flontlowcallback(sensor_msgs::msg::Image::SharedPtr msg){
  auto cv_img=cv_bridge::toCvCopy(msg,"bgr8");
  flont_low=cv_img->image;
}

void ImagePublisher::behindhighcallback(sensor_msgs::msg::Image::SharedPtr msg){
  auto cv_img=cv_bridge::toCvCopy(msg,"bgr8");
  behind_high=cv_img->image;
}

void ImagePublisher::behindlowcallback(sensor_msgs::msg::Image::SharedPtr msg){
  auto cv_img=cv_bridge::toCvCopy(msg,"bgr8");
  behind_low=cv_img->image;
}
void ImagePublisher::publishflont(){
    vector<Mat> imgArray;
    Mat img1;
    Mat img2;
    resize(flont_high,img1,Size(), (double)WIDTH/flont_high.cols ,double(HEIGHT)/flont_low.rows);
    resize(flont_low,img2,Size(), (double)WIDTH/flont_low.cols ,double(HEIGHT)/flont_low.rows);
    imgArray.push_back(img1);
    imgArray.push_back(img2);
    Mat pano=Mat(HEIGHT,WIDTH,CV_8UC3);
    pano.setTo(Scalar(0));
    Stitcher::Mode mode =Stitcher::PANORAMA;
    Ptr<cv::Stitcher> stitcher=Stitcher::create(mode);
    stitcher->stitch(imgArray,pano);
    cv_bridge::CvImage cv_image;
    sensor_msgs::msg::Image pub_image;
    cv_image.encoding = "bgr8";
    cv_image.image=img1;
    cv_image.toImageMsg(pub_image);
    flontimg_pub_->publish(pub_image);
    imshow("pano",pano);
    waitKey(1);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
