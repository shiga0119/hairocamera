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

void PinP_point(const cv::Mat &srcImg, const cv::Mat &smallImg, const cv::Point2f p0, const cv::Point2f p1){
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

class topviewbody : public ParallelLoopBody{
    Mat undistortImage;
    Mat topImage;
    int y;
    int mode;
    double hvc[2] = {2,2}; //
    double hc[2] = {0.7,0.7}; //
    double dvc[2] ={1.7,1.7}; //
    double tht[2]={27,27};
    int xl[2]= {WIDTH/2,HEIGHT/2};
    int yl[2]= {HEIGHT/2,WIDTH/2};
public:
	topviewbody(Mat& undistortImage,Mat& topImage,int y,int mode) : undistortImage(undistortImage),topImage(topImage),y(y),mode(mode){}
	void operator()(const Range& range) const override
  {
        double Hvc = hvc[mode]; //
        double Hc = hc[mode]; //
        double Dvc = dvc[mode]; //
        double f = 630;
        double fp = f;
        double theta = tht[mode]/ 180.0 * M_PI;
        double s = sin(theta);
        double c = cos(theta);
        int cx = xl[mode];
        int cy = yl[mode];
        int cxp = xl[mode];
        int cyp = yl[mode];
        for (int x = 0; x < topImage.cols; x++) {
            int xOrg = x - cx;
            int yOrg = - y + cy;

            double oldX = 0.5 + (Hvc / Hc) * (f / fp) * c * ( s/c - (yOrg*Hvc*s - fp*Hc*c + fp*Dvc*s) / (fp*Hc*s + Hvc*yOrg*c + fp*Dvc*c) ) * xOrg;
            double oldY = 0.5 + f * ((yOrg*Hvc*s - fp*Hc*c + fp*Dvc*s)/(fp*Hc*s + Hvc*yOrg*c + fp*Dvc*c));

            oldX = oldX + cxp;
            oldY = -oldY + cyp;

            if (oldX < 0 || topImage.cols - 1 < oldX || oldY < 0 || topImage.rows - 1 < oldY ) {
            continue;
            }

            if((int)oldX + 1 >= topImage.cols || (int)oldY + 1 >= topImage.rows) {
            topImage.data[(y * topImage.cols + x) * topImage.channels()] = undistortImage.data[((int)oldY * topImage.cols + (int)oldX) * topImage.channels()];
            topImage.data[(y * topImage.cols + x) * topImage.channels() + 1] = undistortImage.data[((int)oldY * topImage.cols + (int)oldX) * topImage.channels() + 1];
            topImage.data[(y * topImage.cols + x) * topImage.channels() + 2] = undistortImage.data[((int)oldY * topImage.cols + (int)oldX) * topImage.channels() + 2];
            continue;
            }


            for (int i = 0; i < topImage.channels(); i++) {

            uchar f11 = undistortImage.data[((int)oldY * topImage.cols + (int)oldX) * topImage.channels() + i];
            uchar f12 = undistortImage.data[(((int)oldY + 1) * topImage.cols + (int)oldX) * topImage.channels() + i];
            uchar f21 = undistortImage.data[((int)oldY * topImage.cols + (int)oldX + 1) * topImage.channels() + i];
            uchar f22 = undistortImage.data[(((int)oldY + 1) * topImage.cols + (int)oldX + 1) * topImage.channels() + i];

            double dx2 = (int)oldX + 1 - oldX;
            double dx1 = oldX - (int)oldX;

            double dy2 = (int)oldY + 1 - oldY;
            double dy1 = oldY - (int)oldY;

            topImage.data[(y * topImage.cols + x) * topImage.channels() + i] = dy2 * (f11 * dx2 + f21 * dx1) + dy1 * (f12 * dx2 + f22 * dx1);
            }
            }
	}
};

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher();
private:
  Mat tpvflont;
  Mat tpvbehind;
  Mat tpvright;
  Mat tpvleft;
  void publishImage();
  void flontcallback(sensor_msgs::msg::Image::SharedPtr msg);
  void behindcallback(sensor_msgs::msg::Image::SharedPtr msg);
  void rightcallback(sensor_msgs::msg::Image::SharedPtr msg);
  void leftcallback(sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr flontsubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr behindsubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rightsubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr leftsubscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tpvimg_pub_;
};
ImagePublisher::ImagePublisher() : Node("topview_node"){
  tpvflont=cv::Mat(WIDTH,HEIGHT,CV_8UC3);
  tpvbehind=cv::Mat(WIDTH,HEIGHT,CV_8UC3);
  tpvright=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  tpvleft=cv::Mat(HEIGHT,WIDTH,CV_8UC3);
  flontsubscriber_=this->create_subscription<sensor_msgs::msg::Image>("/video0/image_raw", 10,std::bind(&ImagePublisher::flontcallback,this,std::placeholders::_1));
  behindsubscriber_=this->create_subscription<sensor_msgs::msg::Image>("/video2/image_raw", 10,std::bind(&ImagePublisher::behindcallback,this,std::placeholders::_1));
  rightsubscriber_=this->create_subscription<sensor_msgs::msg::Image>("/video4/image_raw", 10,std::bind(&ImagePublisher::rightcallback,this,std::placeholders::_1));
  leftsubscriber_=this->create_subscription<sensor_msgs::msg::Image>("/video6/image_raw", 10,std::bind(&ImagePublisher::leftcallback,this,std::placeholders::_1));
  tpvimg_pub_ = create_publisher<sensor_msgs::msg::Image>("image_tpv",10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / 30), std::bind(&ImagePublisher::publishImage, this));
}

void ImagePublisher::flontcallback(sensor_msgs::msg::Image::SharedPtr msg){
  auto cv_img=cv_bridge::toCvCopy(msg,"bgr8");
  Mat flont=cv_img->image;
  rotate(flont,flont,cv::ROTATE_90_CLOCKWISE);
  tpvflont.setTo(Scalar(0));
  for (int y = 0; y < flont.rows; y++) {
            parallel_for_(Range{0,flont.rows},topviewbody{flont,tpvflont,y,1},2);
  }
}

void ImagePublisher::behindcallback(sensor_msgs::msg::Image::SharedPtr msg){
  auto cv_img=cv_bridge::toCvCopy(msg,"bgr8");
  Mat behind=cv_img->image;
  rotate(behind,behind,cv::ROTATE_90_CLOCKWISE);
  tpvbehind.setTo(Scalar(0));
  for (int y = 0; y < behind.rows; y++) {
            parallel_for_(Range{0,behind.rows},topviewbody{behind,tpvbehind,y,1},2);
  }
}

void ImagePublisher::rightcallback(sensor_msgs::msg::Image::SharedPtr msg){
  auto cv_img=cv_bridge::toCvCopy(msg,"bgr8");
  Mat right=cv_img->image;
  tpvright.setTo(Scalar(0));
  for (int y = 0; y < right.rows; y++) {
            parallel_for_(Range{0,right.rows},topviewbody{right,tpvright,y,0},2);
  }
}
void ImagePublisher::leftcallback(sensor_msgs::msg::Image::SharedPtr msg){
  auto cv_img=cv_bridge::toCvCopy(msg,"bgr8");
  Mat left=cv_img->image;
  tpvleft.setTo(Scalar(0));
  for (int y = 0; y < left.rows; y++) {
            parallel_for_(Range{0,left.rows},topviewbody{left,tpvleft,y,0},2);
  }

}

void ImagePublisher::publishImage(){
  Mat trnright;
  Mat trnleft;
  Mat trnbehind;
  rotate(tpvbehind,trnbehind, cv::ROTATE_180);
  rotate(tpvright,trnright,cv::ROTATE_90_CLOCKWISE);
  rotate(tpvleft,trnleft,cv::ROTATE_90_COUNTERCLOCKWISE);
  Mat tpv_img(WIDTH*3,HEIGHT*3,CV_8UC3);
  tpv_img.setTo(Scalar(0));
  Point2f flontp0(HEIGHT,50);
  Point2f flontp1(HEIGHT*2,WIDTH+50);
  Point2f behindp0(HEIGHT,WIDTH*2-50);
  Point2f behindp1(HEIGHT*2,WIDTH*3-50);
  Point2f rightp0(HEIGHT*2-50,WIDTH);
  Point2f rightp1(HEIGHT*3-50,WIDTH*2);
  Point2f leftp0(50,WIDTH);
  Point2f leftp1(HEIGHT+50,WIDTH*2);
  PinP_point(tpv_img,tpvflont,flontp0,flontp1);
  PinP_point(tpv_img,trnbehind,behindp0,behindp1);
  PinP_point(tpv_img,trnright,rightp0,rightp1);
  PinP_point(tpv_img,trnleft,leftp0,leftp1);
  cv_bridge::CvImage cv_image;
  sensor_msgs::msg::Image pub_image;
  cv_image.encoding = "bgr8";
  cv_image.image=tpv_img;
  cv_image.toImageMsg(pub_image);
  tpvimg_pub_->publish(pub_image);
}
int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
