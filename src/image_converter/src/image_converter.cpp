#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/UInt8MultiArray.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 720

using namespace cv;
static const std::string IMG_TOPIC = "camera/image_rect_color";
static const std::string COMPOSITE_IMG_OUT = "Type_Converted_Image"; // lidar points on camera image
ros::Publisher image_pub_;

void imageCb(const sensor_msgs::ImageConstPtr &image_msg)
{

    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    try 
    {
      //cv::waitKey(10);
      //imshow("image", cv_bridge::toCvShare(image_msg, "bgr8")->image);
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception &e) {
      std::cout<<"    1         "<<1<<"\n\n\n\n\n";
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
/*        std::vector<uchar> encode;
        //cv::imencode(".bmp", cv_ptr->image, encode);
image_msgs->data
        // Convert encoded image to ROS std_msgs format
        std_msgs::UInt8MultiArray msgArray;
        msgArray.data.clear();
        msgArray.data.resize(encode.size());
        std::copy(encode.begin(), encode.end(), msgArray.data.begin());

*/
  // Publish msg
//  image_pub_.publish(msgArray);

  std_msgs::UInt8MultiArray msg;
  msg.data.clear();
  msg.data.resize(image_msg->data.size());

  std::copy(image_msg->data.begin(), image_msg->data.end(), msg.data.begin());
  memcpy(msg.data.data(), cv_ptr->image.data, image_msg->data.size());

  image_pub_.publish(msg);


}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh;
  cv::Mat frame;
  frame.empty();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub_ = it.subscribe(IMG_TOPIC, 1, imageCb);
  image_pub_ = nh.advertise<std_msgs::UInt8MultiArray>(COMPOSITE_IMG_OUT, 1);
  ros::spin();
  return 0;
}



