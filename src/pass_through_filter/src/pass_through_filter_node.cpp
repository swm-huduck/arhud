#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/point_cloud_conversion.h>
#include <pcl/conversions.h>
#include<pcl/filters/passthrough.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include <ros/ros.h>


ros::Publisher pub;
// 필터 목적 : x가 음수인(x < 0m) 전부 필터링해서 걸러줌
void cloud_cb(const sensor_msgs::PointCloud2 &input)
{
  // pcl로 형변환
  std::cout<<5<<"\n\n";

  pcl::PointCloud<pcl::PointXYZI>:: Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
std::cout<<6<<"\n\n";
  pcl::fromROSMsg(input, *cloud);
std::cout<<7<<"\n\n";
  // 필터링 후 point cloud : cloud_filtered 선언
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZI>);
  // PassThrough 필터
  pcl::PassThrough<pcl::PointXYZI> filter;
 //출력
  /*ros::Time stamp1= ros::Time::now();
  std::stringstream ss1;
  ss1 << stamp1.sec << "." << stamp1.nsec;
  std::cerr << ss1.str() << "\n";  
  std::cerr<<"Cloud before Pass_thorugh_filter"<<"\n";
  std::cerr<<*cloud<<"\n";
  */
  // 범위 설정
  double min_range_x = 0;   // 최소 0m
  double max_range_x = 100; //최대 100m
  
  //PassThrough 필터 적용
  filter.setInputCloud(cloud);
  filter.setFilterFieldName("x");
  filter.setFilterLimits(min_range_x, max_range_x);
  filter.setFilterLimitsNegative(false);
  filter.filter(*cloud_filtered);

  double min_range_y=-3;
  double max_range_y= 3;
  filter.setInputCloud(cloud_filtered);
  filter.setFilterFieldName("y");
  filter.setFilterLimits(min_range_y, max_range_y);
  filter.setFilterLimitsNegative(false);
  filter.filter(*cloud_filtered2);
std::cout<<9<<"\n\n";  
  //출력
  /*ros::Time stamp= ros::Time::now();
  std::stringstream ss;
  ss << stamp.sec << "." << stamp.nsec;
  std::cerr << ss.str() << "\n";  
  std::cerr<<"Cloud after Pass_thorugh_filter"<<"\n";
  std::cerr<<*cloud_filtered2<<"\n";
  */
  // 퍼블리시
  pub.publish(cloud_filtered2);
std::cout<<10<<"\n\n";

}


int main (int argc, char** argv)
{
std::cout<<1<<"\n\n";
  // Initialize ROS
  ros::init (argc, argv, "Pass_Through_Filter");
  ros::NodeHandle nh;
std::cout<<2<<"\n\n";
  ros::Subscriber sub = nh.subscribe("velodyne_points", 1, cloud_cb);
std::cout<<3<<"\n\n";
  pub = nh.advertise<sensor_msgs::PointCloud2>("pass_through_cloud", 10);
  //pub = nh.advertise<std_msgs::Int32MultiArray> ("PointCloud2PointCloud", 10);
std::cout<<4<<"\n\n";

  ros::spin();

  return 0;
}

