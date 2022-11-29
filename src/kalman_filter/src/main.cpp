#include "opencv2/video/tracking.hpp"
#include "pcl_ros/point_cloud.h"
#include <algorithm>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <string.h>

#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "kalman_filter/my_msg.h"
#include <limits>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace cv;

const int Number_of_Filter = 2;

cv::KalmanFilter KF[Number_of_Filter];
ros::Publisher pub_cluster[Number_of_Filter];
ros::Publisher markerPub;
ros::Publisher kalmanPub;
ros::Publisher objID_pub;
ros::Publisher publisher;

// KF init
int stateDim = 4; 
int measDim = 2;  
int ctrlDim = 0;

geometry_msgs::Point Pre_Position[Number_of_Filter];
geometry_msgs::Point Pre_Posi;

std::vector<geometry_msgs::Point> prevClusterCenters;
//state Matrix   measurementMatrix
// [x             [x   
//  v_x            y]
//  y
//  v_y]
cv::Mat state[Number_of_Filter]; 
cv::Mat measurement[Number_of_Filter];
cv::Mat processNoise[Number_of_Filter];

std::vector<int> objID; // Output of the data association using KF
                        // measurement.setTo(Scalar(0));
ros::Time now_Time;
ros::Time next_Time;

bool firstFrame = true;

double velocity(double v_x, double v_y)
{
  return (double)sqrt(v_x*v_x + v_y*v_y);
}

void KFT(const std_msgs::Float32MultiArray ccs) 
{
  std::vector<geometry_msgs::Point> clusterCenters; // clusterCenters
  
  for (std::vector<float>::const_iterator it = ccs.data.begin();
       it != ccs.data.end(); it += 3) {
    geometry_msgs::Point pt;
    pt.x = *it;
    pt.y = *(it + 1);
    pt.z = *(it + 2);
    //cout<<pt.x<<" "<<pt.y<<" "<<pt.z<<"\n";
    clusterCenters.push_back(pt);
  }

  // First predict, to update the internal statePre variable
  std::vector<cv::Mat> pred;
  for(int i=0;i<Number_of_Filter;i++)
  { 
    
    //cout<<i<<"\n";
    //cout<<"state\n"<<state[i]<<"\n";
    //cout<<"statePre\n"<<KF[i].statePre<<"\n"; 
    //cout<<KF[i].transitionMatrix<<"\n";
    pred.push_back(KF[i].predict());
    //cout<<" "<< pred[i].size()<<"\n";
    //cout<<"pred\n"<<pred[i]<<"\n";
  }//cout<<"\n";

  std::vector<geometry_msgs::Point> KFpredictions;
  
  int i = 0;
  for (auto it = pred.begin(); it != pred.end(); it++) 
  {
    geometry_msgs::Point pt;
    pt.x = (*it).at<float>(0);
    pt.y = (*it).at<float>(2);
    //cout<<pt.x <<" "<<pt.y<<"\n";
    pt.z = 0.0f;
    KFpredictions.push_back(pt);
  }
  // cout<<"Got predictions"<<"\n";
  for(int i=0;i<Number_of_Filter;i++)
  {
    randn(measurement[i], Scalar::all(0), Scalar::all(KF[i].measurementNoiseCov.at<float>(0,0)));
    measurement[i] = KF[i].measurementMatrix*state[i];
  }
  // Find the cluster that is more probable to be belonging to a given KF.
  objID.clear();   // Clear the objID vector
  objID.resize(Number_of_Filter); 


  visualization_msgs::MarkerArray clusterMarkers;
  for (int i = 0; i < Number_of_Filter; i++) {
    visualization_msgs::Marker m;

    m.id = i;
    m.type = visualization_msgs::Marker::CUBE;
    m.header.frame_id = "velodyne";
    m.scale.x = 2;
    m.scale.y = 1.5;
    m.scale.z = 1;
    m.action = visualization_msgs::Marker::ADD;
    m.color.a = 1.0;
    m.color.r = i % 2 ? 1 : 0;
    m.color.g = 0;
    m.color.b = 0;

    //m.color.r = i % 3 ? 1 : 0;
    //m.color.g = i % 4 ? 1 : 0;
    //m.color.b = i % 5 ? 1 : 0;

    // geometry_msgs::Point clusterC(clusterCenters.at(objID[i]));
    //geometry_msgs::Point clusterC(KFpredictions[i]);
    geometry_msgs::Point clusterC(clusterCenters[i]);
    m.pose.position.x = clusterC.x;
    m.pose.position.y = clusterC.y;
    m.pose.position.z = clusterC.z;
    //m.pose.orientation.x = 0.0;
    //m.pose.orientation.y = 0.0;
    //m.pose.orientation.z = 0.0;
    //m.pose.orientation.w = 1.0;
    //cout<<i<<" "<<m<<"\n";
    clusterMarkers.markers.push_back(m);
  }

  prevClusterCenters = clusterCenters;

  markerPub.publish(clusterMarkers);

  visualization_msgs::MarkerArray kalmanMarkers;
  for (int i = 0; i < Number_of_Filter; i++) {
    visualization_msgs::Marker m;

    m.id = i;
    m.type = visualization_msgs::Marker::CUBE;
    m.header.frame_id = "velodyne";
    m.scale.x = 2;
    m.scale.y = 1.5;
    m.scale.z = 1;
    m.action = visualization_msgs::Marker::ADD;
    m.color.a = 1.0;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = i % 3 ? 1 : 0;

    //m.color.r = i % 3 ? 1 : 0;
    //m.color.g = i % 4 ? 1 : 0;
    //m.color.b = i % 5 ? 1 : 0;

    // geometry_msgs::Point clusterC(clusterCenters.at(objID[i]));
    geometry_msgs::Point clusterC(KFpredictions[i]);    
    m.pose.position.x = clusterC.x;
    m.pose.position.y = clusterC.y;
    m.pose.position.z = clusterC.z;
    //m.pose.orientation.x = 0.0;
    //m.pose.orientation.y = 0.0;
    //m.pose.orientation.z = 0.0;
    //m.pose.orientation.w = 1.0;
    //cout<<i<<" "<<m<<"\n";
    kalmanMarkers.markers.push_back(m);
  }


  kalmanPub.publish(kalmanMarkers);


  std_msgs::Int32MultiArray obj_id;
  for (auto it = objID.begin(); it != objID.end(); it++) obj_id.data.push_back(*it);
  // Publish the object IDs
  objID_pub.publish(obj_id);
  // convert clusterCenters from geometry_msgs::Point to floats
  std::vector<std::vector<float>> cc;
  for (int i = 0; i < Number_of_Filter; i++) 
  {
    vector<float> pt;
    pt.push_back(clusterCenters[objID[i]].x);
    pt.push_back(clusterCenters[objID[i]].y);
    pt.push_back(clusterCenters[objID[i]].z);

    cc.push_back(pt);
  }
  // cout<<"cc[5][0]="<<cc[5].at(0)<<"cc[5][1]="<<cc[5].at(1)<<"cc[5][2]="<<cc[5].at(2)<<"\n";
  for(int i=0;i<Number_of_Filter;i++)
  {
    //cout<<i<<" measurement\n"<<measurement[i]<<'\n';
    if(!(measurement[i].at<float>(0,0)==0.0f && measurement[i].at<float>(1,0)==0.0f)) KF[i].correct(measurement[i]);
  }
  
  for(int i=0;i<Number_of_Filter;i++)
  {
    randn(processNoise[i], Scalar::all(0), Scalar::all(sqrt(KF[i].processNoiseCov.at<float>(0,0))));
    state[i].at<float>(0) = clusterCenters[i].x;
    //state[i].at<float>(1) = KFpredictions[i].at<float>(1);
    state[i].at<float>(2) = clusterCenters[i].y;
    //state[i].at<float>(2) = KFpredictions[i].at<float>(2);
    state[i] = KF[i].transitionMatrix*state[i];
  }
  for(int i=0;i<Number_of_Filter;i++)
  {
    if(clusterCenters[i].y<0) continue;
    double delta_x = Pre_Posi.x - clusterCenters[i].x; 
    double delta_y = Pre_Posi.y - clusterCenters[i].y;
    //cout<<i<<" "<<clusterCenters[i].x<<" "<<clusterCenters[i].y<<"\n";
    //cout<<"  "<<Pre_Position[i].x<<" "<<Pre_Position[i].y<<"\n";
    next_Time = ros::Time::now();
    //double delta_time = next_Time - now_Time;
    double v_x = (double)delta_x*10*3.6;
    double v_y = (double)delta_y*10*3.6;
    //cout<<i<<"velocity  "<<v_x<<"   "<<v_y<<"\n";
    double velocity = sqrt(v_x*v_x+v_y*v_y);
    if(delta_x>0)
    {
      cout<<velocity<<"\n";
      kalman_filter::my_msg msg;
      msg.x = clusterCenters[i].y*(-1);
      msg.depth = clusterCenters[i].x;
      cout<<"position "<<msg.x<<"   "<<msg.depth<<"\n";
      msg.level = 1;
      publisher.publish(msg);
    } 
    //cout<<i<<" "<<sqrt(v_x*v_x + v_y*v_y)<<"\n\n";
    //cout<<i<<" velocity"<<(double)velocity(v_x, v_y)<<"\n";
    Pre_Position[i].x = clusterCenters[i].x;
    Pre_Position[i].y = clusterCenters[i].y;
    Pre_Posi.x = clusterCenters[i].x;
    Pre_Posi.y = clusterCenters[i].y;
    now_Time = ros::Time::now();
  }
}
void publish_cloud(ros::Publisher &pub,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
  sensor_msgs::PointCloud2::Ptr clustermsg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cluster, *clustermsg);
  clustermsg->header.frame_id = "velodyne";
  clustermsg->header.stamp = ros::Time::now();
  pub.publish(*clustermsg);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
  // cout<<"IF firstFrame="<<firstFrame<<"\n";
  // If this is the first frame, initialize kalman filters for the clustered
  // objects
  if (firstFrame) {
    // Initialize 6 Kalman Filters; Assuming 6 max objects in the dataset.
    // Could be made generic by creating a Kalman Filter only when a new object
    // is detected

    //TransitionMatrix
    //[dx,  delta_t,  0,       0]
    //[ 0,      dvx,  0,       0]
    //[ 0,        0, dy, delta_y]
    //[ 0,        0,  0,     dvy]
    float dvx = 1.0f; // 1.0
    float dvy = 1.0f; // 1.0
    float dx = 1.0f;
    float dy = 1.0f;
    float delta_time = 0.1f;

    
    for(int i=0;i<Number_of_Filter;i++)
    {
      cv::randn(KF[i].statePost, Scalar::all(0), Scalar::all(0.1));
      cv::randn(state[i], Scalar::all(0), Scalar::all(0.1));
    }
    //transitionMatrix = A
    for(int i=0;i<Number_of_Filter;i++)
    {
      KF[i].transitionMatrix = (Mat_<float>(4, 4) << dx, delta_time,  0,       0, 
                                                      0,     dvx,  0,       0, 
                                                      0,       0, dy, delta_time, 
                                                      0,       0,  0,     dvy);
      //cout<<KF[i].transitionMatrix<<"\n"; 
    }//cout<<"\n";
    //measurementMatrix = H --> 단위행렬로 초기화
    //for(int i=0;i<Number_of_Filter;i++) cv::setIdentity(KF[i].measurementMatrix);
    for(int i=0; i<Number_of_Filter;i++)
    {
      KF[i].measurementMatrix = (Mat_<float>(2, 4) << 1, 0, 0, 0, 
                                                      0, 0, 1, 0);
    }
    // Process Noise Covariance Matrix Q
    // [ Ex 0  0    0 0    0 ]
    // [ 0  Ey 0    0 0    0 ]
    // [ 0  0  Ev_x 0 0    0 ]
    // [ 0  0  0    1 Ev_y 0 ]
    //// [ 0  0  0    0 1    Ew ]
    //// [ 0  0  0    0 0    Eh ]
    float sigmaP = 1e-5; //1-e-5 
    float sigmaQ = 1e-1;
    //processNoiseCov = Q --> sigmaP 단위행렬로 초기화
    for(int i=0;i<Number_of_Filter;i++) setIdentity(KF[i].processNoiseCov, Scalar::all(sigmaP));
 
    //measurementNoiceCov = R --> sigmaQ 단위행렬로 초기화
    for(int i=0;i<Number_of_Filter;i++) cv::setIdentity(KF[i].measurementNoiseCov, cv::Scalar(sigmaQ));
    for(int i=0;i<Number_of_Filter;i++) cv::setIdentity(KF[i].errorCovPost, Scalar::all(1));


    // Process the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    /* Creating the KdTree from input point cloud*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *input_cloud);

    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    //ec.setClusterTolerance(0.08);
    ec.setClusterTolerance(0.22);
    //ec.setMinClusterSize(10);
    ec.setMinClusterSize(50);
    //ec.setMaxClusterSize(600);
    ec.setMaxClusterSize(1000000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    /* Extract the clusters out of pc and save indices in cluster_indices.*/
    ec.extract(cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    // Vector of cluster pointclouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;
    // Cluster centroids
    std::vector<pcl::PointXYZ> clusterCentroids;

    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      float x = 0.0;
      float y = 0.0;
      float z = 0.0;
      int numPts = 0;
      for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

        cloud_cluster->points.push_back(input_cloud->points[*pit]);
        x += input_cloud->points[*pit].x;
        y += input_cloud->points[*pit].y;
        //z += input_cloud->points[*pit].z;
        numPts++;

        // dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
        //                                          origin);
        // mindist_this_cluster = std::min(dist_this_point,
        // mindist_this_cluster);
      }

      pcl::PointXYZ centroid;
      centroid.x = x / numPts;
      centroid.y = y / numPts;
      centroid.z = 0.0;
      //centroid.z = z / numPts;
      //cout<<centroid.x<<' '<<centroid.y<<" "<<centroid.z<<"\n\n";
      cluster_vec.push_back(cloud_cluster);

      // Get the centroid of the cluster
      clusterCentroids.push_back(centroid);
    }
    //cout<<"\n";

    // Ensure at least 6 clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < Number_of_Filter) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0, 0, 0));
      cluster_vec.push_back(empty_cluster);
    }

    while (clusterCentroids.size() < Number_of_Filter) {
      pcl::PointXYZ centroid;
      centroid.x = 0.0;
      centroid.y = 0.0;
      centroid.z = 0.0;

      clusterCentroids.push_back(centroid);
    }
    
    for(int i=0;i<Number_of_Filter;i++)
    {
      //cout<<clusterCentroids.at(i).x<<" "<<clusterCentroids.at(i).y<<" ";
      Pre_Posi.x = clusterCentroids.at(i).x;
      Pre_Posi.y = clusterCentroids.at(i).y;
      Pre_Position[i].x = clusterCentroids.at(i).x;
      Pre_Position[i].y = clusterCentroids.at(i).y;
      now_Time = ros::Time::now();
      KF[i].statePre.at<float>(0) = clusterCentroids.at(i).x;
      KF[i].statePre.at<float>(1) = 0; //initial v_x;
      KF[i].statePre.at<float>(2) = clusterCentroids.at(i).y;
      KF[i].statePre.at<float>(3) = 0; // initial v_y
      state[i].at<float>(0) = clusterCentroids.at(i).x;
      state[i].at<float>(1) = 0;
      state[i].at<float>(2) = clusterCentroids.at(i).y;
      state[i].at<float>(3) = 0;
      //cout<<KF[i].statePre<<"\n";
    }//cout<<"\n";
    
    firstFrame = false;
    
    for (int i = 0; i < Number_of_Filter; i++) {
      geometry_msgs::Point pt;
      pt.x = clusterCentroids.at(i).x;
      pt.y = clusterCentroids.at(i).y;
      pt.z = clusterCentroids.at(i).z;
      //cout<<pt.x<<" "<<pt.y<<" "<<pt.z<<"\n";
      prevClusterCenters.push_back(pt);
    }//cout<<"\n";
  }

  else {
    // cout<<"ELSE firstFrame="<<firstFrame<<"\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    /* Creating the KdTree from input point cloud*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *input_cloud);

    tree->setInputCloud(input_cloud);

    /* Here we are creating a vector of PointIndices, which contains the actual
     * index information in a vector<int>. The indices of each detected cluster
     * are saved here. Cluster_indices is a vector containing one instance of
     * PointIndices for each detected cluster. Cluster_indices[0] contain all
     * indices of the first cluster in input point cloud.
     */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    //ec.setClusterTolerance(0.3);
    ec.setClusterTolerance(0.22);
    //ec.setMinClusterSize(10);
    ec.setMinClusterSize(50);
    //ec.setMaxClusterSize(600);
    ec.setMaxClusterSize(1000000);

    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    // cout<<"PCL init successfull\n";
    /* Extract the clusters out of pc and save indices in cluster_indices.*/
    ec.extract(cluster_indices);
    // cout<<"PCL extract successfull\n";
    /* To separate each cluster out of the vector<PointIndices> we have to
     * iterate through cluster_indices, create a new PointCloud for each
     * entry and write all points of the current cluster in the PointCloud.
     */
    // pcl::PointXYZ origin (0,0,0);
    // float mindist_this_cluster = 1000;
    // float dist_this_point = 1000;

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    // Vector of cluster pointclouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;

    // Cluster centroids
    std::vector<pcl::PointXYZ> clusterCentroids;

    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      float x = 0.0;
      float y = 0.0;
      float z = 0.0;
      int numPts = 0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

        cloud_cluster->points.push_back(input_cloud->points[*pit]);

        x += input_cloud->points[*pit].x;
        y += input_cloud->points[*pit].y;
        //z += input_cloud->points[*pit].z;
        numPts++;

        // dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
        //                                          origin);
        // mindist_this_cluster = std::min(dist_this_point,
        // mindist_this_cluster);
      }

      pcl::PointXYZ centroid;
      centroid.x = x / numPts;
      centroid.y = y / numPts;
      centroid.z = 0.0;
      //centroid.z = z / numPts;
      //cout<<centroid.x<<' '<<centroid.y<<" "<<centroid.z<<"\n";

      //centroid.z = 0.0;

      cluster_vec.push_back(cloud_cluster);

      // Get the centroid of the cluster
      clusterCentroids.push_back(centroid);
    }
    //cout<<"\n";
    // cout<<"cluster_vec got some clusters\n";

    // Ensure at least 6 clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < Number_of_Filter) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0, 0, 0));
      cluster_vec.push_back(empty_cluster);
    }

    while (clusterCentroids.size() < Number_of_Filter) {
      pcl::PointXYZ centroid;
      centroid.x = 0.0;
      centroid.y = 0.0;
      centroid.z = 0.0;

      clusterCentroids.push_back(centroid);
    }

    std_msgs::Float32MultiArray cc;
    for (int i = 0; i < Number_of_Filter; i++) {
      cc.data.push_back(clusterCentroids.at(i).x);
      cc.data.push_back(clusterCentroids.at(i).y);
      cc.data.push_back(clusterCentroids.at(i).z);
      //cout<<clusterCentroids.at(i).x<<" "<<clusterCentroids.at(i).y<<" "<<clusterCentroids.at(i).z<<"\n";
    }//cout<<"\n";
    // cout<<"6 clusters initialized\n";

    // cc_pos.publish(cc);// Publish cluster mid-points.
    KFT(cc);
    int i = 0;
    bool publishedCluster[Number_of_Filter];
    for (auto it = objID.begin(); it != objID.end(); it++) 
    { // cout<<"Inside the for loop\n";
      if(i>=Number_of_Filter) break;
      publish_cloud(pub_cluster[i], cluster_vec[*it]);
      publishedCluster[i] = true;
      i++;
    }
  }
}

int main(int argc, char **argv) {

  for(int i=0;i<Number_of_Filter;i++)
  {
    KF[i].init(stateDim, measDim, ctrlDim, CV_32F);
    state[i] = cv::Mat(stateDim, 1, CV_32F); 
    measurement[i] = cv::Mat(measDim,1, CV_32F);
    processNoise[i] = cv::Mat(stateDim, 1, CV_32F);
  }
  // ROS init
  ros::init(argc, argv, "kf_tracker");
  ros::NodeHandle nh;

  // Publishers to publish the state of the objects (pos and vel)
  // objState1=nh.advertise<geometry_msgs::Twist> ("obj_1",1);

  cout << "About to setup callback\n";

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe("filtered_cloud", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe("pass_through_cloud",1,cloud_cb);
  // Create a ROS publisher for the output point cloud
  for(int i=0;i<Number_of_Filter;i++)
  {
    string str = "cluster_" + to_string(i);
    pub_cluster[i] = nh.advertise<sensor_msgs::PointCloud2>(str, 1);
  }


  // Subscribe to the clustered pointclouds
  // ros::Subscriber c1=nh.subscribe("ccs",100,KFT);
  objID_pub = nh.advertise<std_msgs::Int32MultiArray>("obj_id", 1);
  /* Point cloud clustering
   */

  // cc_pos=nh.advertise<std_msgs::Float32MultiArray>("ccs",100);//clusterCenter1
  markerPub = nh.advertise<visualization_msgs::MarkerArray>("viz", 1);
  kalmanPub = nh.advertise<visualization_msgs::MarkerArray>("kalman_filter_viz", 1);

  publisher = nh.advertise<kalman_filter::my_msg>("warning_data", 1);
  /* Point cloud clustering
   */

  ros::spin();
}

