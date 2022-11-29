#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <ctime>
#include <string>
#include <experimental/filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iomanip>

using namespace cv;
using namespace std;

#include "LaneDetector.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/UInt8MultiArray.h>
#include "lane_detect/line_coef.h"
#include "lane_detect/navigation_turn_event.h"
#include "lane_detect/speed.h"
/*
#include<sensor_msgs/PointCloud2.h>
void cloud_cb(const sensor_msgs::PointCloud2 &input)
{
	std::cout<<"Asdasd\n"; 
}
*/

bool start = false;
ros::Subscriber navigationSubscriber;
ros::Subscriber timeSubscriber;
double currTime, currFrame;
VideoCapture cap;

void skip(std::istream & is, size_t n , char delim) { 
    size_t i = 0; 
    while ( i++ < n) is.ignore(100, delim);  
// ignores up to 80 chars but stops ignoring after delim 
// istream stream position var is changed. (we want that) 
} 

double getFrameTimestamp(int frame) {
    ifstream logFile("../../../src/lane_detect/source/test_video_1.log", std::ifstream::in);
    // int diff = frame - currFrame;

    skip(logFile, frame, '\n');
    std::string line;
    std::getline(logFile, line);
    int pos = line.find(']');
    return stod(line.substr(pos+15, 14));
    // streampos p = logFile.tellg();
}

void navigationCallback(const lane_detect::navigation_turn_event& data) {
    // std::cout<<"[NavigationTurnEventCallback]\nevent turn type: "<<data.next_turn_type<<
    //                                         "\nevent distance: "<<data.next_left_distance<<
    //                                         "\nevent position: "<<data.next_relational_position_x<<" "<<data.next_relational_position_y<< 
    //                                         "\nnext turn type: "<<data.next_next_turn_type<<
    //                                         "\nnext distance: "<<data.next_next_left_distance<<"\n";
    start = true;
    cap.open("../../../src/lane_detect/source/test_video_1.mp4");
    cap.set(CAP_PROP_FPS, 29.97);
    cap.set(CAP_PROP_POS_FRAMES, 0);

    navigationSubscriber.shutdown();
}

void timeCallback(const lane_detect::speed::ConstPtr& data) {
    std::cout<<"[Speed (time) callback]\n";

    int hour = stoi(data->time.substr(0, 2));
    int min = stoi(data->time.substr(3, 2));
    int sec = stoi(data->time.substr(6, 2));
    float msec = stof(data->time.substr(9, 3))/1000;

    struct tm t;
    t.tm_year = 2021 - 1900;
    t.tm_mon = 10 - 1;
    t.tm_mday = 23;
    t.tm_hour = hour;
    t.tm_min = min;
    t.tm_sec = sec;

    currTime = (double)mktime(&t) + msec;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "lane_detect");  // node
    ros::NodeHandle nodeHandler;

//ros::Subscriber sub = nodeHandler.subscribe("velodyne_points", 10, cloud_cb);
    ros::Publisher linePublisher = nodeHandler.advertise<lane_detect::line_coef>("line_coef", 1);
    ros::Publisher videoPublisher = nodeHandler.advertise<std_msgs::UInt8MultiArray>("video", 1);

    ros::Rate rate(30);

    navigationSubscriber = nodeHandler.subscribe("navigation_turn_event", 1, navigationCallback);
    timeSubscriber = nodeHandler.subscribe("speed", 1, timeCallback);


    // string source = "solidYellowLeft.mp4";
    // vector<Point2f> srcPoint = {Point(600, 220*2), Point(150, 718), Point(1100, 718), Point(350*2, 220*2)};
    // vector<Point2f> dstPoint = {Point2f(320, 0), Point2f(320, 720), Point2f(960, 720), Point2f(960, 0)};
    // start = true;
	// cap.open("../../../src/velodyne/lane_detect/source/test_video_1.mp4");
    // cap.set(CAP_PROP_FPS, 29.97);
    // cap.set(CAP_PROP_POS_FRAMES, 65);
    // navigationSubscriber.shutdown();

    // 1.188, .420
    // 320 -> 4, (80) 1.188 -> 95.04
    // 320 -> 6, (53) 1.188 -> 63.36
    // 720 -> 30, (24) 0.420 -> 10.8
    string source = "test_video_1.mp4";
    // vector<Point2f> srcPoint = {Point(490., 621.), Point(476., 641.), Point(615., 649.), Point(619., 628.)};
    // vector<Point2f> srcPoint = {Point(499., 627.), Point(485., 647.), Point(626., 649.), Point(629., 629.)};
    // vector<Point2f> srcPoint = {Point(515., 628.), Point(503., 648.), Point(629., 649.), Point(631., 625.)};
    // vector<Point2f> dstPoint = {Point2f(627, 711), Point2f(627, 720), Point2f(653, 720), Point2f(653, 711)};                    // 35
    // vector<Point2f> dstPoint = {Point2f(640-38, 715-15), Point2f(640-38, 715), Point2f(640+38, 715), Point2f(640+38, 715-15)};  // 20
    // vector<Point2f> dstPoint = {Point2f(640-76, 715-30), Point2f(640-76, 715), Point2f(640+76, 715), Point2f(640+76, 715-30)};  // 10
    // vector<Point2f> dstPoint = {Point2f(640-63.36, 715-10.8), Point2f(640-63.36, 715), Point2f(640+63.36, 715), Point2f(640+63.36, 715-10.8)};

    // 320 -> 3.5
    // 720 -> 32 
    vector<Point2f> srcPoint = {Point(535.2, 461.), Point(358., 630.), Point(764., 630.), Point(605.5, 461.)};
    vector<Point2f> dstPoint = {Point2f(640-160, 0), Point2f(640-160, 720), Point2f(640+160, 720), Point2f(640+160, 0)};

    // string source = "IMG_0306.MOV";
    // string source = "IMG_0288.MOV";
    // vector<Point2f> srcPoint = {Point(635, 415), Point(430, 560), Point(925, 560), Point(687, 415)};
    // vector<Point2f> dstPoint = {Point2f(320, 0), Point2f(320, 720), Point2f(960, 720), Point2f(960, 0)};
    
    Mat pMat = getPerspectiveTransform(srcPoint, dstPoint);

    //VideoCapture cap("../../../src/lane_detect/source/"+source);
    // cap.set(CAP_PROP_FPS, 30);

    // VideoCapture cap(4, CAP_V4L2);
    // cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    // cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    // cap.set(CAP_PROP_FPS, 30);
    // cout<<"IMAGE FORMAT : "<<cap.get(CAP_PROP_FOURCC)<<"\n";
    // cout<<"FRAME WIDTH : "<<cap.get(CAP_PROP_FRAME_WIDTH)<<"\n";
    // cout<<"FRAME HEIGHT : "<<cap.get(CAP_PROP_FRAME_HEIGHT)<<"\n";
    // cout<<"FPS : "<<cap.get(CAP_PROP_FPS)<<"\n";
    // cout<<"FRAME COUNT : "<<cap.get(CAP_PROP_FRAME_COUNT)<<"\n";
    // cout<<"BRIGHTNESS : "<<cap.get(CAP_PROP_BRIGHTNESS)<<"\n";
    // cout<<"CONSTRAST : "<<cap.get(CAP_PROP_CONTRAST)<<"\n";
    // cout<<"SATURATION : "<<cap.get(CAP_PROP_SATURATION)<<"\n";
    // cout<<"HUE : "<<cap.get(CAP_PROP_HUE)<<"\n";
    // cout<<"GAIN : "<<cap.get(CAP_PROP_GAIN)<<"\n";
    // cout<<"EXPOSURE : "<<cap.get(CAP_PROP_EXPOSURE)<<"\n";

    cv::Mat cameraMatrix, distCoeffs;
    std::string cameraParameterPath = "../../../src/lane_detect/param/cameraParameter.yml";
    cv::FileStorage storage(cameraParameterPath, cv::FileStorage::READ);
    storage["camera matrix"] >> cameraMatrix;
    storage["distortion coefficients"] >> distCoeffs;
    storage.release();

    Rect undistortRect;
    Mat newCameraMat = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, Size(1280, 720), 1, Size(1280, 720), &undistortRect);
    Mat undistortMap1, undistortMap2;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), newCameraMat, Size(1280, 720), CV_32FC1, undistortMap1, undistortMap2);

    vector<int> lMiddles, rMiddles;
    vector<Point2d> defaultLeft = {Point2d(320-80, 0), Point2d(320-80, 300), Point2d(320-80, 500), Point2d(320-80, 720)};
    vector<Point2d> defaultRight = {Point2d(320+80, 0), Point2d(320+80, 300), Point2d(320+80, 500), Point2d(320+80, 720)};
    bool pauseFlag = false;
    float totalFPS = 0;
    while(ros::ok()){
    // for(int i=0; i<500; i++) {
        // auto startTime = chrono::high_resolution_clock::now();
        if(!start) {
            ros::spinOnce();
            continue;
        }


        Mat src;// = imread("../../../src/lane_detect/source/test_video_1_3.jpg");
        cap >> src;
        if(src.empty()) {
            // throw runtime_error("cannot find src");
            cap.set(CAP_PROP_POS_FRAMES, 0);
            continue;
            // cout<<"asdsa\n";
            // break;
        }
        // Encode, Decode image example            
        // std::vector<uchar> encode;
        // cv::imencode(".bmp", src, encode);

        // Convert encoded image to ROS std_msgs format
        Mat rgb;
        cv::cvtColor(src, rgb, cv::COLOR_BGR2RGB);
        std_msgs::UInt8MultiArray msgArray;
        msgArray.data.clear();
        msgArray.data.resize(1280*720*3);
        memcpy(msgArray.data.data(), rgb.data, msgArray.data.size());
        // std::copy(encode.begin(), encode.end(), msgArray.data.begin());

        // Publish msg
		// std::cout<<"lane detect : "<<msgArray.data.size()<<"\n";
        videoPublisher.publish(msgArray);

        ros::spinOnce();
        /////
        int frame = cap.get(CAP_PROP_POS_FRAMES);
        double ftime = getFrameTimestamp(frame);
        if(abs(currTime - ftime) > 5) {
	        cout<<setprecision(15)<<ftime<<" : "<<currTime<<"\n";
            cap.set(CAP_PROP_POS_FRAMES, 29.97*(currTime-getFrameTimestamp(0)));
            continue;
        }
        if(abs(currTime - ftime) > 0.04) {
	        cout<<setprecision(15)<<ftime<<" : "<<currTime<<"\n";
            if(currTime > ftime) {
	            continue;
            } else {
                ros::Duration(ftime - currTime).sleep();
                currTime += ftime - currTime;
                currTime += 0.0333;
                continue;
	        }
        }
    	currTime += 0.0333;


        // Mat undistort_src;
        // remap(src, undistort_src, undistortMap1, undistortMap2, INTER_LINEAR);
        // Mat roi_undistort_src;
        // roi_undistort_src = undistort_src(undistortRect & Rect(0, 0, src.cols, src.rows));

        // // imshow("../../../src", src);
        // // imshow("undistort", undistort_src);
        // // imshow("undistort roi", roi_undistort_src);

        // resize(roi_undistort_src, src, Size(1280, 720));
        // warpAffine(src, src, getRotationMatrix2D(Point2f(640, 360), 2, 1), Size(1280, 720));
        // imshow("../../../src", src);
        // resize(src, src, Size(1280/2, 720/2));
        int h = src.size[0];
        int w = src.size[1];
        int t = src.type();

        // Mat pMat = getTopViewTransformMatrix(grayscale, cameraMatrix, Size(9, 7), 20);
        Mat transfomredView = LaneDetector::topViewTransform(src, pMat);

        resize(transfomredView, transfomredView, Size(1280/2, 720/2));
        // imshow("tr", transfomredView);

        // resize(transfomredView, transfomredView, Size(1280, 720), 0, 0, INTER_NEAREST);
        Mat topHatFilter = LaneDetector::topHat(transfomredView, Size(11, 7));

        Mat filtered = LaneDetector::applyTophatFilterMask(transfomredView, topHatFilter);
        Mat colorFilter = LaneDetector::getColorFilter(filtered);

        // imshow("colorFilter", colorFilter);

        Mat grayscale;
        cvtColor(transfomredView, grayscale, COLOR_BGR2GRAY);
        Mat sobelFilter = LaneDetector::getSobelFilter(grayscale);

        // imshow("sobelFilter", sobelFilter);

        Mat lane;
        // bitwise_or(sobelFilter, colorFilter, lane);
        bitwise_and(sobelFilter, colorFilter, lane);
        // bitwise_and(colorFilter, colorFilter, lane);
        erode(lane, lane, Mat(Size(1, 3), CV_8U, {1}));
        dilate(lane, lane, Mat(Size(1, 3), CV_8U, {1}));
        
        // imshow("lane", lane);

        // Sobel(lane, lane, -1, 1, 0, 5);

        int margin = 40;
        vector<Point> lPoints, rPoints;
        lMiddles.clear();
        rMiddles.clear();
        LaneDetector::findLinePoints(lane, lMiddles, rMiddles, lPoints, rPoints, margin);

        // imshow("lane", lane);

        Mat leftCoef, rightCoef;
        vector<Point2d> fittedLeft = LaneDetector::polynomialFit(lPoints, 2, leftCoef);
        if (320-80-margin > fittedLeft.back().x  || fittedLeft.back().x > 320-80+margin) 
            fittedLeft = LaneDetector::polynomialFit(defaultLeft, 2, leftCoef);
        vector<Point2d> fittedRight = LaneDetector::polynomialFit(rPoints, 2, rightCoef);
        if (320+80-margin > fittedRight.back().x  || fittedRight.back().x > 320+80+margin) 
            fittedRight = LaneDetector::polynomialFit(defaultRight, 2, rightCoef);
        // cout<<leftCoef<<"\n";
        // if (leftCoef.at<double>(0) >)

        vector<Point2d> fittedSide(fittedLeft);
        fittedSide.insert(fittedSide.end(), fittedRight.begin(), fittedRight.end());
        Mat centerCoef;
        vector<Point2d> fittedCenter  = LaneDetector::polynomialFit(fittedSide, 2, centerCoef);

        vector<double> pubData = centerCoef;
        double p = -360;
        double q = -320;
        double sx = 35.0/360.0;
        double sy = (3.5/2)/320.0;
        double a = pubData[2];
        double b = pubData[1];
        double c = pubData[0];
        pubData = {(c + q + p*(a*p - (b)))*sy,
                    -(b - (2*a*p))/sx*sy,
                    a/(sx*sx)*sy};
        // cout<<pubData[0]<<" "<<pubData[1]<<" "<<pubData[2]<<" "<<"\n";
        lane_detect::line_coef coef;
        coef.line_coef = pubData;
        linePublisher.publish(coef);
        ros::spinOnce();
        
        // cout << centerCoef << "\n";
        // output << "coef "+to_string(count_output) << centerCoef;
        // count_output++;

        // if((i+1)%100 == 0) cout<<((i+1)/5)<<"%.....\n";
        // totalFPS += time;
        rate.sleep();
        
        // auto currentTime = chrono::high_resolution_clock::now();
        // while (1/chrono::duration<float>(currentTime - startTime).count() > 30)
        //     currentTime = chrono::high_resolution_clock::now();
        // float time = chrono::duration<float>(currentTime - startTime).count();
        // cout<<1/time<<"\n";
        // waitKey(0);
    }
    // totalFPS = 500 / totalFPS;
    // cout<<totalFPS<<"\n";
    // output << "total count" << count_output;
    // output.release();

    return 0;
}
