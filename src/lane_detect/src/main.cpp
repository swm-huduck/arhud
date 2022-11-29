#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <string>
#include <experimental/filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

#include "LaneDetector.h"

#include <ros/ros.h>
#include "lane_detect/line_coef.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "lane_detect");  // node
    ros::NodeHandle nodeHandler;

    ros::Publisher linePublisher = nodeHandler.advertise<lane_detect::line_coef>("line_coef", 1);

    ros::Rate rate(30);
    
    // string source = "solidYellowLeft.mp4";
    // vector<Point2f> srcPoint = {Point(600, 220*2), Point(150, 718), Point(1100, 718), Point(350*2, 220*2)};
    // vector<Point2f> dstPoint = {Point2f(320, 0), Point2f(320, 720), Point2f(960, 720), Point2f(960, 0)};

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

    VideoCapture cap("../../../src/lane_detect/source/"+source);

    // VideoCapture cap(4, CAP_V4L2);
    // cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    // cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CAP_PROP_FPS, 29.99);
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
    while(ros::ok()){
        // auto startTime = chrono::high_resolution_clock::now();

        int key = waitKeyEx(1);
        if(key=='q') break;
        else if (key == 'p') pauseFlag = !pauseFlag;
        else if (key == 65361) {  // left key
            cap.set(CAP_PROP_POS_FRAMES, (int)(cap.get(CAP_PROP_POS_FRAMES) - 30 * 10));
            continue;
        }
        else if (key == 65363) {  // right key
            cap.set(CAP_PROP_POS_FRAMES, (int)(cap.get(CAP_PROP_POS_FRAMES) + 30 * 10));
            continue;
        }

        if(pauseFlag) continue;

        Mat src;// = imread("../../../src/lane_detect/source/test_video_1_3.jpg");
        cap >> src;
        if(src.empty()) {
            // throw runtime_error("cannot find src");
            cap.set(CAP_PROP_POS_FRAMES, 0);
            continue;
            // cout<<"asdsa\n";
            // break;
        }
        Mat undistort_src;
        remap(src, undistort_src, undistortMap1, undistortMap2, INTER_LINEAR);
        Mat roi_undistort_src;
        roi_undistort_src = undistort_src(undistortRect & Rect(0, 0, src.cols, src.rows));

        // imshow("../../../src", src);
        // imshow("undistort", undistort_src);
        // imshow("undistort roi", roi_undistort_src);

        resize(roi_undistort_src, src, Size(1280, 720));
        warpAffine(src, src, getRotationMatrix2D(Point2f(640, 360), 2, 1), Size(1280, 720));
        imshow("../../../src", src);
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

        imshow("colorFilter", colorFilter);

        Mat grayscale;
        cvtColor(transfomredView, grayscale, COLOR_BGR2GRAY);
        Mat sobelFilter = LaneDetector::getSobelFilter(grayscale);

        imshow("sobelFilter", sobelFilter);

        Mat lane;
        // bitwise_or(sobelFilter, colorFilter, lane);
        bitwise_and(sobelFilter, colorFilter, lane);
        erode(lane, lane, Mat(Size(1, 3), CV_8U, {1}));
        dilate(lane, lane, Mat(Size(1, 3), CV_8U, {1}));
        
        imshow("lane", lane);

        // Sobel(lane, lane, -1, 1, 0, 5);

        int margin = 40;
        vector<Point> lPoints, rPoints;
        lMiddles.clear();
        rMiddles.clear();
        LaneDetector::findLinePoints(lane, lMiddles, rMiddles, lPoints, rPoints, margin);

        imshow("lane", lane);

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
        
        LaneDetector::drawLine(transfomredView, fittedLeft, fittedRight);
        imshow("tr", transfomredView);
        // cout << centerCoef << "\n";
        // output << "coef "+to_string(count_output) << centerCoef;
        // count_output++;

        rate.sleep();
        
        // auto currentTime = chrono::high_resolution_clock::now();
        // while (1/chrono::duration<float>(currentTime - startTime).count() > 30)
        //     currentTime = chrono::high_resolution_clock::now();
        // float time = chrono::duration<float>(currentTime - startTime).count();
        // cout<<1/time<<"\n";
        // waitKey(0);
    }
    // output << "total count" << count_output;
    // output.release();

    return 0;
}