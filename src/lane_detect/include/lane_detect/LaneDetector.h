#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

namespace LaneDetector{
    Mat getTopViewTransformMatrix(Mat src, Mat cameraMatrix, Size chessboardSize, float pixelPerBlock){
        int w=src.cols, h=src.rows;

        vector<Point2f> corners;
        bool foundFlag = findChessboardCorners(src, chessboardSize, corners, CALIB_CB_SYMMETRIC_GRID | CALIB_CB_CLUSTERING);
        if(!foundFlag) return (Mat_<double>(3,3) <<
                    1, 0, 0,
                    0, 1, 0,
                    0, 0, 1); //throw runtime_error("can't found chessboard");
        
        vector<Point2d> cornersAfterTransform;
        for(int y=0; y<chessboardSize.height; y++){
            for(int x=0; x<chessboardSize.width; x++){
                cornersAfterTransform.push_back({w/2 - (chessboardSize.width-1)*pixelPerBlock/2 + (double)x*pixelPerBlock, 
                                                h/2 - (chessboardSize.height-1)*pixelPerBlock/2 + (double)y*pixelPerBlock});
            }
        }
        
        vector<Point3f> corners3d;
        for(int y=0; y<chessboardSize.height; y++){
            for(int x=0; x<chessboardSize.width; x++){
                corners3d.push_back({w/2 - (chessboardSize.width-1)*pixelPerBlock/2 + (float)x*pixelPerBlock, 
                                    h/2 - (chessboardSize.height-1)*pixelPerBlock/2 + (float)y*pixelPerBlock,
                                    0});
            }
        }

        Mat homographyMat = findHomography(corners, cornersAfterTransform, RANSAC);
        return homographyMat;

    }
    Mat topViewTransform(const Mat &src, const Mat &perspectiveTransformMatrix){
        Mat transfomredView;
        warpPerspective(src, transfomredView, perspectiveTransformMatrix, src.size());
        // warpAffine(transfomredView, transfomredView, getRotationMatrix2D(Point2f(640, 360), -10, 1), Size(1280, 720));
        // warpPerspective(src, transfomredView, perspectiveTransformMatrix, Size(1150, 5000));
        return transfomredView;
    }
    Mat getColorFilter(const Mat &src){
        Mat hls;
        cvtColor(src, hls, COLOR_RGB2HLS);
        // cv::imwrite("../../../src/lane_detect/source/test_images/hls.jpg", hls);

        // vector<Mat> hlsSplit;
        // split(hls, hlsSplit);
        // // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        // // clahe->setClipLimit(2);
        // // clahe->apply(hlsSplit[1], hlsSplit[1]);
        // Mat hlsHE;
        // merge(hlsSplit, hlsHE);

        Mat yellowFilter, whiteFilter;
        // inRange(hls, Scalar(30, 64, 0), Scalar(120, 255, 255), yellowFilter);  // Find Yellow Line
        // inRange(hls, Scalar(0, 128, 0), Scalar(360, 255, 255), whiteFilter);      // Find White Line
        inRange(hls, Scalar(0, 0, 64), Scalar(180, 255, 255), yellowFilter);  // Find Yellow Line
        inRange(hls, Scalar(0, 205, 0), Scalar(360, 255, 255), whiteFilter);      // Find White Line
        Mat colorFilter = yellowFilter + whiteFilter;
        // imshow("yell", yellowFilter);
        // cv::imwrite("../../../src/lane_detect/source/test_images/yellow.jpg", yellowFilter);
        // imshow("white", whiteFilter);
        // cv::imwrite("../../../src/lane_detect/source/test_images/white.jpg", whiteFilter); 
        return colorFilter;
    }
    Mat getSobelFilter(const Mat &src){
        // cv::imwrite("../../../src/lane_detect/source/test_images/gray.jpg", src);
        Mat blurred;
        GaussianBlur(src, blurred, Size(3, 3), 0, 0, BORDER_DEFAULT);
        // cv::imwrite("../../../src/lane_detect/source/test_images/blurred.jpg", blurred);

        Mat dx, dy;
        Sobel(blurred, dx, -1, 1, 0, 3);
        Sobel(blurred, dy, -1, 0, 1, 3);
        //// imshow("dx", dx);
        //// imshow("dy", dy);
        Mat phaseEdge, phaseFilter;
        phase(Mat_<float>(dy), Mat_<float>(dx), phaseEdge, true);
        inRange(phaseEdge, 70, 100, phaseFilter);  // check degree of image derivative
        // Mat magEdge;
        // magnitude(Mat_<float>(dx), Mat_<float>(dy), magEdge);
        Mat absX, magFilter;
        absX = cv::abs(dx);
        inRange(absX, 40, 255, magFilter);                                          // threshold dx value

        Mat sobelFilter = magFilter;
        bitwise_and(magFilter, phaseFilter, sobelFilter);

        Mat imgMask;
        threshold(blurred, imgMask, 1, 255, CV_8U);
        erode(imgMask, imgMask, Mat(Size(30, 30), CV_8U, {1}));     // transform 으로 인해 생긴 경계 제거
        bitwise_and(sobelFilter, imgMask, sobelFilter);

        dilate(sobelFilter, sobelFilter, Mat(Size(3, 5), CV_8U, {1}));
        // cv::imwrite("../../../src/lane_detect/source/test_images/sobel.jpg", sobelFilter);

        Mat tr = (Mat_<float>(3, 3) << 1, 0, 3, 0, 1, 0, 0, 0, 1);
        warpPerspective(sobelFilter, sobelFilter, tr, sobelFilter.size());
        return sobelFilter;
    }
    Mat topHat(const Mat &src, const Size &kernelSize){
        Mat dst;
        morphologyEx(src, dst, MORPH_TOPHAT, Mat(kernelSize, CV_8U, {1}));
        return dst;
    }
    Mat applyTophatFilterMask(const Mat &src, const Mat &tophatImage) {
        Mat dst;
        Mat mask;
        cvtColor(tophatImage, mask, COLOR_BGR2GRAY);
        // threshold(mask, mask, 10, 255, THRESH_BINARY);
        threshold(mask, mask, 10, 255, ADAPTIVE_THRESH_GAUSSIAN_C);
        mask = 255-mask;
        // threshold(mask, mask, 10, 255, ADAPTIVE_THRESH_MEAN_C);
        src.copyTo(dst, mask);
        //// imshow("masks", dst);
        return dst;
    }
    Mat histogramEqulize(const Mat &src) {
        Mat dst;
        Mat yrb;
        cvtColor(src, yrb, COLOR_BGR2HLS);
        // cv::equalizeHist(dst, dst);
        // cv::threshold(dst, dst, 20, 255, THRESH_BINARY);
        vector<Mat> split;
        cv::split(yrb, split);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(2);
        clahe->apply(split[1], split[1]);
        merge(split, dst);
        cvtColor(dst, dst, COLOR_HLS2BGR);
        // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        // clahe->setClipLimit(2);
        // clahe->apply(dst, dst);
        //// imshow("asdsassdsad", dst);
        return dst;
    }
    void removeOutBound(Mat &src){
        // int ww = 320;
        // for(int y=0; y<src.rows; y++){
        //     for(int x=0; x<src.cols; x++){
        //         // cout<<x<<" "<<y<<"\n";
        //         if (x < ww) {
        //             if(src.at<uchar>(y, x) > 127 && src.at<uchar>(y, x+ww) > 127) continue;
        //         } else if (x >= src.cols-ww) {
        //             if(src.at<uchar>(y, x) > 127 && src.at<uchar>(y, x-ww) > 127) continue;
        //         } else {
        //             if(src.at<uchar>(y, x) > 127 && src.at<uchar>(y, x+ww) > 127) continue;
        //             if(src.at<uchar>(y, x) > 127 && src.at<uchar>(y, x-ww) > 127) continue;
        //         }
        //         src.at<uchar>(y, x) =  0;
        //     }
        // }
    }
    void findLinePoints(Mat &src, const vector<int> &leftMiddles, const vector<int> &rightMiddles, vector<Point> &leftPoints, vector<Point> &rightPoints, const int &margin){
        int meanX = src.cols/2;
        // int lMiddle = src.cols*3/8, rMiddle=src.cols*5/8;
        int lMiddle = src.cols*1/4, rMiddle=src.cols*3/4;
        int level = src.rows/margin;
        int l_boxCount=0, r_boxCount=0;
        int ln_boxCount=0, rn_boxCount=0;
        bool l_noInitialFlag=false, r_noInitialFlag=false;
        for(int yh=level-1; yh>=0; yh--){
            vector<Point> linePoints;
            findNonZero(src(Rect(0, yh*margin, src.cols, margin)), linePoints);     // y margin area
            int lSum=0, rSum=0;
            int lCount=0, rCount=0;
            for(int i=0; i<linePoints.size(); i++){
                if(leftMiddles.empty()){
                    if(yh==level-1){
                        if(linePoints[i].x < meanX){
                            lSum += linePoints[i].x;
                            lCount++;
                        } else {
                            rSum += linePoints[i].x;
                            rCount++;
                        }
                    } else {
                        if (lMiddle-1.5*margin/2 <= linePoints[i].x  && linePoints[i].x <= lMiddle+1.5*margin/2) {
                            lSum += linePoints[i].x;
                            lCount++;
                        } else if (rMiddle-1.5*margin/2 <= linePoints[i].x  && linePoints[i].x <= rMiddle+1.5*margin/2){
                            rSum += linePoints[i].x;
                            rCount++;
                        }
                    }
                } else {
                    if (leftMiddles[yh]-margin/2 <= linePoints[i].x  && linePoints[i].x <= leftMiddles[yh]+margin/2) {
                        lSum += linePoints[i].x;
                        lCount++;
                    } else if (rightMiddles[yh]-margin/2 <= linePoints[i].x  && linePoints[i].x <= rightMiddles[yh]+margin/2){
                        rSum += linePoints[i].x;
                        rCount++;
                    }
                }
                /* if(yh==level-1){
                    if(linePoints[i].x < meanX){
                        lSum += linePoints[i].x;
                        lCount++;
                    } else {
                        rSum += linePoints[i].x;
                        rCount++;
                    }
                } else {
                    if (lMiddle-margin <= linePoints[i].x  && linePoints[i].x <= lMiddle+margin) {
                        lSum += linePoints[i].x;
                        lCount++;
                    } else if (rMiddle-margin <= linePoints[i].x  && linePoints[i].x <= rMiddle+margin){
                        rSum += linePoints[i].x;
                        rCount++;
                    }
                } */
                /* if (lMiddle-margin <= linePoints[i].x  && linePoints[i].x <= lMiddle+margin) {
                    lSum += linePoints[i].x;
                    lCount++;
                } else if (rMiddle-margin <= linePoints[i].x  && linePoints[i].x <= rMiddle+margin){
                    rSum += linePoints[i].x;
                    rCount++;
                } */
            }
            Point lp1, lp2, rp1, rp2;
            lMiddle = (lCount>30)? lSum/lCount : (leftMiddles.empty())? lMiddle : leftMiddles[yh];
            if (30 < lCount && lCount < margin*margin){
                // if(abs(nowLMiddle-lMiddle)<margin || (lMiddle==src.cols/4 &&   lCount>30)) lMiddle = nowLMiddle;
                lp1 = Point(lMiddle-margin/2, yh*margin);
                lp2 = Point(lMiddle+margin/2, (yh+1)*margin);
                rectangle(src, Rect(lp1, lp2), (255, 255, 255));       // draw left point box area
                if (!l_noInitialFlag) l_boxCount++;
                if(ln_boxCount > 2) l_noInitialFlag = true;
                ln_boxCount = 0;
            } else ln_boxCount++;
            // if (!l_noInitialFlag && level - yh > 3) l_noInitialFlag = true;
            
            rMiddle = (rCount>30)? rSum/rCount : (rightMiddles.empty())? rMiddle : rightMiddles[yh];
            if (30 < rCount && rCount < margin*margin){
                // if(abs(nowRMiddle-rMiddle)<margin || (rMiddle==src.cols/4 && rCount>100)) rMiddle = nowRMiddle;
                rp1 = Point(rMiddle-margin/2, yh*margin);
                rp2 = Point(rMiddle+margin/2, (yh+1)*margin);
                rectangle(src, Rect(rp1, rp2), (255, 255, 255));       // draw right point box area
                if (!r_noInitialFlag) r_boxCount++;
                if(rn_boxCount > 3) r_noInitialFlag = true;
                rn_boxCount = 0;
            } else rn_boxCount++;
            // if (!r_noInitialFlag && level - yh > 3) r_noInitialFlag = true;

            // if (lMiddle < 50 || rMiddle > src.cols-50) break;
            // if (abs(meanX - (lMiddle + rMiddle)/2) < margin) meanX = (lMiddle + rMiddle)/2;
            // ww = (rMiddle-lMiddle)/2;
            meanX = (lMiddle + rMiddle)/2;
            // circle(src, Point(meanX, yh*margin+margin/2), 3, Scalar(255));

            for(int i=0; i<linePoints.size(); i++){                 // save points
                linePoints[i].y += yh*margin;
                if (lp1.x < linePoints[i].x && linePoints[i].x < lp2.x && lCount>30) {
                    leftPoints.push_back(linePoints[i]);
                } else if(rp1.x < linePoints[i].x && linePoints[i].x < rp2.x && rCount>30) {
                    rightPoints.push_back(linePoints[i]);
                }
            }
        }
        if ((l_boxCount < 4 || l_noInitialFlag) && (r_boxCount >= l_boxCount && !r_noInitialFlag)) {
            leftPoints.clear();
            for(int i=0; i<rightPoints.size(); i++) leftPoints.push_back(rightPoints[i] - Point(320, 0));
        }
        // if (r_boxCount < 3) rightPoints.clear();
    }
    
    vector<Point2d> polynomialFit(const vector<Point> &points, int order, Mat &K) {
        Mat U(points.size(), (order + 1), CV_64F);
        Mat X(points.size(), 1, CV_64F);
        
        int min=1281, max=-1;
        for (int i = 0; i < U.rows; i++) {
            for (int j = 0; j < U.cols; j++) {
                U.at<double>(i, j) = pow(points[i].y, j);
                min = (points[i].y < min)? points[i].y : min;
                max = (points[i].y > max)? points[i].y : max;
            }
        }
        
        for (int i = 0; i < X.rows; i++) {
            X.at<double>(i, 0) = points[i].x;
        }

        K = Mat((order + 1), 1, CV_64F);
        if(U.data != NULL) {
            K = (U.t() * U).inv() * U.t() * X;
        }

        vector<Point2d> output;
        for (int y=0; y<360; y++) {
            Point2d point(0, y);
            for (int x=0; x<order+1; x++) {
                point.x += K.at<double>(x, 0) * pow(y, x);
            }
            output.push_back(point);
        }
        return output;
    }
    vector<Point2d> polynomialFit(const vector<Point2d> &points, int order, Mat &K) {
        Mat U(points.size(), (order + 1), CV_64F);
        Mat X(points.size(), 1, CV_64F);
        
        int min=1281, max=-1;
        for (int i = 0; i < U.rows; i++) {
            for (int j = 0; j < U.cols; j++) {
                U.at<double>(i, j) = pow(points[i].y, j);
                min = (points[i].y < min)? points[i].y : min;
                max = (points[i].y > max)? points[i].y : max;
            }
        }
        
        for (int i = 0; i < X.rows; i++) {
            X.at<double>(i, 0) = points[i].x;
        }

        K = Mat((order + 1), 1, CV_64F);
        if(U.data != NULL) {
            K = (U.t() * U).inv() * U.t() * X;
        }

        vector<Point2d> output;
        for (int y=0; y<360; y++) {
            Point2d point(0, y);
            for (int x=0; x<order+1; x++) {
                point.x += K.at<double>(x, 0) * pow(y, x);
            }
            output.push_back(point);
        }
        return output;
    }
    void drawLine(Mat &src, const vector<Point2d> &fittedLeftPoints, const vector<Point2d> &fittedRightPoints, const Scalar &color = (255), const int &thickness = 1){
        vector<Point> lpts, rpts;
        for(int i=0; i<fittedLeftPoints.size(); i++) lpts.push_back((Point)fittedLeftPoints[i]);
        for(int i=0; i<fittedRightPoints.size(); i++) rpts.push_back((Point)fittedRightPoints[i]);

        polylines(src, lpts, false, color, thickness);
        polylines(src, rpts, false, color, thickness);
    }
    void drawLine(Mat &src, const vector<Point2d> &fittedPoints, const Scalar &color = (255), const int &thickness = 1){
        vector<Point> lpts, rpts;
        for(int i=0; i<fittedPoints.size(); i++) lpts.push_back((Point)fittedPoints[i]);

        polylines(src, lpts, false, color, thickness);
    }
    void drawLineArea(Mat &src, const vector<Point2d> &fittedLeftPoints, const vector<Point2d> &fittedRightPoints, const Scalar &color = (255)){
        vector<Point> lpts, rpts;
        for(int i=0; i<fittedLeftPoints.size(); i++) lpts.push_back((Point)fittedLeftPoints[i]);
        for(int i=0; i<fittedRightPoints.size(); i++) rpts.push_back((Point)fittedRightPoints[i]);
        vector<Point> points(lpts);
        points.insert(points.end(), rpts.rbegin(), rpts.rend());

        vector<vector<Point>> pts = {points};
        
        Mat groundLane = Mat::zeros(src.size(), CV_8UC3);
        fillPoly(groundLane, pts, color);

        addWeighted(src, 1, groundLane, 0.3, 0, src);
    }
    vector<Point2d> getInversePerspectiveTransformedPoints(const vector<Point2d> &points, const Mat &perspectiveTransformMatrix, const float &scale = 1){
        vector<Point2d> ret;
        if (scale != 1){
            Mat scaleMat = (Mat_<float>(3, 3) << scale, 0, 0, 0, scale, 0, 0, 0, 1);
            perspectiveTransform(points, points, scaleMat);
        }
        perspectiveTransform(points, ret, perspectiveTransformMatrix.inv());
        return ret;
    }

}