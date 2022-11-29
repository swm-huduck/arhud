#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>

void calculateCameraMatrix(std::vector<std::string> files, cv::Size chessboardSize,                                                 //  Input Parameter
                           cv::Mat &cameraMatrix, cv::Mat &distCoeffs, std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs){   // Output Parameter
    std::vector<cv::Point3f> c3d;                                               // fixed chessboard corner 3d position
    for(int y=0; y<chessboardSize.height; y++){
        for(int x=0; x<chessboardSize.width; x++){
            c3d.push_back({(float)x, (float)y, 0});
        }
    }
    std::vector<cv::Mat> imgs;
    for(int i=0; i<files.size(); i++) imgs.push_back(cv::imread(files[i]));     // Load Images from file 
    
    std::vector<std::vector<cv::Point2f>> corners;                              // chessboard corners position in each images
    std::vector<std::vector<cv::Point3f>> corners3d;                            // fixed chessboard corners 3d position vector
    cv::Size imgSize = imgs[0].size();
    for(int i=0; i<imgs.size(); i++){
        cv::Mat gray;
        cvtColor(imgs[i], gray, cv::COLOR_RGB2GRAY);
        std::vector<cv::Point2f> c;
        bool found = cv::findChessboardCorners(gray, chessboardSize, c);
        corners.push_back(c);
        corners3d.push_back(c3d);
        cv::drawChessboardCorners(imgs[i], chessboardSize, c, found);
        std::string fname = std::filesystem::path(files[i]).filename().string();
        std::string path = "./calib/corner_data/before/" + fname.replace(fname.size()-4, 4, ".jpg") ;
        cv::imwrite(path, imgs[i]);

    }
    for(int i=0; i<corners.size(); i++){
        std::string fname = std::filesystem::path(files[i]).filename().string();
        std::string path = "./calib/corner_data/before/" + fname.replace(fname.size()-4, 4, ".txt") ;
        std::ofstream cf(path.data());
        for(cv::Point2f p : corners[i]){
            cf << p.x << " " << p.y <<"\n";
        }
        cf.close();
    }
    calibrateCamera(corners3d, corners, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs);
}

int main(int argc, char const *argv[]) {
    cv::Size chessboardSize(0, 0);
    std::string path = "./calib";
    std::string outputFile = "./cameraParameter.yml";

    for(int i=1; i<argc; i++){
        if(strcmp(argv[i], "-p") == 0){
            path = argv[++i];
        } else if(strcmp(argv[i], "-w") == 0){
            chessboardSize.width = (int)(argv[++i][0]-'0');
        } else if(strcmp(argv[i], "-h") == 0){
            chessboardSize.height = (int)(argv[++i][0]-'0');
        } else if(strcmp(argv[i], "-o") == 0){
            outputFile = argv[++i];
        }
    }

    if(chessboardSize.width == 0 || chessboardSize.height == 0) throw std::runtime_error("chess board size can't has 0 value");

    std::vector<std::string> files;
    for(const auto &file : std::filesystem::directory_iterator(path)){
        if (std::filesystem::is_directory(file.path().string())) continue;
        files.push_back(file.path().string());
    }
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    calculateCameraMatrix(files, chessboardSize, cameraMatrix, distCoeffs, rvecs, tvecs);
    
    cv::FileStorage output(outputFile, cv::FileStorage::WRITE);
    output << "camera matrix" << cameraMatrix;
    output << "distortion coefficients" << distCoeffs; 
    output.release();

    std::cout << "Complete!\n" << "<Cameara Matrix>\n" << cameraMatrix << "\n<Distortion Coefficients>\n" << distCoeffs << "\n";

    cv::Rect undistortRect;
    cv::Mat newCameraMat = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(1280, 720), 1, cv::Size(1280, 720), &undistortRect);
    cv::Mat undistortMap1, undistortMap2;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), newCameraMat, cv::Size(1280, 720), CV_32FC1, undistortMap1, undistortMap2);

    for(int i=0; i<files.size(); i++){
        cv::Mat img = cv::imread(files[i]);
        cv::Mat undistort_img;
        remap(img, undistort_img, undistortMap1, undistortMap2, cv::INTER_LINEAR);
        undistort_img = undistort_img(undistortRect & cv::Rect(0, 0, img.cols, img.rows));
        cv::Mat gray;
        cvtColor(undistort_img, gray, cv::COLOR_RGB2GRAY);
        std::vector<cv::Point2f> c;
        bool found = cv::findChessboardCorners(gray, chessboardSize, c);
        cv::drawChessboardCorners(undistort_img, chessboardSize, c, found);
        std::string fname = std::filesystem::path(files[i]).filename().string();
        std::string path = "./calib/corner_data/after/" + fname.replace(fname.size()-4, 4, ".jpg") ;
        cv::imwrite(path, undistort_img);
        path = "./calib/corner_data/after/" + fname.replace(fname.size()-4, 4, ".txt") ;
        std::ofstream cf(path.data());
        for(cv::Point2f p : c){
            cf << p.x << " " << p.y <<"\n";
        }
        cf.close();
    }
    return 0;
}
/* // 사용 예시 
int main(){
    cv::Mat cameraMatrix, distCoeffs;
    std::string cameraParameterPath = "./cameraParameter.yml";
    cv::FileStorage storage(cameraParameterPath, cv::FileStorage::READ);
    storage["camera matrix"] >> cameraMatrix;
    storage["distortion coefficients"] >> distCoeffs;
    storage.release();

    cv::Rect undistortRect;
    cv::Mat newCameraMat = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(1280, 720), 1, cv::Size(1280, 720), &undistortRect);
    cv::Mat undistortMap1, undistortMap2;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), newCameraMat, cv::Size(1280, 720), CV_32FC1, undistortMap1, undistortMap2);

    cv::Mat img;
    cv::Mat undistort_img;
    remap(img, undistort_img, undistortMap1, undistortMap2, cv::INTER_LINEAR);
    undistort_img = undistort_img(undistortRect & cv::Rect(0, 0, img.cols, img.rows));
}
*/
