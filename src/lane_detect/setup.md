Setup
===
### Downlad OpenCV 4.1.1
Jetson Ubuntu에서 공식 지원하는 OpenCV 4.1.1 다운로드
+ Downlaod opencv 4.1.1 from [here](https://github.com/opencv/opencv/releases/tag/4.1.1)
+ Downlaod opencv_contrib 4.1.1 from [here](https://github.com/opencv/opencv_contrib/releases/tag/4.1.1)

#### required for install
```shell
# Generic Tools
sudo apt install -y build-essential cmake pkg-config unzip yasm git checkinstall
# Image I/O libs
sudo apt install -y libjpeg-dev libpng-dev libtiff-dev libjasper-dev
# Video/Audio libs
sudo apt install -y libavcodec-dev libavformat-dev libswscale-dev libavresample-dev
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt install -y libxvidcore-dev x264 libx264-dev libfaac-dev libmp3lame-dev
sudo apt install -y libfaac-dev libmp3lame-dev libvorbis-dev
# OpenCore - Adaptive Multi Rate Narrow Band (AMRNB) and Wide Band (AMRWB) speech codec
sudo apt install -y libopencore-amrnb-dev libopencore-amrwb-dev
# Cameras programming interface libs
sudo apt install -y libdc1394-22 libdc1394-22-dev libxine2-dev libv4l-dev v4l-utils
cd /usr/include/linux
sudo ln -s -f ../libv4l1-videodev.h videodev.h
cd ~
# GTK lib for the graphical user functionalites coming from OpenCV highghui module
sudo apt install -y libgtk-3-dev
# Python libraries for python2 and python3:
sudo apt install -y python3-dev python3-pip
sudo -H pip3 install -U pip numpy
# Parallelism library C++ for CPU
sudo apt install -y libtbb-dev
# Optimization libraries for OpenCV
sudo apt install -y libatlas-base-dev gfortran
# Optional libraries:
sudo apt install -y libprotobuf-dev protobuf-compiler
sudo apt install -y libgoogle-glog-dev libgflags-dev
sudo apt install -y libgphoto2-dev libeigen3-dev libhdf5-dev doxygen
```

#### Install
```shell
## OpenCV-4.1.1 폴더로 이동
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D INSTALL_C_EXAMPLES=ON \
-D WITH_TBB=ON \
-D WITH_CUDA=ON \
-D BUILD_opencv_cudacodec=OFF \
-D ENABLE_FAST_MATH=1 \
-D CUDA_FAST_MATH=1 \
-D WITH_CUBLAS=1 \
-D WITH_V4L=ON \
-D WITH_QT=OFF \
-D WITH_OPENGL=ON \
-D WITH_GSTREAMER=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D OPENCV_PC_FILE_NAME=opencv.pc \
-D OPENCV_ENABLE_NONFREE=ON \
-D OPENCV_EXTRA_MODULES_PATH=~/Downloads/opencv_contrib-4.1.1/modules \
-D BUILD_EXAMPLES=ON \
-D BUILD_SHARED_LIBS=ON ..
make -j8
sudo make install

# lib 환경설정
sudo /bin/bash -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
```
#### Xaiver Setting
```shell
sudo ln -s /usr/include/opencv4/opencv2 /usr/include/opencv2

export OPENCV_DIR=/usr/local
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$OPENCV_DIR/lib

sudo apt install libcanberra-gtk-module libcanberra-gtk3-module
```

### Trouble Shooting
#### Cannot found Eigen/core
```shell
# eigen3가 설치되어 있을 경우
sudo ln -s  /usr/include/eigen3/Eigen /usr/include/Eigen

# 설치가 안되어있다면
sudo apt install libeigen3-dev
```