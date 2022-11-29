Setup
===
### Linux (Ubuntu) using apt repository 
#### Install vulkan driver 
~~~shell
sudo apt install mesa-vulkan-drivers
~~~

#### Install vulkan sdk
The latest version of the SDK can be found [here](https://vulkan.lunarg.com/sdk/home#linux).

##### Ubuntu 18.04 & vulkan sdk 1.2.182
~~~shell
wget -qO - https://packages.lunarg.com/lunarg-signing-key-pub.asc | sudo apt-key add -
sudo wget -qO /etc/apt/sources.list.d/lunarg-vulkan-1.2.182-bionic.list https://packages.lunarg.com/vulkan/1.2.182/lunarg-vulkan-1.2.182-bionic.list
sudo apt update
sudo apt install vulkan-sdk
~~~

##### Uninstall vulkan sdk
~~~shell
sudo apt purge vulkan-sdk
sudo apt autoremove
~~~

##### Check install
You can find `.json` file in this directory `/etc/vulkan/icd.d/` or `/usr/share/vulkan/icd.d`
~~~shell
vkvia
vulkaninfo
vkcube
~~~

#### Install glfw
Install glfw for create window in multiple OS
~~~shell    
sudo apt install libglfw3-dev
~~~

#### Install glm
Install glm for calculate linear algebra
~~~shell
sudo apt install libglm-dev
~~~

#### Trouble shooting
when you can't find lxi library
~~~shell
sudo apt install libxi-dev
~~~

### Linux (Github) using CMake

#### Vulkan Loader
required for install
~~~shell
sudo apt install git build-essential libx11-xcb-dev libxkbcommon-dev libwayland-dev libxrandr-dev libegl1-mesa-dev cmake
~~~

install vulkan loader
~~~shell
git clone https://github.com/KhronosGroup/Vulkan-Loader.git
cd Vulkan-Loader && mkdir build && cd build
../scripts/update_deps.py
cmake -DCMAKE_BUILD_TYPE=Release -DVULKAN_HEADERS_INSTALL_DIR=$(pwd)/Vulkan-Headers/build/install ..
make -j4

export LD_LIBRARY_PATH=$(pwd)/loader

sudo make install
sudo ldconfig
~~~

##### Vulkan Header
~~~shell
cd Vulkan-Header
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install
~~~

#### Vulkan ValidationLayer
~~~shell
git clone https://github.com/KhronosGroup/Vulkan-ValidationLayers.git
cd Vulkan-ValidationLayers
mkdir build
cd build
../scripts/update_deps.py

cd glslang/build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install

cd ../../SPIRV-Headers/build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install

cd ../..
cp robin-hood-hashing/src/include/robin_hood.h /usr/local/include/

make -j4
sudo make install
~~~