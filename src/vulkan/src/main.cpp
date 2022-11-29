#define STBTT_STATIC
#include "stb_truetype.h"
#define STBI_STATIC
#include "stb_image.h"

#include "header.h"
#include "VulkanApplication.h"

#include <ros/ros.h>

int main(int argc, char **argv) { 
    VulkanApplication *VkApp = VulkanApplication::GetInstance();

    try {
        ros::init(argc, argv, "vulkan");
        VkApp->init();
        // VkApp->getSupportedLayerList();
        // VkApp->getSupportedInstanceExtensionList();
        // VkApp->getSupportedDevice();
        // VkApp->getSupportedDeviceExtensionList();

        VkApp->loop();
        VkApp->finish();
    } catch (const std::exception& e){
        std::cerr << e.what() << "\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
