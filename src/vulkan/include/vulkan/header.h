#ifndef __HEADER_H__
#define __HEADER_H__

/** C/C++ HEADER FILES */
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <cstring>
#include <optional>
#include <cassert>
#include <set>
#include <codecvt>
// #include <cstdlib>

#include <chrono>

// #include <typeinfo>
// using namespace std;

// Header files for Singleton
#include <memory>
#include <mutex>

/** VULKAN HEADER FILES */
#include <vulkan/vulkan.h>

/** For GUI */
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

/** For Linear Algebra */
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

/** ROS **/
#include <ros/ros.h>

/** define */
#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720

#define WINDOW_FPS 60

#define NONE_TEXTURE ""

#define MOVE_FRONT 0
#define MOVE_BACK 1
#define MOVE_LEFT 2
#define MOVE_RIGHT 3
#define MOVE_UP 4
#define MOVE_DOWN 5

#define TURN_LEFT 2
#define TURN_RIGHT 3
#define TURN_UP 4
#define TURN_DOWN 5

#define PREFIX_PATH "../../../src/vulkan"

// Shader path
#define SHADER_PATH_VERT "../../../src/vulkan/shaders/vert.spv"
#define SHADER_PATH_FRAG "../../../src/vulkan/shaders/frag.spv"
#define SHADER_PATH_NOTEX_VERT "../../../src/vulkan/shaders/shader_notex_vert.spv"
#define SHADER_PATH_NOTEX_FRAG "../../../src/vulkan/shaders/shader_notex_frag.spv"

// Font path
#define FONT_PATH_NANUMGOTHIC "../../../src/vulkan/fonts/NanumGothic-Regular.ttf"
#define FONT_PATH_NANUMGOTHIC_EB "../../../src/vulkan/fonts/NanumGothic-ExtraBold.ttf"
#define FONT_PATH_DIGITAL "../../../src/vulkan/fonts/DigitalEnterBelow.ttf"
#define FONT_PATH_ANVYL "../../../src/vulkan/fonts/ANVYL.otf"

#ifdef NDEBUG
    const bool enableValidationLayers = false;
#else
    const bool enableValidationLayers = true;
#endif

typedef enum {
    OBJECT_FLAG_USE_TEXTURE = (0x01<<0),
    OBJECT_FLAG_USE_3D = (0x01<<1),
    // FOURTH = (0x01<<3),   // 8
    // FIFTH = (0x01<<4)     // 16
} OBJECT_FLAG;

typedef enum {
    OBJECT_STATE_DRAWABLE = 1,
    OBJECT_STATE_RESERVE_DELETE = 2,
    OBJECT_STATE_DELETABLE = 3,
    OBJECT_STATE_UPDATE_TEXTURE = 4,
    OBJECT_STATE_UPDATE_VERTEX = 5,
    OBJECT_STATE_UPDATE = 6,
    // FIFTH = (0x01<<4)     // 16
} OBJECT_STATE;

const std::vector<const char *> validationLayers = {
    "VK_LAYER_KHRONOS_validation"
};

const std::vector<const char *> deviceExtensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME
};

struct SwapChainSupportDetails {
    VkSurfaceCapabilitiesKHR        capabilities;
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR>   presentModes;
};

struct QueueFamilyIndices {
    std::optional<uint32_t> graphicsFamily;
    std::optional<uint32_t> presentFamily;

    bool isComplete() {
        return graphicsFamily.has_value() && presentFamily.has_value();
    }
};

const int MAX_FRAMES_IN_FLIGHT = 2;

struct VertexLayout {
    int count;
    std::vector<int> attributeTypeSize;
    std::vector<int> attributeCount;
    int size=0;

    void push(int typeSize, int count){
        attributeTypeSize.push_back(typeSize);
        attributeCount.push_back(count);
        size += typeSize*count;
    }

    int getAttributeCount() {
        return attributeTypeSize.size();
    }

    static VkFormat getFormat(int atttributeCount){
        switch(atttributeCount){
        case 1:
            return VK_FORMAT_R32_SFLOAT;
        case 2:
            return VK_FORMAT_R32G32_SFLOAT;
        case 3:
            return VK_FORMAT_R32G32B32_SFLOAT;
        case 4:    
            return VK_FORMAT_R32G32B32A32_SFLOAT;
        }
        return VK_FORMAT_R32G32B32A32_SFLOAT;
    }
};

struct Camera {
    glm::vec3 eye;
    glm::vec3 eye2center;
    glm::vec3 up;
    glm::vec3 defaultEye = {0, 0, 2};

    static Camera& getInstance()
    {
        static Camera instance;
        return instance;
    }

    void move(float x, float y, float z) {
        eye += glm::vec3({-x, 0, -z});
    }
    void move(unsigned int dir){
        glm::vec3 step;
        switch (dir)
        {
        case MOVE_FRONT:
            // step = eye2center;
            step = {0, 1, 0};
            break;
        case MOVE_BACK:
            // step = -eye2center;
            step = {0, -1, 0};
            break;
        case MOVE_LEFT:
            // step = -glm::normalize(glm::cross(eye2center, up));
            step = {-1, 0, 0};
            break;
        case MOVE_RIGHT:
            // step = glm::normalize(glm::cross(eye2center, up));
            step = {1, 0, 0};
            break;
        case MOVE_UP:
            // step = glm::normalize(glm::cross(glm::cross(eye2center, up), eye2center));
            step = {0, 0, 1};
            break;
        case MOVE_DOWN:
            // step = -glm::normalize(glm::cross(glm::cross(eye2center, up), eye2center));
            step = {0, 0, -1};
            break;
        default:
            break;
        }
        eye += (glm::vec3(0.1) * step);
    }

    void turn(unsigned int dir){
        float angle = 2;
        glm::mat4 rotate;
        switch (dir)
        {
        case TURN_LEFT:
            rotate = glm::rotate(glm::identity<glm::mat4>(), glm::radians(angle), glm::cross(glm::cross(eye2center, up), eye2center));
            break;
        case TURN_RIGHT:
            rotate = glm::rotate(glm::identity<glm::mat4>(), glm::radians(-angle), glm::cross(glm::cross(eye2center, up), eye2center));
            break;
        case TURN_UP:
            rotate = glm::rotate(glm::identity<glm::mat4>(), glm::radians(angle), glm::cross(eye2center, up));
            break;
        case TURN_DOWN:
            rotate = glm::rotate(glm::identity<glm::mat4>(), glm::radians(-angle), glm::cross(eye2center, up));
            break;
        default:
            break;
        }
        eye2center = glm::vec3(rotate * glm::vec4({eye2center, 1}));
    }
    
    Camera(Camera const&) = delete;
    Camera(Camera&&) = delete;

private:
    Camera() { 
        eye = defaultEye;
        eye2center = {0, 1, 0};
        up = {0, 0, 1};
    }
};

struct UniformBufferObject {
    glm::mat4 mvp;
    glm::vec4 color;
};

#endif