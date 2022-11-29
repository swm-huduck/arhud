#ifndef __UTILS_H__
#define __UTILS_H__

#include "header.h"

VkFormat findSupportedFormat(const VkPhysicalDevice &physicalDevice, const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features);

VkFormat findDepthFormat(const VkPhysicalDevice &physicalDevice);

uint32_t findMemoryType(const VkPhysicalDevice &physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties);

void createImage(const VkDevice &device, const VkPhysicalDevice &physicalDevice, 
				 uint32_t width, uint32_t height, VkFormat format, 
				 VkImageTiling tiling, 
				 VkImageUsageFlags usage, 
				 VkMemoryPropertyFlags properties, 
				 VkImage& image, 
				 VkDeviceMemory& imageMemory);

VkImageView createImageView(const VkDevice &device, VkImage image, VkFormat format, VkImageAspectFlags aspectFlags);

VkCommandBuffer beginSingleTimeCommands(const VkDevice &device, const VkCommandPool &commandPool);

void endSingleTimeCommands(const VkDevice &device, const VkCommandPool &commandPool, VkQueue &graphicsQueue, VkCommandBuffer &commandBuffer);

void createBuffer(const VkDevice &device, const VkPhysicalDevice &physicalDevice, 
				  VkDeviceSize size, 
				  VkBufferUsageFlags usage, 
				  VkMemoryPropertyFlags properties, 
				  VkBuffer& buffer, 
				  VkDeviceMemory& bufferMemory);

void copyBuffer(const VkDevice &device, const VkCommandPool &commandPool, VkQueue &graphicsQueue, 
				VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);

#endif // __UTILS_H__