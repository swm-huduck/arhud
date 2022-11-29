#ifndef __SWAPCHAIN_H__
#define __SWAPCHAIN_H__
#include "header.h"

class Swapchain
{
private:
	VkPhysicalDevice 			&physicalDevice;
	VkDevice					&device;
	QueueFamilyIndices	    	&queueFamilyIndices;
	VkSurfaceKHR				&surface;
    
    VkSwapchainKHR 				swapchain;
	VkFormat 					swapchainImageFormat;
	VkExtent2D 					swapchainExtent;
	std::vector<VkImage> 		swapchainImages;
	std::vector<VkImageView>	swapchainImageViews;

public:
    Swapchain(VkPhysicalDevice &physicalDevice, VkDevice &device, QueueFamilyIndices &queueFamilyIndices, VkSurfaceKHR &surface);
    ~Swapchain();

    void create();
    void destroy();

    inline VkSwapchainKHR getSwapchain() const { return swapchain; }
    inline VkFormat getSwapchainImageFormat() const { return swapchainImageFormat; }
    inline VkExtent2D getSwapchainExtent() const { return swapchainExtent; }
    inline const std::vector<VkImageView> getSwapchainImageViews() const& { return swapchainImageViews; }
private:
    SwapChainSupportDetails querySwapchainSupport();
    VkSurfaceFormatKHR getSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR> &availableFormats);
    VkPresentModeKHR getSwapPresentMode(const std::vector<VkPresentModeKHR> &availablePresentModes);
    VkExtent2D getSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities);
};

 
#endif // __SWAPCHAIN_H__