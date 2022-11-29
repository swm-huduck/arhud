#include "Swapchain.h"

#include "Utils.h"

Swapchain::Swapchain(VkPhysicalDevice &physicalDevice, VkDevice &device, QueueFamilyIndices &queueFamilyIndices, VkSurfaceKHR &surface)
	:physicalDevice(physicalDevice), device(device), queueFamilyIndices(queueFamilyIndices), surface(surface)
{
}

Swapchain::~Swapchain()
{
}

void Swapchain::create()
{
    SwapChainSupportDetails swapchainSupport = querySwapchainSupport();

    VkSurfaceFormatKHR surfaceFormat = getSwapSurfaceFormat(swapchainSupport.formats);
    VkPresentModeKHR presentMode = getSwapPresentMode(swapchainSupport.presentModes);
    VkExtent2D extent = getSwapExtent(swapchainSupport.capabilities);

	uint32_t imageCount = swapchainSupport.capabilities.minImageCount + 1;
	if (swapchainSupport.capabilities.maxImageCount > 0 && 
		swapchainSupport.capabilities.maxImageCount < imageCount)
    	imageCount = swapchainSupport.capabilities.maxImageCount;

	VkSwapchainCreateInfoKHR createInfo{};
	createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
	createInfo.surface = surface;

	createInfo.minImageCount = imageCount;
	createInfo.imageFormat = surfaceFormat.format;
	createInfo.imageColorSpace = surfaceFormat.colorSpace;
	createInfo.imageExtent = extent;
	createInfo.imageArrayLayers = 1;
	createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

	uint32_t pQueueFamilyIndices[] = {queueFamilyIndices.graphicsFamily.value(), queueFamilyIndices.presentFamily.value()};

	if (queueFamilyIndices.graphicsFamily != queueFamilyIndices.presentFamily) {
		createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
		createInfo.queueFamilyIndexCount = 2;
		createInfo.pQueueFamilyIndices = pQueueFamilyIndices;
	} else {
		createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
		createInfo.queueFamilyIndexCount = 0; // Optional
		createInfo.pQueueFamilyIndices = nullptr; // Optional
	}

	createInfo.preTransform = swapchainSupport.capabilities.currentTransform;
	createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
	createInfo.presentMode = presentMode;
	createInfo.clipped = VK_TRUE;

	createInfo.oldSwapchain = VK_NULL_HANDLE;

	assert(vkCreateSwapchainKHR(device, &createInfo, nullptr, &swapchain) == VK_SUCCESS);

	vkGetSwapchainImagesKHR(device, swapchain, &imageCount, nullptr);
	swapchainImages.resize(imageCount);
	vkGetSwapchainImagesKHR(device, swapchain, &imageCount, swapchainImages.data());
	swapchainImageFormat = surfaceFormat.format;
	swapchainExtent = extent;

	swapchainImageViews.resize(swapchainImages.size());
	for (size_t i = 0; i < swapchainImages.size(); i++) {
		swapchainImageViews[i] = createImageView(device, swapchainImages[i], swapchainImageFormat, VK_IMAGE_ASPECT_COLOR_BIT);
	}
}

void Swapchain::destroy() 
{
	for (auto imageView : swapchainImageViews) vkDestroyImageView(device, imageView, nullptr);
	vkDestroySwapchainKHR(device, swapchain, nullptr);
}

//
SwapChainSupportDetails Swapchain::querySwapchainSupport(){
	SwapChainSupportDetails details;

	vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &details.capabilities);

	uint32_t formatCount;
	vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, nullptr);
	if(formatCount != 0) {
		details.formats.resize(formatCount);
		vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, details.formats.data());
	}

	uint32_t presentModeCount;
	vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount, nullptr);
	if(presentModeCount != 0) {
		details.presentModes.resize(presentModeCount);
		vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount, details.presentModes.data());
	}

	return details;
}

VkSurfaceFormatKHR Swapchain::getSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR> &availableFormats){
	for(const auto &availableFormat : availableFormats){
		if (availableFormat.format == VK_FORMAT_B8G8R8A8_SRGB && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
			return availableFormat;
	}

	return availableFormats[0];
}

VkPresentModeKHR Swapchain::getSwapPresentMode(const std::vector<VkPresentModeKHR> &availablePresentModes){
	for(const auto &availablePresentMode : availablePresentModes){
		if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR)
			return availablePresentMode;
	}

	return VK_PRESENT_MODE_FIFO_KHR;
}

VkExtent2D Swapchain::getSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities){
	if (capabilities.currentExtent.width != UINT32_MAX) {
        return capabilities.currentExtent;
    } else {
        int width=WINDOW_WIDTH, height=WINDOW_HEIGHT;

        VkExtent2D actualExtent = {
            static_cast<uint32_t>(width),
            static_cast<uint32_t>(height)
        };

        actualExtent.width = std::clamp(actualExtent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
        actualExtent.height = std::clamp(actualExtent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);

        return actualExtent;
    }
}