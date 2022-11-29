#ifndef __RENDERER_H__
#define __RENDERER_H__
#include "header.h"

#include "Swapchain.h"
#include "Object.h"
#include "UserObject.h"

class Renderer
{
private:
    VkPhysicalDevice      			&physicalDevice;
    VkDevice              			&device;
	VkQueue							&graphicsQueue;
	VkQueue							&presentQueue;
	QueueFamilyIndices				&queueFamilyIndices;
	VkSurfaceKHR					&surface;

	VkImage 						depthImage;
	VkDeviceMemory 					depthImageMemory;
	VkImageView 					depthImageView;

	std::vector<VkSemaphore> 		imageAvailableSemaphores;
	std::vector<VkSemaphore> 		renderFinishedSemaphores;
	std::vector<VkFence> 			inFlightFences;
	std::vector<VkFence> 			imagesInFlight;

	VkRenderPass 					renderPass;
	std::vector<VkFramebuffer> 		frameBuffers;
	VkCommandPool 					commandPool;
	std::vector<VkCommandBuffer> 	commandBuffers;

	Swapchain 						*swapchain = nullptr;

    size_t 							currentFrame = 0;
	uint32_t 						imageIndex;
public:
    Renderer();
    Renderer(VkPhysicalDevice &physicalDevice, VkDevice &device, VkQueue &graphicsQueue, VkQueue &presentQueue, QueueFamilyIndices &queueFamilyIndices, VkSurfaceKHR &surface);
    ~Renderer();

	void create();
	void sync();
	// void bind(std::vector<Object *> &objects);
	// void update(std::vector<Object *> &objects);
	void bind(std::vector<UserObject *> &objects);
	void update(std::vector<UserObject *> &objects);
	void draw();
	void destroy();

	void createDepthResources();
	void createSyncObjects();
    void createRenderPass();
	void createFramebuffers();
	void createCommandPool();
	void createCommandBuffers();

	// void createCommandBuffers(std::vector<Object *> objects);
	// void createCommandBuffers(std::vector<UserObject *> objects);

	inline Swapchain *getSwapchain() const { return swapchain; }
	inline VkCommandPool &getCommandPool() { return commandPool; }
	inline VkRenderPass &getRenderPass() { return renderPass; }

	inline uint32_t getImageIndex() { return imageIndex; }
};


#endif // __RENDERER_H__