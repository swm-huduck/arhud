#include "Renderer.h"
#include "VulkanApplication.h"

#include "Utils.h"

Renderer::Renderer()
	: physicalDevice(VulkanApplication::GetInstance()->physicalDevice), device(VulkanApplication::GetInstance()->device), graphicsQueue(VulkanApplication::GetInstance()->graphicsQueue), presentQueue(VulkanApplication::GetInstance()->presentQueue), queueFamilyIndices(VulkanApplication::GetInstance()->queueFamilyIndices), surface(VulkanApplication::GetInstance()->surface)
{
}

Renderer::Renderer(VkPhysicalDevice &physicalDevice, VkDevice &device, VkQueue &graphicsQueue, VkQueue &presentQueue, QueueFamilyIndices &queueFamilyIndices, VkSurfaceKHR &surface)
	: physicalDevice(physicalDevice), device(device), graphicsQueue(graphicsQueue), presentQueue(presentQueue), queueFamilyIndices(queueFamilyIndices), surface(surface)
{
}

Renderer::~Renderer()
{
}

void Renderer::create()
{
	swapchain = new Swapchain(physicalDevice, device, queueFamilyIndices, surface);
    swapchain->create();

	createDepthResources();
	
	createSyncObjects();

	createCommandPool();

	createRenderPass();
	createFramebuffers();

	createCommandBuffers();
}

void Renderer::sync()
{
	vkWaitForFences(device, 1, &inFlightFences[currentFrame], VK_TRUE, UINT64_MAX);
}

void Renderer::bind(std::vector<UserObject *> &objects) 
{	
	// std::cout<<objects.back()->getState()<<"\n";
	for (size_t i = 0; i < commandBuffers.size(); i++) {
		if(i == imageIndex) continue;
		VkCommandBufferBeginInfo beginInfo{};
		beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;	//

		if (vkBeginCommandBuffer(commandBuffers[i], &beginInfo) != VK_SUCCESS) {
			throw std::runtime_error("failed to begin recording command buffer!");
		}

		VkRenderPassBeginInfo renderPassInfo{};
		renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
		renderPassInfo.renderPass = renderPass;
		renderPassInfo.framebuffer = frameBuffers[i];
		renderPassInfo.renderArea.offset = {0, 0};
		renderPassInfo.renderArea.extent = swapchain->getSwapchainExtent();

		std::array<VkClearValue, 2> clearValues{};
		clearValues[0].color = {0.0f, 0.0f, 0.0f, 1.0f};
		clearValues[1].depthStencil = {1.0f, 0};

		renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
		renderPassInfo.pClearValues = clearValues.data();

		vkCmdBeginRenderPass(commandBuffers[i], &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
            for(auto object : objects){
				object->bindCommandBuffer(commandBuffers[i], i);
            }
		vkCmdEndRenderPass(commandBuffers[i]);

		if (vkEndCommandBuffer(commandBuffers[i]) != VK_SUCCESS) {
			throw std::runtime_error("failed to record command buffer!");
		}

	}

    vkAcquireNextImageKHR(device, swapchain->getSwapchain(), UINT64_MAX, imageAvailableSemaphores[currentFrame], VK_NULL_HANDLE, &imageIndex);

	// Check if a previous frame is using this image (i.e. there is its fence to wait on)
    if (imagesInFlight[imageIndex] != VK_NULL_HANDLE) {
        vkWaitForFences(device, 1, &imagesInFlight[imageIndex], VK_TRUE, UINT64_MAX);
    }
    // Mark the image as now being in use by this frame
    imagesInFlight[imageIndex] = inFlightFences[currentFrame];
}

void Renderer::update(std::vector<UserObject *> &objects) 
{
	// std::cout<<objects.back()->getState()<<"\n";
	for(int i=0; i<objects.size(); i++){
		objects[i]->applyState();
		if(objects[i]->getState() == OBJECT_STATE_DELETABLE) {
			objects[i]->applyState();
			if(dynamic_cast<Warning *>(objects[i])) 
				VulkanApplication::GetInstance()->warning = nullptr;
			delete objects[i];
			objects.erase(objects.begin() + i);
			i--;
		}
		if (objects[i]->getState() == OBJECT_STATE_DRAWABLE || objects[i]->getState() == OBJECT_STATE_UPDATE_TEXTURE) 
			objects[i]->updateUniformBuffer(imageIndex);
	}
	// std::cout<<"\n";
}

void Renderer::draw()
{
	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

	VkSemaphore waitSemaphores[] = {imageAvailableSemaphores[currentFrame]};
	VkPipelineStageFlags waitStages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
	submitInfo.waitSemaphoreCount = 1;
	submitInfo.pWaitSemaphores = waitSemaphores;
	submitInfo.pWaitDstStageMask = waitStages;

	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &commandBuffers[imageIndex];

	VkSemaphore signalSemaphores[] = {renderFinishedSemaphores[currentFrame]};
	submitInfo.signalSemaphoreCount = 1;
	submitInfo.pSignalSemaphores = signalSemaphores;
	
	vkResetFences(device, 1, &inFlightFences[currentFrame]);
	
	if (vkQueueSubmit(graphicsQueue, 1, &submitInfo, inFlightFences[currentFrame]) != VK_SUCCESS) {
		throw std::runtime_error("failed to submit draw command buffer!");
	}

	VkPresentInfoKHR presentInfo{};
	presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

	presentInfo.waitSemaphoreCount = 1;
	presentInfo.pWaitSemaphores = signalSemaphores;

	VkSwapchainKHR swapChains[] = {swapchain->getSwapchain()};
	presentInfo.swapchainCount = 1;
	presentInfo.pSwapchains = swapChains;
	presentInfo.pImageIndices = &imageIndex;

	presentInfo.pResults = nullptr; // Optional

	vkQueuePresentKHR(presentQueue, &presentInfo);

	currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
}

void Renderer::destroy()
{
	for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
		vkDestroySemaphore(device, renderFinishedSemaphores[i], nullptr);
		vkDestroySemaphore(device, imageAvailableSemaphores[i], nullptr);
		vkDestroyFence(device, inFlightFences[i], nullptr);
	}
	
	vkFreeCommandBuffers(device, commandPool, static_cast<uint32_t>(commandBuffers.size()), commandBuffers.data());
	
	vkDestroyCommandPool(device, commandPool, nullptr);

    for (auto framebuffer : frameBuffers) vkDestroyFramebuffer(device, framebuffer, nullptr);
	vkDestroyRenderPass(device, renderPass, nullptr);

    vkDestroyImageView(device, depthImageView, nullptr);
    vkDestroyImage(device, depthImage, nullptr);
    vkFreeMemory(device, depthImageMemory, nullptr);

	// pipe line

	swapchain->destroy();
}

void Renderer::createDepthResources() {
	VkFormat depthFormat = findDepthFormat(physicalDevice);

	createImage(device, physicalDevice, 
                swapchain->getSwapchainExtent().width, swapchain->getSwapchainExtent().height, depthFormat, 
				VK_IMAGE_TILING_OPTIMAL, 
				VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, 
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 
				depthImage, 
				depthImageMemory);
	depthImageView = createImageView(device, depthImage, depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);
}

void Renderer::createSyncObjects() 
{
    imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);
    imagesInFlight.resize(swapchain->getSwapchainImageViews().size(), VK_NULL_HANDLE);

    VkSemaphoreCreateInfo semaphoreInfo{};
    semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        assert (vkCreateSemaphore(device, &semaphoreInfo, nullptr, &imageAvailableSemaphores[i]) == VK_SUCCESS &&
            	vkCreateSemaphore(device, &semaphoreInfo, nullptr, &renderFinishedSemaphores[i]) == VK_SUCCESS &&
            	vkCreateFence(device, &fenceInfo, nullptr, &inFlightFences[i]) == VK_SUCCESS);
    }
}

void Renderer::createRenderPass() 
{
	VkAttachmentDescription colorAttachment{};
	colorAttachment.format = swapchain->getSwapchainImageFormat();
	colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
	colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

	VkAttachmentDescription depthAttachment{};
	depthAttachment.format = findDepthFormat(physicalDevice);
	depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
	depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

	VkAttachmentReference colorAttachmentRef{};
	colorAttachmentRef.attachment = 0;
	colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	
	VkAttachmentReference depthAttachmentRef{};
	depthAttachmentRef.attachment = 1;
	depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

	VkSubpassDescription subpass{};
	subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
	subpass.colorAttachmentCount = 1;
	subpass.pColorAttachments = &colorAttachmentRef;
	subpass.pDepthStencilAttachment = &depthAttachmentRef;

	VkSubpassDependency dependency{};
	dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
	dependency.dstSubpass = 0;
	dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
	dependency.srcAccessMask = 0;
	dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
	dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

	std::array<VkAttachmentDescription, 2> attachments = {colorAttachment, depthAttachment};
	VkRenderPassCreateInfo renderPassInfo{};
	renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
	renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
	renderPassInfo.pAttachments = attachments.data();
	renderPassInfo.subpassCount = 1;
	renderPassInfo.pSubpasses = &subpass;
	renderPassInfo.dependencyCount = 1;
	renderPassInfo.pDependencies = &dependency;

	assert(vkCreateRenderPass(device, &renderPassInfo, nullptr, &renderPass) == VK_SUCCESS);
}

void Renderer::createFramebuffers()
{
	frameBuffers.resize(swapchain->getSwapchainImageViews().size());

	for (size_t i = 0; i < swapchain->getSwapchainImageViews().size(); i++) {
		std::array<VkImageView, 2> attachments = {
			swapchain->getSwapchainImageViews()[i],
    		depthImageView
		};

		VkFramebufferCreateInfo framebufferInfo{};
		framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		framebufferInfo.renderPass = renderPass;
		framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
		framebufferInfo.pAttachments = attachments.data();
		framebufferInfo.width = swapchain->getSwapchainExtent().width;
		framebufferInfo.height = swapchain->getSwapchainExtent().height;
		framebufferInfo.layers = 1;

		assert(vkCreateFramebuffer(device, &framebufferInfo, nullptr, &frameBuffers[i]) == VK_SUCCESS);
	}
}

void Renderer::createCommandPool() {
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;	//
	poolInfo.queueFamilyIndex = queueFamilyIndices.graphicsFamily.value();

	if (vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool) != VK_SUCCESS) {
		throw std::runtime_error("failed to create command pool!");
	}
}

void Renderer::createCommandBuffers() 
{
	commandBuffers.resize(swapchain->getSwapchainImageViews().size());

	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.commandPool = commandPool;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandBufferCount = (uint32_t) commandBuffers.size();

	if (vkAllocateCommandBuffers(device, &allocInfo, commandBuffers.data()) != VK_SUCCESS) {
		throw std::runtime_error("failed to allocate command buffers!");
	}
}
