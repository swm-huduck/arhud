#include "Object.h"
#include "VulkanApplication.h"

#include "Utils.h"

Object::Object() 
	: physicalDevice(VulkanApplication::GetInstance()->physicalDevice), device(VulkanApplication::GetInstance()->device), queueFamilyIndices(VulkanApplication::GetInstance()->queueFamilyIndices), graphicsQueue(VulkanApplication::GetInstance()->graphicsQueue), swapchain(*(VulkanApplication::GetInstance()->renderer->getSwapchain())), commandPool(VulkanApplication::GetInstance()->renderer->getCommandPool()), renderPass(VulkanApplication::GetInstance()->renderer->getRenderPass())
{
	swapchainSize = swapchain.getSwapchainImageViews().size();
	setPosition({0, 0, 0});
	setWidth({1, 1});
	setNormal({0, 0, 1});
	setColor({1, 1, 1, 1});
	isBindingBuffers = new bool[swapchainSize];
}

Object::Object(VkPhysicalDevice &physicalDevice, VkDevice &device, QueueFamilyIndices &queueFamilyIndices, VkQueue &graphicsQueue, Swapchain &swapchain, VkCommandPool &commandPool, VkRenderPass &renderPass)
    :physicalDevice(physicalDevice), device(device), queueFamilyIndices(queueFamilyIndices), graphicsQueue(graphicsQueue), swapchain(swapchain), commandPool(commandPool), renderPass(renderPass)
{
	swapchainSize = swapchain.getSwapchainImageViews().size();
	setPosition({0, 0, 0});
	setWidth({1, 1});
	setNormal({0, 0, 1});
	setColor({1, 1, 1, 1});
	isBindingBuffers = new bool[swapchainSize];
}

Object::~Object()
{
}

void Object::create(int objectFlag, std::string texturePath)
{
	if ((objectFlag & OBJECT_FLAG_USE_TEXTURE) == OBJECT_FLAG_USE_TEXTURE) flagUseTexture = true;
	if ((objectFlag & OBJECT_FLAG_USE_3D) == OBJECT_FLAG_USE_3D) flagUse3d = true;

	if(flagUseTexture) assert(texturePath != NONE_TEXTURE);

	if(vertices.empty()) setDefaultVertices();

	pipeline = new Pipeline(device, swapchain, renderPass);
	descriptor = new Descriptor(device, swapchainSize, flagUseTexture);

	descriptor->createDescriptorSetLayout();

	if(flagUseTexture){
		pipeline->createGraphicsPipeline(descriptor->getDescriptorSetLayout(), 
										SHADER_PATH_VERT, SHADER_PATH_FRAG, 
										vertexLayout);
	} else {
		pipeline->createGraphicsPipeline(descriptor->getDescriptorSetLayout(), 
										SHADER_PATH_NOTEX_VERT, SHADER_PATH_NOTEX_FRAG, 
										vertexLayout);
	}

    createVertexBuffers();
    createIndexBuffer();
	createUniformBuffers();

	descriptor->createDescriptorPool();

	if(flagUseTexture){
		texture = new Texture(physicalDevice, device, graphicsQueue, commandPool);
		texture->createImagePixels(texturePath);
		texture->create();
		descriptor->createDescriptorSets(uniformBuffers, texture);
	} else {
		descriptor->createDescriptorSets(uniformBuffers, nullptr);
	}

	state = OBJECT_STATE_DRAWABLE;
}

void Object::destroy() 
{
	descriptor->destroy();

	for (size_t i = 0; i < swapchainSize; i++) {
        vkDestroyBuffer(device, uniformBuffers[i], nullptr);
        vkFreeMemory(device, uniformBuffersMemory[i], nullptr);
    }

    vkDestroyBuffer(device, indexBuffer, nullptr);
    vkFreeMemory(device, indexBufferMemory, nullptr);
	vkDestroyBuffer(device, vertexBuffer, nullptr);
	vkFreeMemory(device, vertexBufferMemory, nullptr);
	
	pipeline->destroy();
    
	if(flagUseTexture) texture->destroy();
}

void Object::createVertexBuffers()
{
	VkDeviceSize bufferSize = sizeof(vertices[0]) * vertices.size();

	VkBuffer stagingBuffer;
	VkDeviceMemory stagingBufferMemory;
    createBuffer(device, physicalDevice, bufferSize, 
				 VK_BUFFER_USAGE_TRANSFER_SRC_BIT, 
				 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, 
				 stagingBuffer, 
				 stagingBufferMemory);

	void* data;
	vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
		memcpy(data, vertices.data(), (size_t) bufferSize);
	vkUnmapMemory(device, stagingBufferMemory);
	
	createBuffer(device, physicalDevice, bufferSize, 
				 VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, 
				 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 
				 vertexBuffer, 
				 vertexBufferMemory);

	copyBuffer(device, commandPool, graphicsQueue,
               stagingBuffer, vertexBuffer, bufferSize);
	vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingBufferMemory, nullptr);
}

void Object::createIndexBuffer() 
{
    VkDeviceSize bufferSize = sizeof(indices[0]) * indices.size();
    // cout<<bufferSize<<"\n";

    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    createBuffer(device, physicalDevice, bufferSize, 
				 VK_BUFFER_USAGE_TRANSFER_SRC_BIT, 
				 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, 
				 stagingBuffer, 
				 stagingBufferMemory);

    void* data;
    vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
    	memcpy(data, indices.data(), (size_t) bufferSize);
    vkUnmapMemory(device, stagingBufferMemory);

    createBuffer(device, physicalDevice, bufferSize, 
				 VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT, 
				 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 
				 indexBuffer, 
				 indexBufferMemory);

    copyBuffer(device, commandPool, graphicsQueue,
               stagingBuffer, indexBuffer, bufferSize);

    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingBufferMemory, nullptr);
}

void Object::createUniformBuffers()
{
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);

    uniformBuffers.resize(swapchainSize);
    uniformBuffersMemory.resize(swapchainSize);

    for (size_t i = 0; i < swapchainSize; i++) {
        createBuffer(device, physicalDevice, bufferSize, 
					 VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, 
					 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, 
					 uniformBuffers[i], 
					 uniformBuffersMemory[i]);
    }
}

void Object::setDefaultVertices()
{
	if(flagUseTexture){
		vertices = {
			0, 0, 0, 	0, 1,
			0, 1, 0, 	0, 0,
			1, 1, 0, 	1, 0,
			1, 0, 0, 	1, 1
		};
		indices = {
			0, 2, 1, 2, 0, 3
		};
		vertexLayout.count = 4;
		vertexLayout.push(sizeof(float), 3);
		vertexLayout.push(sizeof(float), 2);
	} else {
		vertices = {
			0, 0, 0,
			0, 1, 0,
			1, 1, 0,
			1, 0, 0
		};
		indices = {
			0, 2, 1, 2, 0, 3
		};
		vertexLayout.count = 4;
		vertexLayout.push(sizeof(float), 3);
	}
}

void Object::applyState() 
{
	switch(state){
	case OBJECT_STATE_RESERVE_DELETE:
		if(isAllUnbinding()) state = OBJECT_STATE_DELETABLE;
		break;
	case OBJECT_STATE_DELETABLE:
		destroy();
		break;
	case OBJECT_STATE_UPDATE_TEXTURE:
		for (auto tex : oldTexture) {
			tex->destroy();
		}
		oldTexture.clear();
		state = OBJECT_STATE_DRAWABLE;
		break;
	}
}

void Object::updateUniformBuffer(uint32_t currentImage) 
{
	glm::mat4 model, view, proj;
	if(flagUse3d){
		glm::mat4 r;
		if(normal.x == 0 && normal.y == 0) r = glm::mat4(1.0f);
		else {
			r = glm::rotate(glm::mat4(1.0f),
							(float)-atan2(sqrtf(normal.x*normal.x+normal.y*normal.y), normal.z), 
							glm::vec3({normal.y, -normal.x, 0})) *
				glm::rotate(glm::mat4(1.0f),
							(float)atan2(normal.x, -normal.y), 
							glm::vec3({0, 0, 1}));
		}

		model = glm::translate(glm::mat4(1.0f), position) *					
				r *
				glm::translate(glm::mat4(1.0f), {-width.x/2, -width.y/2, 0}) *
				glm::scale(glm::mat4(1.0f), {width, 1});
		// model = glm::identity<glm::mat4>();

		view = glm::lookAt (Camera::getInstance().eye,
							Camera::getInstance().eye + Camera::getInstance().eye2center,
							Camera::getInstance().up);

		proj = glm::perspective(glm::radians(60.0f), 		            //fov-y
								WINDOW_WIDTH / (float) WINDOW_HEIGHT,	//aspect 
								0.001f,						            //z near
								100.0f);					            //z far
		proj[1][1] *= -1;
	} else if(!flagUse3d){
		model = glm::translate(glm::mat4(1.0f), position) *
				glm::scale(glm::mat4(1.0f), {width, 1});

		view = glm::lookAt (glm::vec3(0, 0, -1),		        //eye
							glm::vec3(0, 0, 0), 	            //center
							glm::vec3(0, -1, 0));	            //up axis
							
		proj = glm::ortho((float)0, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT, (float)0, 0.0f, 11.0001f);
		proj[1][1] *= -1;
	}

	updateUniformBuffer(currentImage, proj * view * model);
}

void Object::updateUniformBuffer(uint32_t currentImage, glm::mat4 mvp) 
{
	// glm::mat4 r = glm::rotate<float>(glm::mat4(1.0f),
	// 						 		(float)-35.0 * M_PI / 180.0, 
	// 						 		glm::vec3({1, 0, 0}));
	// glm::mat4 r(1.50943698e+03, -3.67088919e+02, -6.71844275e+05,  9.66039668e+05,
	// 			0.00000000e+00,  7.77597240e+02, -6.20556693e+05,  1.20754958e+06,
	// 			0.00000000e+00, -5.73576436e-01,  4.59680301e+02,  1.50943698e+03,
	// 			0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00);
	UniformBufferObject ubo{};
	ubo.mvp = mvp;
	ubo.color = color;
	void* data;
	vkMapMemory(device, uniformBuffersMemory[currentImage], 0, sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
	vkUnmapMemory(device, uniformBuffersMemory[currentImage]);
}

void Object::updateTexture(std::string path)
{
	if(state == OBJECT_STATE_RESERVE_DELETE || state == OBJECT_STATE_DELETABLE) return;
	state = OBJECT_STATE_UPDATE_TEXTURE;
	oldTexture.push_back(texture);

	texture = new Texture(physicalDevice, device, graphicsQueue, commandPool);
	texture->createImagePixels(path);
	texture->create();

	// std::cout<<texture->getSampler()<<"\n";
	for (size_t i = 0; i < swapchainSize; i++)
		descriptor->updateDescriptorSets(uniformBuffers, texture, i);
}

void Object::updateTexture(std::vector<unsigned char> data, int imageWidth, int imageHeight)
{
	if(state == OBJECT_STATE_RESERVE_DELETE || state == OBJECT_STATE_DELETABLE) return;
	state = OBJECT_STATE_UPDATE_TEXTURE;
	oldTexture.push_back(texture);

	texture = new Texture(physicalDevice, device, graphicsQueue, commandPool);
	texture->createImagePixels(data, imageWidth, imageHeight);
	texture->create();
	
	// std::cout<<texture->getSampler()<<"\n";
	for (size_t i = 0; i < swapchainSize; i++)
		descriptor->updateDescriptorSets(uniformBuffers, texture, i);
}

/////////////////////////////////////////////////////////

Text::Text()
	: Object()
{
	flagUseTexture = true;
	width = {0, 100};
	color = {1, 1, 1, 1};
}

Text::Text(VkPhysicalDevice &physicalDevice, VkDevice &device, QueueFamilyIndices &queueFamilyIndices, VkQueue &graphicsQueue, Swapchain &swapchain, VkCommandPool &commandPool, VkRenderPass &renderPass)
	: Object(physicalDevice, device, queueFamilyIndices, graphicsQueue, swapchain, commandPool, renderPass)
{
	flagUseTexture = true;
	width = {0, 100};
	color = {1, 1, 1, 1};
}

Text::~Text()
{
}

void Text::create(int objectFlag, std::string texturePath, std::string msg)
{
	objectFlag |= OBJECT_FLAG_USE_TEXTURE;
	if ((objectFlag & OBJECT_FLAG_USE_3D) == OBJECT_FLAG_USE_3D) flagUse3d = true;

	assert(texturePath != NONE_TEXTURE);
	assert(!msg.empty());

	VertexLayout vertexLayout;
	vertices = {
		0, 0, 0, 	0, 1,
		0, 1, 0, 	0, 0,
		1, 1, 0, 	1, 0,
		1, 0, 0, 	1, 1
	};
	indices = {
		0, 2, 1, 2, 0, 3
	};
	vertexLayout.count = 4;
	vertexLayout.push(sizeof(float), 3);
	vertexLayout.push(sizeof(float), 2);

	descriptor = new Descriptor(device, swapchainSize, flagUseTexture);
	pipeline = new Pipeline(device, swapchain, renderPass);

	descriptor->createDescriptorSetLayout();

	pipeline->createGraphicsPipeline(descriptor->getDescriptorSetLayout(), 
									SHADER_PATH_VERT, SHADER_PATH_FRAG, 
									vertexLayout);
    createVertexBuffers();
    createIndexBuffer();
	createUniformBuffers();

	descriptor->createDescriptorPool();

	std::wstring wmsg = std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(msg);

	texture = new Texture(physicalDevice, device, graphicsQueue, commandPool);
	texture->createTextPixels(texturePath, wmsg, color, width.x, width.y);
	texture->create();
	descriptor->createDescriptorSets(uniformBuffers, texture);

	textureRatio = width.x/width.y;
	this->texturePath = texturePath;

	state = OBJECT_STATE_DRAWABLE;
}

void Text::destroy() 
{
	descriptor->destroy();

	for (size_t i = 0; i < swapchainSize; i++) {
        vkDestroyBuffer(device, uniformBuffers[i], nullptr);
        vkFreeMemory(device, uniformBuffersMemory[i], nullptr);
    }

    vkDestroyBuffer(device, indexBuffer, nullptr);
    vkFreeMemory(device, indexBufferMemory, nullptr);

	vkDestroyBuffer(device, vertexBuffer, nullptr);
	vkFreeMemory(device, vertexBufferMemory, nullptr);
	
	pipeline->destroy();
    
	if(flagUseTexture) texture->destroy();
	for (auto tex : oldTexture) {
		tex->destroy();
	}
	oldTexture.clear();
}

void Text::applyState() 
{
	switch(state){
	case OBJECT_STATE_RESERVE_DELETE:
		if(isAllUnbinding()) state = OBJECT_STATE_DELETABLE;
		break;
	case OBJECT_STATE_DELETABLE:
		destroy();
		break;
	case OBJECT_STATE_UPDATE_TEXTURE:
		for (auto tex : oldTexture) {
			tex->destroy();
		}
		oldTexture.clear();
		state = OBJECT_STATE_DRAWABLE;
		break;
	}
}

void Text::setHeight(float size)
{
	width.y = size;
	width.x = size*textureRatio;
}

void Text::updateText(std::string msg)
{
	if(state == OBJECT_STATE_RESERVE_DELETE || state == OBJECT_STATE_DELETABLE) return;
	state = OBJECT_STATE_UPDATE_TEXTURE;
	oldTexture.push_back(texture);

	std::wstring wmsg = std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(msg);
	texture = new Texture(physicalDevice, device, graphicsQueue, commandPool);
	texture->createTextPixels(this->texturePath, wmsg, color, width.x, width.y);
	texture->create();
	// std::cout<<texture->getSampler()<<"\n";
	for (size_t i = 0; i < swapchainSize; i++)
		descriptor->updateDescriptorSets(uniformBuffers, texture, i);
}

/////////////////////////////////////////////////////////
