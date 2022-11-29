#include "VulkanApplication.h"

#include "Utils.h"

std::unique_ptr<VulkanApplication> VulkanApplication::ApplicationInstance;
std::once_flag VulkanApplication::onlyOnce;

// std::vector<Object*> VulkanApplication::objects;
std::vector<UserObject*> VulkanApplication::objects;
NotificationManager 	*VulkanApplication::notiManager;
Speed					*VulkanApplication::speed;
Polyline				*VulkanApplication::lane;
Navigation				*VulkanApplication::navi;
BackGround				*VulkanApplication::bg;
Warning					*VulkanApplication::warning;
float VulkanApplication::time;

// 생성자
VulkanApplication::VulkanApplication() {}
VulkanApplication::~VulkanApplication() {}

// Returns the Single ton object of VulkanApplication
VulkanApplication* VulkanApplication::GetInstance(){
    std::call_once(onlyOnce, [](){ApplicationInstance.reset(new VulkanApplication()); });
    return ApplicationInstance.get();
}

/** 
 * public function 
 */
void VulkanApplication::init() 
{
	/*  ros setting */
	nodeHandle = new ros::NodeHandle;
	subscriber = nodeHandle->subscribe("line_coef__", 1, lineCallback);
	callSubscriber = nodeHandle->subscribe("call", 1, callCallback);
	smsSubscriber = nodeHandle->subscribe("sms", 1, smsCallback);
	kakaoSubscriber = nodeHandle->subscribe("kakao_talk", 1, kakaoCallback);
	speedSubscriber = nodeHandle->subscribe("speed", 1, speedCallback);
	navigationSubscriber = nodeHandle->subscribe("navigation_turn_event", 1, navigationCallback);
	warningSubscriber = nodeHandle->subscribe("warning_data", 1, warningCallback);
	eyeSubscriber = nodeHandle->subscribe("eye_position", 1, EyePositionCallback);
	rate = new ros::Rate(WINDOW_FPS);

	// bgSubscriber = nodeHandle->subscribe("video", 1, imageCallback);
	bgSubscriber = nodeHandle->subscribe("Type_Converted_Image", 1, imageCallback);

	/** init Window */
	glfwInit();
	// OpenGL context로 생성 X
	glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
	// resize 금지
	glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
	// window 생성 (width, height, title, monitor option, relevent to OpenGL)
	// window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Vulkan Application", glfwGetPrimaryMonitor(), nullptr);
	window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Vulkan Application", nullptr, nullptr);

	glfwSetWindowUserPointer(window, this);

	/* set keyboard callback */
	glfwSetKeyCallback(window, key_callback);
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GLFW_TRUE);

	/** init vulkan */
	createInstance();
	setupDebugMessenger();

	createSurface();
	getPhysicalDevice();
	createLogicalDevice();

	// renderer = new Renderer(physicalDevice, device, graphicsQueue, presentQueue, queueFamilyIndices, surface);
	renderer = new Renderer();
	renderer->create();

	notiManager = new NotificationManager;

	lane = new Polyline;
	std::vector<double> K;
	K.push_back(0);
	K.push_back(0);
	K.push_back(0);
	// K.push_back(0.11686732309238017);
	// K.push_back(-0.004611065445485643);
	// K.push_back(0.0003924677808668353);
	// K.push_back(0.1476621317093852);
	// K.push_back(-0.009574478691038517);
	// K.push_back(0.0005761585884746678);
	lane->create(K, 2, 0.3f, 100, 0.1f);
	// lane->setColor({1, 1, 1, 1});

	// Notification *noti = new Notification();
	// noti->create("../../../src/vulkan/textures/msgg.png", "정용훈 : 밥 먹었어?", 5);
	// objects.push_back(noti);

	speed = new Speed;
	speed->create(0);

	// Direction2D *direc2d = new Direction2D;
	// direc2d->create(0, 300);
	// direc2d->setPosition({WINDOW_WIDTH/2 + 150, 40});
	// objects.push_back(direc2d);

	// Direction2D *direc2d2 = new Direction2D;
	// direc2d2->create(1, 3340);
	// direc2d2->setSize(70);
	// direc2d2->setPosition({WINDOW_WIDTH/2 + 300, 40});
	// objects.push_back(direc2d2);

	// Direction3D *direc3d = new Direction3D;
	// direc3d->create(20);
	// // direc3d->setPosition({WINDOW_WIDTH/2 + 450, 100});
	// objects.push_back(direc3d);

	navi = new Navigation;

	bg = new BackGround;
	bg->create();

	// for(int i=0; i<13; i++){	
	// 	Object *tmp = new Object(physicalDevice, device, queueFamilyIndices, graphicsQueue,
	// 							*(renderer->getSwapchain()), renderer->getCommandPool(), renderer->getRenderPass());
	// 	tmp->create(OBJECT_FLAG_USE_TEXTURE | OBJECT_FLAG_USE_3D, "textures/tex.jpg");
	// 	tmp->setPosition({i/5, i%5, 0});
	// 	objects.push_back(tmp);
	// }
	
	// Object *tmp2 = new Object(physicalDevice, device, queueFamilyIndices, graphicsQueue,
	// 						*(renderer->getSwapchain()), renderer->getCommandPool(), renderer->getRenderPass());
	// tmp2->create(OBJECT_FLAG_USE_3D, NONE_TEXTURE);
	// tmp2->setColor({1, 0, 1, 1});
	// objects.push_back(tmp2);
	
	// Object *bg = new Object(physicalDevice, device, queueFamilyIndices, graphicsQueue,
	// 						*(renderer->getSwapchain()), renderer->getCommandPool(), renderer->getRenderPass());
	// bg->create(OBJECT_FLAG_USE_TEXTURE_IMG, "textures/bg.png");
	// bg->setPosition({0, 0, 10});
	// bg->setWidth({WINDOW_WIDTH, WINDOW_HEIGHT});
	// objects.push_back(bg);

	/* Object *direc = new Object(physicalDevice, device, queueFamilyIndices, graphicsQueue,
							*(renderer->getSwapchain()), renderer->getCommandPool(), renderer->getRenderPass());
	direc->create(OBJECT_FLAG_USE_TEXTURE | OBJECT_FLAG_USE_3D, "../../../src/vulkan/textures/direction.png");
	direc->setPosition({0, 3, 0});
	direc->setWidth({3, 3});
	direc->setNormal({0, -1, 0});
	objects.push_back(direc);

	Text *fps = new Text(physicalDevice, device, queueFamilyIndices, graphicsQueue,
						*(renderer->getSwapchain()), renderer->getCommandPool(), renderer->getRenderPass());
	fps->create(0, FONT_PATH_NANUMGOTHIC, L"60");
	fps->setColor({1, 1, 0, 1});
	fps->setHeight(30.0f);
	fps->setPosition({WINDOW_WIDTH/2-fps->getWidth().x/2, WINDOW_HEIGHT-30, 0});
	objects.push_back(fps); */

	// Object *vor = new Object(physicalDevice, device, queueFamilyIndices, graphicsQueue,
	// 						*(renderer->getSwapchain()), renderer->getCommandPool(), renderer->getRenderPass());
	// vor->create(0, NONE_TEXTURE);
	// vor->setWidth({vo->getWidth().x + 5, vo->getWidth().y + 5});
	// vor->setPosition({voi->getPosition().x + voi->getWidth().x + 7.5, voi->getPosition().y, 1});
	// vor->setColor({0.5, 0.5, 0.5, 0.5});
	// objects.push_back(vor);
	
	// warning = new Warning;
	// warning->create(1, 0, 10, 1);

	// objects.push_back(lane);
	objects.push_back(speed);
	objects.push_back(bg);
	// objects.push_back(warning);

}

void VulkanApplication::loop(){
	// auto startTime = std::chrono ::high_resolution_clock::now();
	ros::Time st, ct;
	// for(int i=0; i<6; i++){
	while (!glfwWindowShouldClose(window) && ros::ok()) {
		auto startTime = std::chrono::high_resolution_clock::now();
		glfwPollEvents();
		ros::spinOnce();

		renderer->sync();
		renderer->bind(objects);
		renderer->update(objects);
		renderer->draw();
		
		rate->sleep();
		ct = ros::Time::now();
		time = (ct - st).nsec / 1e9;
		st = ros::Time::now();
		
		notiManager->update();
		navi->update();
		// ((Direction3D *)(objects.back()))->setDistance(20);
		// speed->updateSpeed((int)round(1/time));
		// std::cout<<1/time<<"\n";

		// auto currentTime = std::chrono::high_resolution_clock::now();		

		// float time = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - startTime).count();
		// while (time < (float)1/60){
		// 	currentTime = std::chrono::high_resolution_clock::now();
		// 	time = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - startTime).count();
		// }
		// ((Text *)objects.back())->updateText(std::to_wstring(1/time), renderer->getImageIndex());

	}
	// auto currentTime = std::chrono::high_resolution_clock::now();
	// float time = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - startTime).count();
	// std::cout<<time<<"\n";
	// std::cout<<"\n\n";
	vkDeviceWaitIdle(device);
}

void VulkanApplication::finish() {
	for(auto object : objects) {
		object->destroy();
		delete object;
	}

	renderer->destroy();

	vkDestroyDevice(device, nullptr);
	
	if (enableValidationLayers) DestroyDebugUtilsMessengerEXT(instance, debugMessenger, nullptr);
	vkDestroySurfaceKHR(instance, surface, nullptr);	
	vkDestroyInstance(instance, nullptr);

	glfwDestroyWindow(window);
	glfwTerminate();
}

std::vector<VkExtensionProperties> VulkanApplication::getSupportedInstanceExtensionList(){
	VkResult result;
	uint32_t extensionCount = 0;
	vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr);
	
	std::vector<VkExtensionProperties> extensions(extensionCount);
	vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, extensions.data());

	std::cout << "available layer extensions:\n";
	for (const auto& extension : extensions) {
		std::cout << '\t' << extension.extensionName << '\n';
	}

	return extensions;
}

std::vector<VkLayerProperties> VulkanApplication::getSupportedLayerList(){
    uint32_t layerCount = 0;
    vkEnumerateInstanceLayerProperties(&layerCount, nullptr);

	std::vector<VkLayerProperties> layers(layerCount);
    vkEnumerateInstanceLayerProperties(&layerCount, layers.data());

	std::cout << "available layers:\n";
	for (const auto& layer : layers) {
		std::cout << '\t' << layer.layerName << '\n';
	}

	return layers;
}

VkPhysicalDeviceProperties VulkanApplication::getSupportedDevice(){
	VkPhysicalDeviceProperties physicalDeviceProperties;
	
	vkGetPhysicalDeviceProperties(physicalDevice, &physicalDeviceProperties);

	std::cout << "available devices:\n";
	std::cout << '\t' << physicalDeviceProperties.deviceName << '\n';

	return physicalDeviceProperties;
}

std::vector<VkExtensionProperties> VulkanApplication::getSupportedDeviceExtensionList(){
	uint32_t extensionCount = 0;
	vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &extensionCount, nullptr);
	
	std::vector<VkExtensionProperties> availableExtensions(extensionCount);
	vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &extensionCount, availableExtensions.data());	
	
	std::cout << "available device extensions:\n";
	for (const auto& extension : availableExtensions) {
		std::cout << '\t' << extension.extensionName << '\n';
	}

	return availableExtensions;
}

/** 
 * private function 
 */
/** ----------------------------- Intance 관련 -------------------------------------- */
void VulkanApplication::createInstance(){
	if (enableValidationLayers && !checkValidationLayerSupport()) {
        throw std::runtime_error("validation layers requested, but not available!");
    }

	// Vulkan 애플리케이션 구조체 정의
	VkApplicationInfo appInfo 	= {};
    appInfo.sType 				= VK_STRUCTURE_TYPE_APPLICATION_INFO;
	appInfo.pNext				= NULL;
    appInfo.pApplicationName 	= "Application Name";
    appInfo.applicationVersion 	= VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName 		= "No Engine";
    appInfo.engineVersion 		= VK_MAKE_VERSION(1, 0, 0);
	// VK_API_VERSION은 지원 종료, 대신 VK_MAKE_VERSION 사용
	appInfo.apiVersion			= VK_MAKE_VERSION(1, 0, 0);
	
	// Vulkan 인스턴스 생성 정보 구조체 정의
	VkInstanceCreateInfo instanceInfo 	= {};
	instanceInfo.sType 					= VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	instanceInfo.pNext					= VK_NULL_HANDLE;
	instanceInfo.flags					= 0;
	instanceInfo.pApplicationInfo 		= &appInfo;
	
	auto extensions = getRequiredExtensions();
	instanceInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
	instanceInfo.ppEnabledExtensionNames = extensions.data();

	if (enableValidationLayers) {
		instanceInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
		instanceInfo.ppEnabledLayerNames = validationLayers.data();
	} else {
		instanceInfo.enabledLayerCount = 0;
	}

	VkResult result = vkCreateInstance(&instanceInfo, NULL, &instance);
	if(result != VK_SUCCESS) throw std::runtime_error("failed to create instance!");
}

/** ----------------------------- Debug 관련 -------------------------------------- */
void VulkanApplication::setupDebugMessenger() {
	if (!enableValidationLayers) return;

	VkDebugUtilsMessengerCreateInfoEXT createInfo;
	createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    createInfo.messageSeverity = //VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | 
								 VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | 
								 VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | 
							 VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | 
							 VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    createInfo.pfnUserCallback = debugCallback;

	if (CreateDebugUtilsMessengerEXT(instance, &createInfo, nullptr, &debugMessenger) != VK_SUCCESS)
		throw std::runtime_error("failed to set up debug messenger!");
}

VkResult VulkanApplication::CreateDebugUtilsMessengerEXT(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pDebugMessenger) {
	auto func = (PFN_vkCreateDebugUtilsMessengerEXT) vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
	if (func != nullptr) return func(instance, pCreateInfo, pAllocator, pDebugMessenger);
	else 				 return VK_ERROR_EXTENSION_NOT_PRESENT;
}

void VulkanApplication::DestroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks* pAllocator) {
	auto func = (PFN_vkDestroyDebugUtilsMessengerEXT) vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT");
	if (func != nullptr) func(instance, debugMessenger, pAllocator);
}

/** ----------------------------- Device 관련 -------------------------------------- */
void VulkanApplication::createLogicalDevice(){
	queueFamilyIndices = findQueueFamilies(physicalDevice, surface);
	std::set<uint32_t> uniqueQueueFamilies = {queueFamilyIndices.graphicsFamily.value(), queueFamilyIndices.presentFamily.value()};

	std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;	
	float queuePriority = 1.0f;
	for (uint32_t queueFamily : uniqueQueueFamilies) {
		VkDeviceQueueCreateInfo queueCreateInfo{};
		queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
		queueCreateInfo.queueFamilyIndex = queueFamily;
		queueCreateInfo.queueCount = 1;
		queueCreateInfo.pQueuePriorities = &queuePriority;
		queueCreateInfos.push_back(queueCreateInfo);
	}

	VkPhysicalDeviceFeatures deviceFeatures{};
	deviceFeatures.samplerAnisotropy = VK_TRUE;

	VkDeviceCreateInfo createInfo{};
	createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;

	createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
	createInfo.pQueueCreateInfos = queueCreateInfos.data();

	createInfo.pEnabledFeatures = &deviceFeatures;

	createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
	createInfo.ppEnabledExtensionNames = deviceExtensions.data();

	if (enableValidationLayers) {
		createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
		createInfo.ppEnabledLayerNames = validationLayers.data();
	} else {
		createInfo.enabledLayerCount = 0;
	}

	VkResult result = vkCreateDevice(physicalDevice, &createInfo, nullptr, &device);
	if (result != VK_SUCCESS) throw std::runtime_error("failed to create device!");

	vkGetDeviceQueue(device, queueFamilyIndices.graphicsFamily.value(), 0, &graphicsQueue);
	vkGetDeviceQueue(device, queueFamilyIndices.presentFamily.value(), 0, &presentQueue);
}

void VulkanApplication::getPhysicalDevice(){
	uint32_t deviceCount = 0;
	vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);

	if (deviceCount == 0) throw std::runtime_error("failed to find GPUs with Vulkan support!");

	std::vector<VkPhysicalDevice> devices(deviceCount);
	vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data());
	
	for (const auto& device : devices) {
		if (isDeviceSuitable(device, surface)) {
			physicalDevice = device;
			break;
		}
	}

	if (physicalDevice == VK_NULL_HANDLE) throw std::runtime_error("failed to find a suitable GPU!");
}

/** ----------------------------- Presentation 관련 -------------------------------------- */
void VulkanApplication::createSurface(){
	if (glfwCreateWindowSurface(instance, window, nullptr, &surface) != VK_SUCCESS) {
		throw std::runtime_error("failed to create window surface!");
	}
}


std::vector<const char*> VulkanApplication::getRequiredExtensions() {
	// GLFW has a handy built-in function that returns the extension(s) 
	// it needs to do that which we can pass to the struct
    uint32_t glfwExtensionCount = 0;
    const char** glfwExtensions;
    glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);

    std::vector<const char*> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);

    if (enableValidationLayers) extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);

    return extensions;
}

bool VulkanApplication::checkValidationLayerSupport() {
	// get supported layer list
    uint32_t layerCount;
    vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
    std::vector<VkLayerProperties> availableLayers(layerCount);
    vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());

	// check layer list has validation layer 
	for (const char* layerName : validationLayers) {
		bool layerFound = false;

		for (const auto& layerProperties : availableLayers) {
			if (strcmp(layerName, layerProperties.layerName) == 0) {
				layerFound = true;
				break;
			}
		}

		if (!layerFound) return false;
	}

	return true;
}

bool VulkanApplication::isDeviceSuitable(const VkPhysicalDevice &device, const VkSurfaceKHR &surface){
	// 그래픽스 큐 가지고 있는지 & 프레젠트 큐 지원하는지
    QueueFamilyIndices indices = findQueueFamilies(device, surface);
	// 디바이스 익스텐션 지원 체크 (스왑체인)
	bool extensionsSupported = checkDeviceExtensionSupport(device);
	// 스왑체인 디테일 체크 (이미지 포맷, 프레젠테이션 모드 1개 이상 지원하는지)
	bool swapCahinAdequate = false;
	if (extensionsSupported){
		SwapChainSupportDetails swapCahinSupport = querySwapchainSupport(device, surface);
		swapCahinAdequate = !swapCahinSupport.formats.empty() && !swapCahinSupport.presentModes.empty();
	}
	VkPhysicalDeviceFeatures supportedFeatures;
    vkGetPhysicalDeviceFeatures(device, &supportedFeatures);

    return indices.isComplete() && 
		   extensionsSupported && 
		   swapCahinAdequate && 
		   supportedFeatures.samplerAnisotropy;
}

bool VulkanApplication::checkDeviceExtensionSupport(const VkPhysicalDevice &device){
	uint32_t extensionCount;
	vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, nullptr);

	std::vector<VkExtensionProperties> availableExtensions(extensionCount);
	vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, availableExtensions.data());

	std::set<std::string> requiredExtensions(deviceExtensions.begin(), deviceExtensions.end());
	for (const auto& extension : availableExtensions) {
		requiredExtensions.erase(extension.extensionName);
	}

	return requiredExtensions.empty();
}

QueueFamilyIndices VulkanApplication::findQueueFamilies(const VkPhysicalDevice &device, const VkSurfaceKHR &surface) {
	QueueFamilyIndices indices;

	uint32_t queueFamilyCount = 0;
	vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);

	std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
	vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilies.data());

	int i = 0;
	for (const auto& queueFamily : queueFamilies) {
		if (queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
			indices.graphicsFamily = i;
		}

		VkBool32 presentSupport = false;
		vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface, &presentSupport);
		if (presentSupport) {
			indices.presentFamily = i;
		}

		if (indices.isComplete()) break;
		i++;
	}

	return indices;
}

SwapChainSupportDetails VulkanApplication::querySwapchainSupport(const VkPhysicalDevice &physicalDevice, const VkSurfaceKHR &surface){
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