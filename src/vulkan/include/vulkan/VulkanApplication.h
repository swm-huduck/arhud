#ifndef __VULKANAPPLICATION_H__
#define __VULKANAPPLICATION_H__
#include "header.h"

#include "Renderer.h"
#include "Object.h"
#include "UserObject.h"

#include "vulkan/line_coef.h"
#include "vulkan/speed.h"
#include "vulkan/navigation_turn_event.h"
#include "vulkan/setting.h"
#include "vulkan/call.h"
#include "vulkan/sms.h"
#include "vulkan/kakao_talk.h"
#include "vulkan/eye_position.h"
#include "vulkan/warning.h"
#include <std_msgs/UInt8MultiArray.h>

#include <ros/package.h>

class VulkanApplication {
private:
	static std::unique_ptr<VulkanApplication> 	ApplicationInstance;
    static std::once_flag 						onlyOnce;

	ros::NodeHandle 			*nodeHandle;
	ros::Subscriber 			subscriber;
	ros::Subscriber 			callSubscriber;
	ros::Subscriber 			smsSubscriber;
	ros::Subscriber 			kakaoSubscriber;
	ros::Subscriber 			speedSubscriber;
	ros::Subscriber 			navigationSubscriber;
	ros::Subscriber 			warningSubscriber;
	ros::Subscriber 			eyeSubscriber;
    ros::Rate 					*rate;

	ros::Subscriber 			bgSubscriber;

	GLFWwindow 					*window;

	VkInstance 					instance;
	VkDebugUtilsMessengerEXT 	debugMessenger;
	
	// static std::vector<Object*> objects;

public:	
	VkPhysicalDevice 			physicalDevice = VK_NULL_HANDLE;
	VkDevice					device;

	QueueFamilyIndices			queueFamilyIndices;
	VkQueue						graphicsQueue;
	VkQueue						presentQueue;

	VkSurfaceKHR				surface;

	Renderer					*renderer;

	static NotificationManager 	*notiManager;
	static Speed				*speed;
	static Polyline				*lane;
	static Navigation			*navi;
	static BackGround			*bg;
	static Warning				*warning;
	static std::vector<UserObject*> objects;
	static float time;

private:
	// 생성자 (For Singleton)
	VulkanApplication();

public:
	// 소멸자
	~VulkanApplication();

    static VulkanApplication* GetInstance();

	void init();	
	void loop();
	void finish();
private:
	void createInstance();

    void setupDebugMessenger();
	VkResult CreateDebugUtilsMessengerEXT(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pDebugMessenger);
	void DestroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks* pAllocator);

	void createLogicalDevice();
	void getPhysicalDevice();

	void createSurface();
	


	std::vector<VkExtensionProperties> getSupportedInstanceExtensionList();
	std::vector<VkLayerProperties> getSupportedLayerList();
	VkPhysicalDeviceProperties getSupportedDevice();
	std::vector<VkExtensionProperties> getSupportedDeviceExtensionList();
	bool hasStencilComponent(VkFormat format) {
    	return format == VK_FORMAT_D32_SFLOAT_S8_UINT || format == VK_FORMAT_D24_UNORM_S8_UINT;
	}
	std::vector<const char*> getRequiredExtensions();
	bool checkValidationLayerSupport();
	bool isDeviceSuitable(const VkPhysicalDevice &device, const VkSurfaceKHR &surface);
	bool checkDeviceExtensionSupport(const VkPhysicalDevice &device);
	QueueFamilyIndices findQueueFamilies(const VkPhysicalDevice &device, const VkSurfaceKHR &surface);

	SwapChainSupportDetails querySwapchainSupport(const VkPhysicalDevice &physicalDevice, const VkSurfaceKHR &surface);

	static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageType, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData, void* pUserData) {
		/** 
		 * VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT: Diagnostic message
		 * VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT: 	Informational message like the creation of a resource
		 * VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT: Message about behavior that is not necessarily an error, but very likely a bug in your application
		 * VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT: 	Message about behavior that is invalid and may cause crashes
		*/
		std::cerr << "validation layer: " << pCallbackData->pMessage << "\n";
		return VK_FALSE;
	}

	static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
	{
		if (action == GLFW_PRESS || action == GLFW_REPEAT){
			switch (key){
			case GLFW_KEY_W:
				Camera::getInstance().move(MOVE_FRONT);
				break;
			case GLFW_KEY_A:
				Camera::getInstance().move(MOVE_LEFT);
				break;
			case GLFW_KEY_S:
				Camera::getInstance().move(MOVE_BACK);
				break;
			case GLFW_KEY_D:
				Camera::getInstance().move(MOVE_RIGHT);
				break;
			case GLFW_KEY_Q:
				Camera::getInstance().move(MOVE_UP);
				break;
			case GLFW_KEY_E:
				Camera::getInstance().move(MOVE_DOWN);
				break;
			case GLFW_KEY_LEFT:
				Camera::getInstance().turn(TURN_LEFT);
				break;
			case GLFW_KEY_RIGHT:
				Camera::getInstance().turn(TURN_RIGHT);
				break;
			case GLFW_KEY_UP:
				Camera::getInstance().turn(TURN_UP);
				break;
			case GLFW_KEY_DOWN:
				Camera::getInstance().turn(TURN_DOWN);
				break;
			case GLFW_KEY_SPACE:
				break;
			default:
				break;
			}
		}
	}
	static void lineCallback(const vulkan::line_coef::ConstPtr& msg)
	{
		// for(auto v : msg.line_coef)
		// 	std::cout<<v<<" ";
		// std::cout<<"\n";
		// ROS_INFO("callback ");
		lane->updateVertices(msg->line_coef, 2);
	}

	// 전화 하는 도중에 전화가 올 수도 있음!
	// -> 전화 받으면 알림 사라짐 -> 전화 종료 시에 굳이 알림 X
	// 전화 옴 -> 안받음 -> 끊어지면 전화 종료 -> 글씨 빨간색으로 0.5초 후 삭제
	// 전화 옴 -> 받음 -> 전화 또 옴 -> 받을 때 사라졌다가 다시 알림 뜸
	static void callCallback(const vulkan::call::ConstPtr& data) {
		std::cout<<"[CallCallback]\nname: "<< data->name.c_str() <<"\ncallType: "<< (int)data->call_type <<"\n";

		notiManager->addCall(data->name, data->call_type);
	}

	static void smsCallback(const vulkan::sms::ConstPtr& data) {
		std::cout<<"[SmsCallback]\nname: "<< data->name.c_str() <<"\ncontent: "<< data->content.c_str() <<"\n";
		
		notiManager->addSMS(data->name+": "+data->content);
	}

	static void kakaoCallback(const vulkan::kakao_talk::ConstPtr& data) {
		std::cout<<"[KakaoTalkCallback]\nname: "<< data->name.c_str() <<"\ncontent: "<< data->content.c_str() <<"\n";

		notiManager->addKakao(data->name+": "+data->content);
	}
	
	static void speedCallback(const vulkan::speed::ConstPtr& data) {
		// std::cout<<"[SpeedCallback]\nvalue: "<<data->value<<"\n";
		speed->updateSpeed(data->value);
		// std::cout<<ros::Time::now().sec<<"\n";
	}

	static void navigationCallback(const vulkan::navigation_turn_event::ConstPtr& data) {
		// std::cout<<"[NavigationTurnEventCallback]\nevent turn type: "<<data->next_turn_type<<
		// 										"\nevent distance: "<<data->next_left_distance<<
		// 										"\nevent position: "<<data->next_relational_position_x<<" "<<data->next_relational_position_y<< 
		// 										"\nnext turn type: "<<data->next_next_turn_type<<
		// 										"\nnext distance: "<<data->next_next_left_distance<<"\n";

		navi->updateData(data->next_turn_type, data->next_left_distance, data->next_next_turn_type, data->next_next_left_distance);
	}

	static void warningCallback(const vulkan::warning::ConstPtr& data) {
		std::cout<<"[Warning Callback]\n";
		if (warning == nullptr) {
			warning = new Warning;
			warning->create(data->x, data->y, data->depth, data->level);
			objects.push_back(warning);
		} else {
			warning->setPosition(data->x, data->y, data->depth, data->level);
		}

	}

	/* static void SettingCallback(const vulkan::setting& data) {
		printf("[SettingCallback] item: %s, value: %s", data.item.c_str(), data.value.c_str());
		ROS_INFO("");
	} */

	static void EyePositionCallback(const vulkan::eye_position& data) {
		printf("[EyePositionCallback] position: (%d, %d)", data.x, data.y);
		float x = (float)data.x/1000.0;
		float y = (float)data.y/1000.0;
		Camera::getInstance().move(x, 0, y);
	} 

	static void imageCallback(const std_msgs::UInt8MultiArray::ConstPtr& array) {
		// std::cout<<"[ImageCallback]\n";
		bg->updateTexture(array->data);
	}

};
#endif // __VULKANAPPLICATION_H__