#ifndef __OBJECT_H__
#define __OBJECT_H__
#include "header.h"

#include "Swapchain.h"
#include "Texture.h"
#include "Pipeline.h"
#include "Descriptor.h"

class Object
{
protected:
    VkPhysicalDevice      			&physicalDevice;
    VkDevice              			&device;
	QueueFamilyIndices				&queueFamilyIndices;
    VkQueue               			&graphicsQueue;
    Swapchain                 		&swapchain; 
	VkCommandPool 					&commandPool;
	VkRenderPass					&renderPass;
	size_t							swapchainSize = 0;

	VkBuffer 						vertexBuffer;
	VkDeviceMemory 					vertexBufferMemory;
	VkBuffer 						indexBuffer;
	VkDeviceMemory 					indexBufferMemory;
	std::vector<VkBuffer> 			uniformBuffers;
	std::vector<VkDeviceMemory>		uniformBuffersMemory;
	
    bool							flagUseTexture = false;
    bool							flagUse3d = false;
	int								state;
	bool							*isBindingBuffers;
    
	Descriptor						*descriptor = nullptr;
	Pipeline						*pipeline = nullptr;
    Texture							*texture = nullptr;
	std::vector<Texture *> 			oldTexture;
	
	VertexLayout 					vertexLayout;
	std::vector<float>             	vertices;
    std::vector<uint16_t>           indices;

    glm::vec3						position;
    glm::vec2						width;
    glm::vec3						normal;
    glm::vec4						color;
	
public:
	Object();
    Object(	VkPhysicalDevice &physicalDevice, VkDevice &device, 
			QueueFamilyIndices &queueFamilyIndices, VkQueue &graphicsQueue, 
			Swapchain &swapchain, VkCommandPool &commandPool, VkRenderPass &renderPass);
    ~Object();
	
	void create(int objectFlag, std::string texturePath);
	virtual void destroy();

	void createVertexBuffers();
	void createIndexBuffer();
	void createUniformBuffers();

	void setDefaultVertices();
	void applyState();
    void updateUniformBuffer(uint32_t currentImage);
    void updateUniformBuffer(uint32_t currentImage, glm::mat4 mvp);
	void updateTexture(std::string path);
	void updateTexture(std::vector<unsigned char> data, int imageWidth, int imageHeight);

	void setState(const int &state) { this->state = state; }
	inline int getState() const { return state; }
	void setBindingFlag(int index, bool flag) { isBindingBuffers[index] = flag; }
	inline bool isAllUnbinding() const {
		for (int i=0; i<swapchainSize; i++) if (isBindingBuffers[i]) return false;
		return true;
	}
	
	void setVertexLayout(VertexLayout vertexLayout) { this->vertexLayout = vertexLayout; }
	inline VertexLayout getVertexLayout() const { return vertexLayout; }
	void setVertices(std::vector<float> vertices) { this->vertices = vertices; };
	inline std::vector<float> getVertices() const { return vertices; }
	void setIndices(std::vector<uint16_t> indices) { this->indices = indices; };
	inline std::vector<uint16_t> getIndices() const { return indices; }
    void setPosition(const glm::vec3 &position){ this->position = position;	}
	inline glm::vec3 getPosition() const { return position; }
    void setWidth(const glm::vec2 &width){ this->width = width; }
	inline glm::vec2 getWidth() const { return width; }
    void setNormal(const glm::vec3 &normal){ this->normal = normal;	}
	inline glm::vec3 getNormal() const { return normal; }
    void setColor(const glm::vec4 &color){ this->color = color;	}
	inline glm::vec4 getColor() const { return color; }

	inline VkBuffer getVertexBuffer() { return vertexBuffer; }
	inline VkDeviceMemory getVertexBufferMemory() { return vertexBufferMemory; }
	inline VkBuffer getIndexBuffer() { return indexBuffer; }
	inline VkDeviceMemory getIndexBufferMemory() { return indexBufferMemory; }
	inline Descriptor *getDescriptor() const { return descriptor; }
	inline Pipeline *getPipeline() const { return pipeline; }
	inline uint32_t getIndicesSize() const { return static_cast<uint32_t>(indices.size()); }
};

class Text : public Object
{
private:
	float textureRatio = 0;
	std::string texturePath;
public:
	Text();
	Text(	VkPhysicalDevice &physicalDevice, VkDevice &device, 
			QueueFamilyIndices &queueFamilyIndices, VkQueue &graphicsQueue, Swapchain &swapchain, 
			VkCommandPool &commandPool, VkRenderPass &renderPass);
	~Text();

	void create(int objectFlag, std::string texturePath, std::string msg);
	void destroy();

	void applyState();
	
	void setHeight(float size);
	void updateText(std::string msg);
private:
};

#endif // __OBJECT_H__