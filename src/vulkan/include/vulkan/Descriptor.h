#ifndef __DESCRIPTOR_H__
#define __DESCRIPTOR_H__
#include "header.h"

#include "Texture.h"

class Descriptor
{
private:
	VkDevice						&device;
    size_t							swapchainSize;
	bool 							flagUseTexture;

	VkDescriptorSetLayout 			descriptorSetLayout;
	VkDescriptorPool 			    descriptorPool;
	std::vector<VkDescriptorSet> 	descriptorSets;
public:
    Descriptor(VkDevice &device, size_t swapchainSize, bool flagUseTexture);
    ~Descriptor();

	void createDescriptorSetLayout();
	void createDescriptorPool();
	void createDescriptorSets(std::vector<VkBuffer> &uniformBuffers, Texture *texture);
	void updateDescriptorSets(std::vector<VkBuffer> &uniformBuffers, Texture *texture, uint32_t imageIndex);
    void destroy();
	void destroyDescriptorPool();

    inline VkDescriptorSetLayout getDescriptorSetLayout() const { return descriptorSetLayout; }
    inline const std::vector<VkDescriptorSet> getDescriptorSets() const& { return descriptorSets; }
};
#endif // __DESCRIPTOR_H__