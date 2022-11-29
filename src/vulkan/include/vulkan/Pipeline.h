#ifndef __PIPELINE_H__
#define __PIPELINE_H__
#include "header.h"

#include "Swapchain.h"

class Pipeline
{
private:
    VkDevice          		&device; 
	Swapchain 				&swapchain;
	VkRenderPass 			&renderPass;

	VkPipelineLayout 		pipelineLayout;
	VkPipeline 				graphicsPipeline;
public:
    Pipeline(VkDevice &device, Swapchain &swapchain, VkRenderPass &renderPass);
    ~Pipeline();

	void create(const VkDescriptorSetLayout &descriptorSetLayout);
    void createGraphicsPipeline(const VkDescriptorSetLayout &descriptorSetLayout, const std::string &uniformPath, const std::string &fragmentPath, VertexLayout vertexLayout);
	void destroy();

	// inline VkRenderPass getRenderPass() const { return renderPass; }
	inline VkPipelineLayout getPipelineLayout() const { return pipelineLayout; }
	inline VkPipeline getPipeline() const { return graphicsPipeline; }
private:
	VkShaderModule createShaderModule(const std::vector<char>& code);
	static std::vector<char> readFile(const std::string& filename);
};


#endif // __PIPELINE_H__