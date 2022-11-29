#include "Texture.h"

#include "Utils.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

Texture::Texture(VkPhysicalDevice &physicalDevice, VkDevice &device, VkQueue &graphicsQueue, VkCommandPool &commandPool) 
	: physicalDevice(physicalDevice), device(device), graphicsQueue(graphicsQueue), commandPool(commandPool)
{
}

Texture::~Texture()
{
}

void Texture::createImagePixels(std::string filepath)
{
	pixels = stbi_load(filepath.c_str(), &width, &height, &channel, STBI_rgb_alpha);
}

void Texture::createImagePixels(std::vector<unsigned char> data, int width, int height)
{
	channel = 3;
	pixels = (unsigned char *)calloc(width*height*channel, sizeof(unsigned char));
	std::copy(data.begin(), data.end(), pixels);
	// pixels = stbi_load_from_memory(data.data(), data.size(), &width, &height, &channel, STBI_rgb);
	pixels = stbi__convert_format(pixels, STBI_rgb, STBI_rgb_alpha, width, height);
	this->width = width;
	this->height = height;
	channel = 4;
}

void Texture::createTextPixels(std::string filepath, std::wstring msg, glm::vec4 color, float &width, float &height)
{
	this->height = height;
	pixels = getTextbitmap(filepath, msg);
	// pixels = (unsigned char*)calloc(width*height*4, sizeof(unsigned char));
	pixels = stbi__convert_format(pixels, STBI_grey, STBI_rgb_alpha, this->width, this->height);
	for(int i=0; i<this->width*this->height*4; i+=4) {
		pixels[i+3] = pixels[i] * color.w;
		pixels[i] *= color.x;
		pixels[i+1] *= color.y;
		pixels[i+2] *= color.z;
	}
	width = this->width;
	height = this->height;
}

void Texture::create()
{
	assert(pixels != nullptr);

    createTextureImage();
    createTextureImageView();
    createTextureSampler();
	stbi_image_free(pixels);
}

void Texture::destroy()
{
	vkDestroySampler(device, textureSampler, nullptr);
	vkDestroyImageView(device, textureImageView, nullptr);
    vkDestroyImage(device, textureImage, nullptr);
    vkFreeMemory(device, textureImageMemory, nullptr);
}

void Texture::createTextureImage()
{
	assert(pixels != nullptr);

    VkDeviceSize imageSize = width * height * 4;    

	VkBuffer stagingBuffer;
	VkDeviceMemory stagingBufferMemory;

	createBuffer(device, physicalDevice, imageSize, 
				 VK_BUFFER_USAGE_TRANSFER_SRC_BIT, 
				 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, 
				 stagingBuffer, 
				 stagingBufferMemory);
	void* data;
	vkMapMemory(device, stagingBufferMemory, 0, imageSize, 0, &data);
		memcpy(data, pixels, static_cast<size_t>(imageSize));
	vkUnmapMemory(device, stagingBufferMemory);

	createImage(device, physicalDevice, 
                width, height, VK_FORMAT_R8G8B8A8_SRGB, 
				VK_IMAGE_TILING_OPTIMAL, 
				VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, 
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 
				textureImage, 
				textureImageMemory);

	transitionImageLayout(textureImage, 
						  VK_FORMAT_R8G8B8A8_SRGB, 
						  VK_IMAGE_LAYOUT_UNDEFINED, 
						  VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
	copyBufferToImage(stagingBuffer, 
					  textureImage, 
					  static_cast<uint32_t>(width), static_cast<uint32_t>(height));
	transitionImageLayout(textureImage, 
						  VK_FORMAT_R8G8B8A8_SRGB, 
						  VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 
						  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
	
	vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingBufferMemory, nullptr);
}

void Texture::createTextureImageView() 
{
	textureImageView = createImageView(device, textureImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_ASPECT_COLOR_BIT);
}

void Texture::createTextureSampler() 
{
	VkPhysicalDeviceProperties properties{};
	vkGetPhysicalDeviceProperties(physicalDevice, &properties);

	VkSamplerCreateInfo samplerInfo{};
	samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
	samplerInfo.magFilter = VK_FILTER_LINEAR;
	samplerInfo.minFilter = VK_FILTER_LINEAR;

	samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;

	samplerInfo.anisotropyEnable = VK_TRUE;
	samplerInfo.maxAnisotropy = properties.limits.maxSamplerAnisotropy;

	samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
	
	samplerInfo.unnormalizedCoordinates = VK_FALSE;

	samplerInfo.compareEnable = VK_FALSE;
	samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
	
	samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
	samplerInfo.mipLodBias = 0.0f;
	samplerInfo.minLod = 0.0f;
	samplerInfo.maxLod = 0.0f;

	assert(vkCreateSampler(device, &samplerInfo, nullptr, &textureSampler) == VK_SUCCESS);
}

void Texture::setFontInfo(stbtt_fontinfo &fontInfo, std::string filepath){
	unsigned char* fontBuffer;
	FILE* fontFile = fopen(filepath.c_str(), "rb");
	fseek(fontFile, 0, SEEK_END);
	long size = ftell(fontFile); /* how long is the file ? */
	fseek(fontFile, 0, SEEK_SET); /* reset */

	fontBuffer = (unsigned char*)malloc(size);

	size_t s = fread(fontBuffer, size, 1, fontFile);
	fclose(fontFile);

	assert(stbtt_InitFont(&fontInfo, fontBuffer, 0));
}

unsigned char *Texture::getTextbitmap(std::string filepath, std::wstring msg){
	assert(!msg.empty());

	stbtt_fontinfo fontInfo;
	setFontInfo(fontInfo, filepath);

	/* calculate font scaling */
	float scale = stbtt_ScaleForPixelHeight(&fontInfo, height);

	width = 0;
	for (int i = 0; i < msg.size(); ++i) {
		int ax, lsb;
		stbtt_GetCodepointHMetrics(&fontInfo, msg[i], &ax, &lsb);
		width += roundf(ax * scale);
	}

	/* create a bitmap for the phrase */
	unsigned char* pixels = (unsigned char*)calloc(width * height, sizeof(unsigned char));
	
	int ascent, descent, lineGap;
	stbtt_GetFontVMetrics(&fontInfo, &ascent, &descent, &lineGap);
	ascent = roundf(ascent * scale);
	
	int x = 0;
	for (int i = 0; i < msg.size(); ++i) {
		/* how wide is this character */
		int ax;
		int lsb;
		stbtt_GetCodepointHMetrics(&fontInfo, msg[i], &ax, &lsb);

		/* get bounding box for character (may be offset to account for chars that dip above or below the line */
		int c_x1, c_y1, c_x2, c_y2;
		stbtt_GetCodepointBitmapBox(&fontInfo, msg[i], scale, scale, &c_x1, &c_y1, &c_x2, &c_y2);

		// stbtt_GetCodepointBitmap(&fontInfo, scale, scale, msg[i], )

		/* compute y (different characters have different heights */
		int y = ascent + c_y1;
		
		/* render character (stride and offset is important here) */
		int byteOffset = x + roundf(lsb * scale) + (y * width);
		stbtt_MakeCodepointBitmap(&fontInfo, pixels + byteOffset, c_x2 - c_x1, c_y2 - c_y1, width, scale, scale, msg[i]);

		/* advance x */
		x += roundf(ax * scale);
		
		/* add kerning */
		int kern = stbtt_GetCodepointKernAdvance(&fontInfo, msg[i], msg[i + 1]);
		x += roundf(kern * scale);
	}
	return pixels;
}

void Texture::transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout) 
{
    VkCommandBuffer commandBuffer = beginSingleTimeCommands(device, commandPool);
	
	VkImageMemoryBarrier barrier{};
	barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	barrier.oldLayout = oldLayout;
	barrier.newLayout = newLayout;
	barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

	barrier.image = image;
	barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	barrier.subresourceRange.baseMipLevel = 0;
	barrier.subresourceRange.levelCount = 1;
	barrier.subresourceRange.baseArrayLayer = 0;
	barrier.subresourceRange.layerCount = 1;

	VkPipelineStageFlags sourceStage;
	VkPipelineStageFlags destinationStage;

	if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
		barrier.srcAccessMask = 0;
		barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

		sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
		destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
	} else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) {
		barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

		sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
		destinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
	} else 
		throw std::invalid_argument("unsupported layout transition!");

	vkCmdPipelineBarrier(commandBuffer,
						 sourceStage, destinationStage,
						 0,
						 0, nullptr,
						 0, nullptr,
						 1, &barrier);

    endSingleTimeCommands(device, commandPool, graphicsQueue, commandBuffer);
}

void Texture::copyBufferToImage(VkBuffer buffer, VkImage image, uint32_t width, uint32_t height) 
{
    VkCommandBuffer commandBuffer = beginSingleTimeCommands(device, commandPool);
	
	VkBufferImageCopy region{};
	region.bufferOffset = 0;
	region.bufferRowLength = 0;
	region.bufferImageHeight = 0;

	region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	region.imageSubresource.mipLevel = 0;
	region.imageSubresource.baseArrayLayer = 0;
	region.imageSubresource.layerCount = 1;

	region.imageOffset = {0, 0, 0};
	region.imageExtent = {width,
						  height,
						  1 };

	vkCmdCopyBufferToImage(commandBuffer,
    					   buffer,
    					   image,
    					   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
    					   1,
    					   &region);
    
	endSingleTimeCommands(device, commandPool, graphicsQueue, commandBuffer);
}