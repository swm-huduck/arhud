#ifndef __TEXTURE_H__
#define __TEXTURE_H__
#include "header.h"

#ifdef STB_IMAGE_IMPLEMENTATION
#undef STB_IMAGE_IMPLEMENTATION
#endif
#include "stb_image.h"
#ifdef STB_TRUETYPE_IMPLEMENTATION
#undef STB_TRUETYPE_IMPLEMENTATION
#endif
#include "stb_truetype.h"

class Texture
{
private:
    VkPhysicalDevice            &physicalDevice;
    VkDevice                    &device;
    VkQueue                     &graphicsQueue;
    VkCommandPool               &commandPool;

    VkImage 					textureImage;
	VkDeviceMemory 				textureImageMemory;
	VkImageView					textureImageView;
	VkSampler 					textureSampler;
    
    unsigned char               *pixels;
    int                         width;
    int                         height;
    int                         channel;
    
public:
    Texture(VkPhysicalDevice &physicalDevice, VkDevice &device, VkQueue &graphicsQueue, VkCommandPool &commandPool);
    Texture(std::string path);
    ~Texture();

    void createImagePixels(std::string filepath);
    void createImagePixels(std::vector<unsigned char> data, int width, int height);
    void createTextPixels(std::string filepath, std::wstring msg, glm::vec4 color, float &width, float &height);
    void create();
    void destroy();

    void createTextureImage();
    void createTextureImageView();
    void createTextureSampler();

    void setFontInfo(stbtt_fontinfo &fontInfo, std::string filepath);
    unsigned char *getTextbitmap(std::string filepath, std::wstring msg);

    // void setImagePixels(unsigned char *pixels, int width, int height);
    // void createTextureImageText(unsigned char *single_pixels, int texWidth, int texHeight);

    inline VkImageView getIamgeView() const { return textureImageView; }
    inline VkSampler getSampler() const { return textureSampler; }
private:
    void transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout);
    void copyBufferToImage(VkBuffer buffer, VkImage image, uint32_t width, uint32_t height);
};
#endif // __TEXTURE_H__