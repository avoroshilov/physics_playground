#pragma once

#include <vulkan/vulkan.h>

namespace vulkan
{

void getGenericSupportedDeviceExtensionsList(const VkPhysicalDevice & physDev, std::vector<VkExtensionProperties> * supportedExtensionsProps);

uint32_t findMemoryType(const VkPhysicalDevice & physDev, uint32_t typeFilter, VkMemoryPropertyFlags properties);

bool hasStencilComponent(VkFormat format);

VkFormat findSupportedFormat(
	const VkPhysicalDevice & physDev,
	const std::vector<VkFormat> & formatCandidates,
	VkImageTiling imageTiling,
	VkFormatFeatureFlags formatFeatures
	);

void createBuffer(
	const VkPhysicalDevice & physDev,
	const VkDevice & logicDev,
	VkDeviceSize size,
	VkBufferUsageFlags usage,
	VkMemoryPropertyFlags memoryProperties,
	VkBuffer * buffer,
	VkDeviceMemory * bufferDeviceMemory,
	VkAllocationCallbacks * pAllocator
	);

void createImage(
	const VkPhysicalDevice & physDev,
	const VkDevice & logicDev,
	uint32_t width,
	uint32_t height,
	VkFormat format,
	VkImageTiling tiling,
	VkImageUsageFlags usage,
	VkMemoryPropertyFlags memoryProperties,
	VkImage * image,
	VkDeviceMemory * imageDeviceMemory,
	VkAllocationCallbacks * pAllocator
	);

VkImageView createImageView2D(
	const VkDevice & logicDev,
	VkImage image,
	VkFormat format,
	VkImageAspectFlags imageAspectFlags,
	VkAllocationCallbacks * pAllocator
	);

/*** Graphics pipeline stage creation helpers ***/

VkPipelineViewportStateCreateInfo getDefaultViewportStateCreateInfo(VkExtent2D bufferSize);

// No DepthClamp, no RasterizerDiscard, PolygonMode=Fill, CullMode=Back, FrontFace=CW, no DepthBias, LineWidth=1.0
VkPipelineRasterizationStateCreateInfo getDefaultRasterizationStateCreateInfo();

// Per-fragment shading, 1 sample per fragment, no Alpha-To-Coverage
VkPipelineMultisampleStateCreateInfo getDefaultMultisampleStateCreateInfo();

// WriteMask=RGBA, no Blend, Color/Alpha Blend: Src=1 Dst=0 Op=add
VkPipelineColorBlendAttachmentState getDefaultColorBlendAttachmentState();

// no LogicOp, LogicOp=Copy, BlendConst=(0,0,0,0), !no attachments!
VkPipelineColorBlendStateCreateInfo getDefaultColorBlendStateCreateInfo();

// DepthTest=1, DepthWrite=1, DepthCompare=Less, no DepthBoundsTest, Min/MaxDepthBounds=0.0/1.0, no StencilTest
VkPipelineDepthStencilStateCreateInfo getDefaultDepthStencilStateCreateInfo();

}