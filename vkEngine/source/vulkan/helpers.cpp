#include <vector>
#include <assert.h>
#include <stdio.h>

#include "helpers.h"

#define REPORT_WARNING(text, file, line) \
	printf("%s:%d> " text "\n", file, line);

#define REPORT_ERROR(text, file, line) \
	printf("%s:%d> " text "\n", file, line); \
	assert(false && text);

namespace vulkan
{

void getGenericSupportedDeviceExtensionsList(const VkPhysicalDevice & physDev, std::vector<VkExtensionProperties> * supportedExtensionsProps)
{
	if (!supportedExtensionsProps)
		return;

	uint32_t extensionCount = 0;

	// Rare edge case where between two calls, list could be altered,
	//	result could be VK_INCOMPLETE, this process should be continued until status is not VK_INCOMPLETE
	VkResult result;
	do
	{
		result = vkEnumerateDeviceExtensionProperties(physDev, nullptr, &extensionCount, nullptr);

		supportedExtensionsProps->resize(extensionCount);
		result = vkEnumerateDeviceExtensionProperties(physDev, nullptr, &extensionCount, supportedExtensionsProps->data());
	} while (result == VK_INCOMPLETE);
}

uint32_t findMemoryType(const VkPhysicalDevice & physDev, uint32_t typeFilter, VkMemoryPropertyFlags memoryProperties)
{
	VkPhysicalDeviceMemoryProperties physicalDeviceMemoryProperties;
	vkGetPhysicalDeviceMemoryProperties(physDev, &physicalDeviceMemoryProperties);

	for (uint32_t i = 0; i < physicalDeviceMemoryProperties.memoryTypeCount; ++i)
	{
		if ( (typeFilter & (1 << i)) &&
				((physicalDeviceMemoryProperties.memoryTypes[i].propertyFlags & memoryProperties) == memoryProperties) )
		{
			return i;
		}
	}

	REPORT_WARNING("Failed to find suitable memory type!", __FILE__, __LINE__);
	return 0xFFffFFff;
}

bool hasStencilComponent(VkFormat format)
{
	return format == VK_FORMAT_D32_SFLOAT_S8_UINT || format == VK_FORMAT_D24_UNORM_S8_UINT;
}

VkFormat findSupportedFormat(
	const VkPhysicalDevice & physDev,
	const std::vector<VkFormat> & formatCandidates,
	VkImageTiling imageTiling,
	VkFormatFeatureFlags formatFeatures
	)
{
	for (VkFormat format : formatCandidates)
	{
		VkFormatProperties formatProperties;
		vkGetPhysicalDeviceFormatProperties(physDev, format, &formatProperties);

		if (imageTiling == VK_IMAGE_TILING_LINEAR && (formatProperties.linearTilingFeatures & formatFeatures) == formatFeatures)
		{
			return format;
		}
		else if (imageTiling == VK_IMAGE_TILING_OPTIMAL && (formatProperties.optimalTilingFeatures & formatFeatures) == formatFeatures)
		{
			return format;
		}
	}

	REPORT_ERROR("Failed to find suitable format!", __FILE__, __LINE__);
	return VkFormat::VK_FORMAT_UNDEFINED;
}

void createBuffer(
	const VkPhysicalDevice & physDev,
	const VkDevice & logicDev,
	VkDeviceSize size,
	VkBufferUsageFlags usage,
	VkMemoryPropertyFlags memoryProperties,
	VkBuffer * buffer,
	VkDeviceMemory * bufferDeviceMemory,
	VkAllocationCallbacks * pAllocator
	)
{
	if (buffer == nullptr || bufferDeviceMemory == nullptr)
	{
		REPORT_WARNING("Wrong data supplied for createBuffer!", __FILE__, __LINE__);
	}

	VkBufferCreateInfo bufferCreateInfo = {};
	bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferCreateInfo.size = size;
	bufferCreateInfo.usage = usage;
	bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

	if (vkCreateBuffer(logicDev, &bufferCreateInfo, pAllocator, buffer) != VK_SUCCESS)
	{
		REPORT_ERROR("Failed to create triangle vertex buffer!", __FILE__, __LINE__);
	}

	VkMemoryRequirements memoryRequirements;
	vkGetBufferMemoryRequirements(logicDev, *buffer, &memoryRequirements);

	VkMemoryAllocateInfo memoryAllocateInfo = {};
	memoryAllocateInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	memoryAllocateInfo.allocationSize = memoryRequirements.size;
	memoryAllocateInfo.memoryTypeIndex = findMemoryType(physDev, memoryRequirements.memoryTypeBits, memoryProperties);

	// TODO: vulkan memory allocation
	//	using fine-grained allocations for each buffer is bad, since devices could have very limited amount of
	//	allocations being live simultaneously. Instead, it is better to gather all of the buffers and make single
	//	allocation for them, then splitting the data between them.
	if (vkAllocateMemory(logicDev, &memoryAllocateInfo, pAllocator, bufferDeviceMemory) != VK_SUCCESS)
	{
		REPORT_ERROR("Failed to allocate triangle vertex buffer memory!", __FILE__, __LINE__);
	}

	// Offset (currently simply 0) needs to be divisible by memoryRequirements.alignment
	vkBindBufferMemory(logicDev, *buffer, *bufferDeviceMemory, 0);
}

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
	)
{
	if (image == nullptr || imageDeviceMemory == nullptr)
	{
		REPORT_WARNING("Wrong data supplied for createImage!", __FILE__, __LINE__);
	}

	VkImageCreateInfo imageCreateInfo = {};
	imageCreateInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
	imageCreateInfo.extent.width = width;
	imageCreateInfo.extent.height = height;
	imageCreateInfo.extent.depth = 1;
	imageCreateInfo.mipLevels = 1;
	imageCreateInfo.arrayLayers = 1;
	imageCreateInfo.format = format;
	imageCreateInfo.tiling = tiling;
	imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	imageCreateInfo.usage = usage;
	imageCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
	imageCreateInfo.flags = 0;

	if (vkCreateImage(logicDev, &imageCreateInfo, pAllocator, image) != VK_SUCCESS)
	{
		REPORT_ERROR("Failed to create image!", __FILE__, __LINE__);
	}

	VkMemoryRequirements memoryRequirements;
	vkGetImageMemoryRequirements(logicDev, *image, &memoryRequirements);

	VkMemoryAllocateInfo memoryAllocateInfo = {};
	memoryAllocateInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	memoryAllocateInfo.allocationSize = memoryRequirements.size;
	memoryAllocateInfo.memoryTypeIndex = findMemoryType(physDev, memoryRequirements.memoryTypeBits, memoryProperties);

	if (vkAllocateMemory(logicDev, &memoryAllocateInfo, pAllocator, imageDeviceMemory) != VK_SUCCESS)
	{
		REPORT_ERROR("Failed to allocate image memory!", __FILE__, __LINE__);
	}

	vkBindImageMemory(logicDev, *image, *imageDeviceMemory, 0);
}

VkImageView createImageView2D(
	const VkDevice & logicDev,
	VkImage image,
	VkFormat format,
	VkImageAspectFlags imageAspectFlags,
	VkAllocationCallbacks * pAllocator
	)
{
	VkImageView imageView;

	VkImageViewCreateInfo imageViewCreateInfo = {};
	imageViewCreateInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	imageViewCreateInfo.image = image;
	imageViewCreateInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
	imageViewCreateInfo.format = format;

	imageViewCreateInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
	imageViewCreateInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
	imageViewCreateInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
	imageViewCreateInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;

	imageViewCreateInfo.subresourceRange.aspectMask = imageAspectFlags;
	imageViewCreateInfo.subresourceRange.baseMipLevel = 0;
	imageViewCreateInfo.subresourceRange.levelCount = 1;
	imageViewCreateInfo.subresourceRange.baseArrayLayer = 0;
	imageViewCreateInfo.subresourceRange.layerCount = 1;

	if (vkCreateImageView(logicDev, &imageViewCreateInfo, pAllocator, &imageView) != VK_SUCCESS)
	{
		REPORT_ERROR("Failed to create image view!", __FILE__, __LINE__);
	}

	return imageView;
}

// Zero offsets, covers bufferSize.width x bufferSize.height, min/max depth = 0.0/1.0
VkPipelineViewportStateCreateInfo getDefaultViewportStateCreateInfo(VkExtent2D bufferSize)
{
	static VkViewport viewport = {};
	viewport.x = 0.0f;
	viewport.y = 0.0f;
	viewport.width = (float)bufferSize.width;
	viewport.height = (float)bufferSize.height;
	viewport.minDepth = 0.0f;
	viewport.maxDepth = 1.0f;

	static VkRect2D scissor = {};
	scissor.offset = { 0, 0 };
	scissor.extent = bufferSize;

	VkPipelineViewportStateCreateInfo viewportStateCreateInfo = {};
	viewportStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
	viewportStateCreateInfo.viewportCount = 1;
	viewportStateCreateInfo.pViewports = &viewport;
	viewportStateCreateInfo.scissorCount = 1;
	viewportStateCreateInfo.pScissors = &scissor;

	return viewportStateCreateInfo;
}

// No DepthClamp, no RasterizerDiscard, PolygonMode=Fill, CullMode=Back, FrontFace=CW, no DepthBias, LineWidth=1.0
VkPipelineRasterizationStateCreateInfo getDefaultRasterizationStateCreateInfo()
{
	VkPipelineRasterizationStateCreateInfo pipelineRasterizationStateCreateInfo = {};
	pipelineRasterizationStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
	
	// controls whether to clamp the fragment’s depth values instead of clipping primitives to the z planes of the frustum, as described in spec "Primitive Clipping"
	pipelineRasterizationStateCreateInfo.depthClampEnable = VK_FALSE;
	// controls whether primitives are discarded immediately before the rasterization stage
	pipelineRasterizationStateCreateInfo.rasterizerDiscardEnable = VK_FALSE;

	pipelineRasterizationStateCreateInfo.polygonMode = VK_POLYGON_MODE_FILL;
	pipelineRasterizationStateCreateInfo.cullMode = VK_CULL_MODE_BACK_BIT;
	pipelineRasterizationStateCreateInfo.frontFace = VK_FRONT_FACE_CLOCKWISE;

	pipelineRasterizationStateCreateInfo.depthBiasEnable = VK_FALSE;
	pipelineRasterizationStateCreateInfo.depthBiasConstantFactor = 0.0f;
	pipelineRasterizationStateCreateInfo.depthBiasClamp = 0.0f;
	pipelineRasterizationStateCreateInfo.depthBiasSlopeFactor = 0.0f;

	pipelineRasterizationStateCreateInfo.lineWidth = 1.0f;

	return pipelineRasterizationStateCreateInfo;
}

// Per-fragment shading, 1 sample per fragment, no Alpha-To-Coverage
VkPipelineMultisampleStateCreateInfo getDefaultMultisampleStateCreateInfo()
{
	VkPipelineMultisampleStateCreateInfo pipelineMultisampleStateCreateInfo = {};
	pipelineMultisampleStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
	pipelineMultisampleStateCreateInfo.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
	
	// specifies that fragment shading executes per-sample if VK_TRUE, or per-fragment if VK_FALSE
	pipelineMultisampleStateCreateInfo.sampleShadingEnable = VK_FALSE;
	// the minimum fraction of sample shading, as described in spec "Sample Shading"
	pipelineMultisampleStateCreateInfo.minSampleShading = 1.0f;
	
	// bitmask of static coverage information that is ANDed with the coverage information generated during rasterization
	pipelineMultisampleStateCreateInfo.pSampleMask = nullptr;
	pipelineMultisampleStateCreateInfo.alphaToCoverageEnable = VK_FALSE;
	pipelineMultisampleStateCreateInfo.alphaToOneEnable = VK_FALSE;

	return pipelineMultisampleStateCreateInfo;
}

// WriteMask=RGBA, no Blend, Color/Alpha Blend: Src=1 Dst=0 Op=add
VkPipelineColorBlendAttachmentState getDefaultColorBlendAttachmentState()
{
	VkPipelineColorBlendAttachmentState pipelineColorBlendAttachmentState = {};
	pipelineColorBlendAttachmentState.blendEnable = VK_FALSE;
	pipelineColorBlendAttachmentState.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
	pipelineColorBlendAttachmentState.dstColorBlendFactor = VK_BLEND_FACTOR_ZERO;
	pipelineColorBlendAttachmentState.colorBlendOp = VK_BLEND_OP_ADD;
	pipelineColorBlendAttachmentState.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
	pipelineColorBlendAttachmentState.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
	pipelineColorBlendAttachmentState.alphaBlendOp = VK_BLEND_OP_ADD;

	pipelineColorBlendAttachmentState.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

	return pipelineColorBlendAttachmentState;
}

// no LogicOp, LogicOp=Copy, BlendConst=(0,0,0,0), !no attachments!
VkPipelineColorBlendStateCreateInfo getDefaultColorBlendStateCreateInfo()
{
	VkPipelineColorBlendStateCreateInfo pipelineColorBlendStateCreateInfo = {};
	pipelineColorBlendStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
	pipelineColorBlendStateCreateInfo.logicOpEnable = VK_FALSE;
	pipelineColorBlendStateCreateInfo.logicOp = VK_LOGIC_OP_COPY;
	pipelineColorBlendStateCreateInfo.attachmentCount = 0;
	pipelineColorBlendStateCreateInfo.pAttachments = nullptr;
	pipelineColorBlendStateCreateInfo.blendConstants[0] = 0.0f;
	pipelineColorBlendStateCreateInfo.blendConstants[1] = 0.0f;
	pipelineColorBlendStateCreateInfo.blendConstants[2] = 0.0f;
	pipelineColorBlendStateCreateInfo.blendConstants[3] = 0.0f;

	return pipelineColorBlendStateCreateInfo;
}

// DepthTest=1, DepthWrite=1, DepthCompare=Less, no DepthBoundsTest, Min/MaxDepthBounds=0.0/1.0, no StencilTest
VkPipelineDepthStencilStateCreateInfo getDefaultDepthStencilStateCreateInfo()
{
	VkPipelineDepthStencilStateCreateInfo pipelineDepthStencilStateCreateInfo = {};
	pipelineDepthStencilStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	pipelineDepthStencilStateCreateInfo.depthTestEnable = VK_TRUE;
	pipelineDepthStencilStateCreateInfo.depthWriteEnable = VK_TRUE;
	pipelineDepthStencilStateCreateInfo.depthCompareOp = VK_COMPARE_OP_LESS;
	pipelineDepthStencilStateCreateInfo.depthBoundsTestEnable = VK_FALSE;
	pipelineDepthStencilStateCreateInfo.minDepthBounds = 0.0f;
	pipelineDepthStencilStateCreateInfo.maxDepthBounds = 1.0f;
	pipelineDepthStencilStateCreateInfo.stencilTestEnable = VK_FALSE;
	pipelineDepthStencilStateCreateInfo.front = {};	// StencilOpState
	pipelineDepthStencilStateCreateInfo.back = {};	// StencilOpState

	return pipelineDepthStencilStateCreateInfo;
}


}