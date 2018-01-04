#pragma once

#include <stdio.h>
#include <vector>
#include <algorithm>

#define NOMINMAX
#include <Windows.h>		// GetModuleHandle

#define VK_USE_PLATFORM_WIN32_KHR
#include <vulkan\vulkan.h>

#include "math\Vec2.h"
#include "math\Vec3.h"
#include "math\Vec4.h"

#include "math\Mat44.h"

#include "definitions.h"
#include "mesh.h"

namespace vulkan
{
	struct GlobalConstantsUBO
	{
		float time;
	};

	struct TransformUBO
	{
		math::Mat44 view;
		math::Mat44 proj;
	};

	struct ModelPushConstant
	{
		math::Mat44 model;
	};

	class Wrapper
	{
	public:

		int m_windowWidth = -1, m_windowHeight = -1;

		void setWindowSize(int width, int height)
		{
			m_windowWidth = width;
			m_windowHeight = height;
		}

		VkInstance m_vkInstance = VK_NULL_HANDLE;

		VkSurfaceKHR m_vkPresentableSurface = VK_NULL_HANDLE;

		struct VulkanPhysicalDeviceData
		{
			VkPhysicalDevice vkHandle = VK_NULL_HANDLE;
			VkPhysicalDeviceFeatures deviceFeatures;
			VkPhysicalDeviceProperties deviceProperties;

			std::vector<const char *> requiredExtensionNamesList;
			std::vector<VkExtensionProperties> supportedExtensionsProps;

			struct SurfaceInfo
			{
				VkSurfaceCapabilitiesKHR capabilities;
				std::vector<VkSurfaceFormatKHR> formats;
				std::vector<VkPresentModeKHR> presentModes;
			};

			SurfaceInfo surfaceInfo; 
		};
		VulkanPhysicalDeviceData m_vkPhysicalDeviceData;

		struct VulkanLogicalDeviceData
		{
			VkDevice vkHandle = VK_NULL_HANDLE;

			VkQueue graphicsQueue = VK_NULL_HANDLE;
			VkQueue presentingQueue = VK_NULL_HANDLE;

			uint32_t graphicsQueueFamilyIndex = 0xFFffFFff;
			uint32_t presentingQueueFamilyIndex = 0xFFffFFff;
		};
		VulkanLogicalDeviceData m_vkLogicalDeviceData;

		VkPhysicalDevice getPhysicalDeviceHandle() const { return m_vkPhysicalDeviceData.vkHandle; }
		VkDevice getLogicalDeviceHandle() const { return m_vkLogicalDeviceData.vkHandle; }

		VkQueue getGraphicsQueue() const { return m_vkLogicalDeviceData.graphicsQueue; }
		VkQueue getPresentingQueue() const { return m_vkLogicalDeviceData.presentingQueue; }

		VkAllocationCallbacks * getVkAllocator() { return nullptr; }

		// Vulkan-specific projection matrix
		void projPerspective(float fovRad, float aspect, float zNear, float zFar, float width, float height, math::Mat44 * projection);

		struct VulkanSwapchainData
		{
			VkSwapchainKHR vkHandle = VK_NULL_HANDLE;
			VkFormat format;
			VkFormatProperties formatProps;

			// Screenshots
			VkFormat captureFormat;
			VkFormatProperties captureFormatProps;

			VkExtent2D extent;
			VkColorSpaceKHR colorSpace;
			uint32_t imageIndexInSwapchain;
			std::vector<VkImage> images;
			std::vector<VkImageView> imageViews;
			std::vector<VkFramebuffer> framebuffers;
			bool supportsCapture;
		};
		VulkanSwapchainData m_vkSwapchainData;

		// Default scene depth-buffer (used in z-prepass and final shading)
		VkFormat m_vkDepthFormat;
		VkImage m_vkDepthImage = VK_NULL_HANDLE;
		VkDeviceMemory m_vkDepthDeviceMemory = VK_NULL_HANDLE;
		VkImageView m_vkDepthImageView = VK_NULL_HANDLE;

		/* Shadowmap generation and z-prepass */

		// Descriptor set layout defines set of resources for a *shader*;
		//	i.e. how much uniforms it has, how much storage buffers, samplers, push constants, etc.
		// This one is for the depth-only passes
		VkDescriptorSetLayout m_vkDepthOnlyShaderDescriptorSetLayout = VK_NULL_HANDLE;

		// Pipeline: Depth-only passes (e.g. shadowmap, z-prepass)
		VkPipelineLayout m_vkDepthOnlyPipelineLayout = VK_NULL_HANDLE;
		VkPipeline m_vkDepthOnlyGraphicsPipeline = VK_NULL_HANDLE;

		// Render pass: Shadowmap generation
		VkRenderPass m_vkShadowMapGenRenderPass = VK_NULL_HANDLE;

		// Descriptor sets
		VkDescriptorSet m_vkShadowMapGenDescriptorSet = VK_NULL_HANDLE;

		// Attachment: Shadowmap depth-buffer, used as attachment in shadowmap generation, and as shader resource in final shading
		VkFormat m_vkShadowMapDepthFormat;
		VkImage m_vkShadowMapDepthImage = VK_NULL_HANDLE;
		VkDeviceMemory m_vkShadowMapDepthDeviceMemory = VK_NULL_HANDLE;
		VkImageView m_vkShadowMapDepthImageView = VK_NULL_HANDLE;

		VkFramebuffer m_vkShadowMapGenFramebuffer = VK_NULL_HANDLE;

		// ZPrePass
		VkRenderPass m_vkZPrePassRenderPass = VK_NULL_HANDLE;

		VkDescriptorSet m_vkZPrePassDescriptorSet = VK_NULL_HANDLE;

		VkFramebuffer m_vkZPrePassFramebuffer = VK_NULL_HANDLE;

		/* Forward rendering (mesh & debug) */

		VkDescriptorSetLayout m_vkFwdShadingDescriptorSetLayout = VK_NULL_HANDLE;

		// General mesh rendering
		VkPipelineLayout m_vkFwdShadingPipelineLayout = VK_NULL_HANDLE;
		VkPipeline m_vkFwdShadingGraphicsPipeline = VK_NULL_HANDLE;

		// Debug lines rendering
		VkPipelineLayout m_vkDebugVisPipelineLayout = VK_NULL_HANDLE;
		VkPipeline m_vkDebugVisGraphicsPipeline = VK_NULL_HANDLE;

		VkRenderPass m_vkRenderPass = VK_NULL_HANDLE;

		VkDescriptorSet m_vkDescriptorSet = VK_NULL_HANDLE;

		// We don't have post-processing, so final shading uses VkFramebuffer of the swapchain, created for each swapchain image,
		//	and all are stored inside the VulkanSwapchainData structure

		/* Others */

		// Samplers
		// General texture sampler
		VkSampler m_vkTextureSampler = VK_NULL_HANDLE;
		// Sampler for HW PCF
		VkSampler m_vkDepthTextureSampler = VK_NULL_HANDLE;

		// Hardcoded triangle mesh (fullscreen quad)
		int m_vkTriangleVerticesCount = -1;
		VkBuffer m_vkTriangleVertexBuffer;
		VkDeviceMemory m_vkTriangleVertexBufferDeviceMemory;

		int m_vkTriangleIndicesCount = -1;
		VkIndexType m_vkTriangleIndexBufferType;
		VkBuffer m_vkTriangleIndexBuffer = VK_NULL_HANDLE;
		VkDeviceMemory m_vkTriangleIndexBufferDeviceMemory = VK_NULL_HANDLE;

		// Semaphores
		VkSemaphore m_vkSemaphoreImageAvailable = VK_NULL_HANDLE;
		VkSemaphore m_vkSemaphoreZPrePassFinished = VK_NULL_HANDLE;
		VkSemaphore m_vkSemaphoreShadowMapGenFinished = VK_NULL_HANDLE;
		VkSemaphore m_vkSemaphoreRenderFinished = VK_NULL_HANDLE;


		std::vector<const char *> m_requiredExtensionNamesList;
		std::vector<VkExtensionProperties> m_supportedExtensionsProps;

		bool m_debugCallbackInitialized = false;
		VkDebugReportCallbackEXT m_debugCallbackDesc = VK_NULL_HANDLE;
		std::vector<const char *> m_requiredInstanceValidationLayerNamesList;
		std::vector<const char *> m_requiredLogDevValidationLayerNamesList;

		void buildSupportedInstanceExtensionsList(bool printList = false);
		void buildRequiredInstanceExtensionsList(bool printList = false);

		bool m_enableValidationLayers = false;
		bool initValidationLayers(bool areValidationLayersEnabled = false);

		void initDebugCallback(PFN_vkDebugReportCallbackEXT debugCallback);
		void deinitDebugCallback();

		void initInstance();
		void deinitInstance();

		// TODO: implement through getQueueFamilyIndex (request idx if required and check if it's valid)
		static bool checkQueuesPresence(const VkPhysicalDevice & physDev, const VkSurfaceKHR & surface, bool needsGraphics, bool needsPresent, bool needsMemoryTransfer, bool needsCompute);

		static VulkanPhysicalDeviceData::SurfaceInfo queryDeviceSurfaceInfo(const VkPhysicalDevice & physDev, const VkSurfaceKHR & surface);

		static bool checkPhysicalDevice(const VkPhysicalDevice & physDev, const std::vector<const char *> & requiredExtensionNamesList, const VkSurfaceKHR & surface);

		void selectPhysicalDevice();

		void initLogicalDevice();
		void deinitLogicalDevice();

		void initWindowSurface(HWND hWnd);
		void deinitWindowSurface();

		void buildSupportedDeviceExtensionsList(bool printList = false);

		void buildRequiredDeviceExtensionsList(bool printList = false);

		static VkExtent2D selectPresentableSurfaceExtents(const VkSurfaceCapabilitiesKHR & capabilities, uint32_t w, uint32_t h);

		static VkSurfaceFormatKHR selectPresentableSurfaceFormat(const std::vector<VkSurfaceFormatKHR> & availableFormats);

		static VkPresentModeKHR selectPresentMode(const std::vector<VkPresentModeKHR> & availablePresentModes);

		void initSwapchain();
		void deinitSwapchain();

		void reinitSwapchain();

		void storeSwapchainImage(const wchar_t * filename);

		void onWindowResize(int width, int height)
		{
			if (width == 0 || height == 0)
				return;

			setWindowSize(width, height);
			reinitSwapchain();
		}

		PFN_vkDebugReportCallbackEXT m_vkDebugCallback = nullptr;
		void setDebugCallback(PFN_vkDebugReportCallbackEXT debugCallback)
		{
			m_vkDebugCallback = debugCallback;
		}
		PFN_vkDebugReportCallbackEXT getDebugCallback() const
		{
			return m_vkDebugCallback;
		}

		std::vector<VkShaderModule> m_vkShaderModules;
		VkShaderModule initShaderModule(const std::vector<char> & shaderByteCode);
		void deinitShaderModules();

		void initShadowMapGenRenderPass();
		void deinitShadowMapGenRenderPass();

		void initZPrePassRenderPass();
		void deinitZPrePassRenderPass();

		void initRenderPass();
		void deinitRenderPass();

		static void getVertexInputDescriptions(VkVertexInputBindingDescription * bindingDescr, VkVertexInputAttributeDescription attribsDescr[], int numAttribs = 3);
		static void getLinePointInputDescriptions(VkVertexInputBindingDescription * bindingDescr, VkVertexInputAttributeDescription attribsDescr[], int numAttribs = 3);

		// Depth-only (z-prepass, shadowmap) pipeline state
		void initDepthOnlyPipelineState();
		void deinitDepthOnlyPipelineState();

		// Forward (mesh & debug)
		void initPipelineState();
		void deinitPipelineState();

		void initDebugPipelineState();
		void deinitDebugPipelineState();

		// TODO: probably enhance/reuse TextureResource init/deinit
		void initShadowMapDepthBuffer();
		void deinitShadowMapDepthBuffer();

		void initShadowMapGenFramebuffers();
		void deinitShadowMapGenFramebuffers();

		void initZPrePassFramebuffers();
		void deinitZPrePassFramebuffers();

		void initSwapchainFramebuffers();
		void deinitSwapchainFramebuffers();

		// Command pool is per-thread, in case of multi-threaded scene generation - create as many command pools as there are threads
		VkCommandPool m_vkMainThreadCommandPool = VK_NULL_HANDLE;

		VkCommandBuffer m_vkShadowMapGenCommandBuffer = VK_NULL_HANDLE;
		VkCommandBuffer m_vkZPrePassCommandBuffer = VK_NULL_HANDLE;

		std::vector<VkCommandBuffer> m_vkMainThreadCommandBuffers;
		std::vector<VkCommandBuffer> m_vkMainThreadSwapchainCommandBuffers;

		void initCommandPool();
		void deinitCommandPool();

		void initDepthBuffer();
		void deinitDepthBuffer();

		VkCommandPool getTransientCommandPool();
		VkCommandBuffer beginTransientCommandBuffer();
		void endTransientCommandBuffer(const VkCommandBuffer & transientCommandBuffer);

		void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);

		void insertImageMemoryBarrierBuf(
			const VkCommandBuffer & commandBuffer,
			VkImage image,
			VkAccessFlags srcAccessMask,
			VkAccessFlags dstAccessMask,
			VkPipelineStageFlags srcSyncStageMask,
			VkPipelineStageFlags dstSyncStageMask,
			VkDependencyFlags dependencyFlags,
			VkImageLayout oldImageLayout,
			VkImageLayout newImageLayout,
			const VkImageSubresourceRange & subresourceRange
			);
		// This function effectively creates transient cmd buf, inserts image barrier to it, and submits it
		void transitionImageLayout(
			VkImage image,
			VkFormat format,
			VkImageLayout oldImageLayout,
			VkImageLayout newImageLayout
			);
		// Image layout should be transitioned to VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL
		void copyBufferToImage(uint32_t width, uint32_t height, VkBuffer buffer, VkImage image);

		struct TextureResource
		{
			uint32_t width, height;
			VkFormat imageFormat;
			VkImage image = VK_NULL_HANDLE;
			VkDeviceMemory imageDeviceMemory = VK_NULL_HANDLE;
			VkImageView imageView = VK_NULL_HANDLE;
		};

		std::vector<TextureResource *> m_textureResources;
		TextureResource * createTextureResource(uint32_t width, uint32_t height, void * data)
		{
			// TODO: could be replaced with pool
			TextureResource * newTextureResource = new TextureResource;

			initTextureResource(newTextureResource, width, height, data);

			m_textureResources.push_back(newTextureResource);
			return newTextureResource;
		}
		void destroyTextureResource(TextureResource * textureResource)
		{
			delete textureResource;
		}

		void initTextureResource(TextureResource * textureResource, uint32_t width, uint32_t height, void * data);
		void deinitTextureResource(TextureResource * textureResource);

		TextureResource m_albedoTex;
		void initAlbedoTexResource();
		void deinitAlbedoTexResource();

		TextureResource m_lightProjTex;
		void initLightProjTexResource();
		void deinitLightProjTexResource();

		void initTextureSampler();
		void deinitTextureSampler();

		void initDepthTextureSampler();
		void deinitDepthTextureSampler();

		void initFSQuadBuffers();
		void deinitFSQuadBuffers();

		// Shadowmap parameters
		uint32_t m_smWidth = 1024, m_smHeight = 1024;
		// Debug (immediate-mode-like) triangles
		std::vector<Vertex> m_debugTris;
		Mesh m_debugTrisMesh;

		// Debug (immediate-mode-like) lines
		std::vector<LinePoint> m_debugLines;
		Mesh m_debugLinesMesh;

		std::vector<Mesh *> m_meshes;
		Mesh * createMesh()
		{
			// TODO: could be replaced with pool
			Mesh * newMesh = new Mesh;
			newMesh->init(this);
			m_meshes.push_back(newMesh);
			return newMesh;
		}
		void destroyMesh(Mesh * mesh)
		{
			delete mesh;
		}

		void initDepthOnlyDescriptorSetLayout();
		void deinitDepthOnlyDescriptorSetLayout();

		void initDescriptorSetLayout();
		void deinitDescriptorSetLayout();

		VkBuffer m_vkShadowMapGenTransformUBOBuffer = VK_NULL_HANDLE;
		VkDeviceMemory m_vkShadowMapGenTransformUBOBufferDeviceMemory = VK_NULL_HANDLE;
		void initShadowMapGenTransformUBO();
		void deinitShadowMapGenTransformUBO();

		VkBuffer m_vkGlobalConstantsUBOBuffer = VK_NULL_HANDLE;
		VkDeviceMemory m_vkGlobalConstantsUBOBufferDeviceMemory = VK_NULL_HANDLE;
		void initGlobalConstantsUBO();
		void deinitGlobalConstantsUBO();

		VkBuffer m_vkTransformUBOBuffer = VK_NULL_HANDLE;
		VkDeviceMemory m_vkTransformUBOBufferDeviceMemory = VK_NULL_HANDLE;
		void initTransformUBO();
		void deinitTransformUBO();

		VkDescriptorPool m_vkDescriptorPool = VK_NULL_HANDLE;
		void initDescriptorPool();
		void deinitDescriptorPool();

		void initShadowMapGenDescriptorSet();
		void updateShadowMapGenDescriptorSetBindings();
		void deinitShadowMapGenDescriptorSet();

		void initZPrePassDescriptorSet();
		void updateZPrePassDescriptorSetBindings();
		void deinitZPrePassDescriptorSet();

		void initDescriptorSet();
		void updateDescriptorSetBindings();
		void deinitDescriptorSet();

		void buildShadowMapGenCommandBuffer();
		void updateShadowMapGenCommandBuffers();
		void destroyShadowMapGenCommandBuffer();

		void buildZPrePassCommandBuffer();
		void updateZPrePassCommandBuffers();
		void destroyZPrePassCommandBuffer();

		void buildSecondaryCommandBuffers();
		void updateSecondaryCommandBuffers(const VkCommandBufferInheritanceInfo & commandBufferInheritanceInfo);
		void destroySecondaryCommandBuffers();

		void buildSwapchainCommandBuffers();
		void updateSwapchainCommandBuffers(const VkFramebuffer & framebuffer);
		void destroySwapchainCommandBuffers();

		void initSemaphores();
		void deinitSemaphores();

		HWND m_hWnd = nullptr;
		void init(HWND hWnd, int width, int height);
		void deinit();

		bool m_captureRequested = false;
		void requestCapture(bool captureRequested) { m_captureRequested = captureRequested; }

		math::Mat34 m_viewMatrix;
		math::Mat34 & getViewMatrix() { return m_viewMatrix; }
		const math::Mat34 & getViewMatrix() const { return m_viewMatrix; }

		double m_elapsedTimeMS = 0.0, m_dtMS = 0.0;
		double getElapsedTime() const { return m_elapsedTimeMS; }
		void increaseDTime(double dtMS) { m_dtMS += dtMS; }

		struct PerspProjParams
		{
			float fovRad;
			float zNear;
			float zFar;
			float width, height;
			float aspect;
		};
		void setPerspectiveProjectionParameters(float fovRad, float aspect, float zNear, float zFar, float width, float height, PerspProjParams * params)
		{
			params->fovRad = fovRad;
			params->width = width;
			params->height = height;
			params->aspect = aspect;
			params->zNear = zNear;
			params->zFar = zFar;
		}

		PerspProjParams m_viewPerspParameters;
		void setViewPerspProjParams(float fovRad, float aspect, float zNear, float zFar, float width, float height)
		{
			setPerspectiveProjectionParameters(fovRad, aspect, zNear, zFar, width, height, &m_viewPerspParameters);
		}
		void getViewPerspProjParams(float * fovRad, float * aspect, float * zNear, float * zFar, float * width, float * height) const
		{
			*fovRad = m_viewPerspParameters.fovRad;
			*aspect = m_viewPerspParameters.aspect;
			*zNear = m_viewPerspParameters.zNear;
			*zFar = m_viewPerspParameters.zFar;
			*width = m_viewPerspParameters.width;
			*height = m_viewPerspParameters.height;
		}

		float getViewFOV() const
		{
			return m_viewPerspParameters.fovRad;
		}
		void setViewFOV(float fovRad)
		{
			m_viewPerspParameters.fovRad = fovRad;
		}

		PerspProjParams m_lightPerspParameters;
		void setLightPerspProjParams(float fovRad, float aspect, float zNear, float zFar, float width, float height)
		{
			if (aspect != 1.0f)
			{
				// TODO: warning
				printf("Non-unit aspect for light projection matrix is not supprted!\n");
				aspect = 1.0f;
			}

			setPerspectiveProjectionParameters(fovRad, aspect, zNear, zFar, width, height, &m_lightPerspParameters);
		}
		void getLightPerspProjParams(float * fovRad, float * aspect, float * zNear, float * zFar, float * width, float * height) const
		{
			*fovRad = m_lightPerspParameters.fovRad;
			*aspect = m_lightPerspParameters.aspect;
			*zNear = m_lightPerspParameters.zNear;
			*zFar = m_lightPerspParameters.zFar;
			*width = m_lightPerspParameters.width;
			*height = m_lightPerspParameters.height;
		}

		math::Mat44 m_lightViewMatrix;
		void setLightViewMatrix(const math::Mat44 & lightViewMatrix) { m_lightViewMatrix = lightViewMatrix; }
		const math::Mat44 & getLightViewMatrix() const { return m_lightViewMatrix; }

		// All rendering-related updates should go strickly after this function
		void beginFrame();

		void update();
		void render();
	};

}