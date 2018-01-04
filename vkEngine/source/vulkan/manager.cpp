#include <fstream>
#include <assert.h>
#include <ctime>
#include <chrono>

#include "math/AuxMath.h"
#include "formats/tga.h"
#include "vulkan/manager.h"
#include "vulkan/helpers.h"

#include "shader_bindings.h"

#define SCENE_QUAD	1
#define SCENE_MESH	2
#define SCENE SCENE_MESH

#define DBG_FORCE_SYNCS		0

#if (DBG_FORCE_SYNCS != 0)
#define DBG_RENDER_FORCED_SYNC_IF_REQ(device) vkDeviceWaitIdle(device);
#else
#define DBG_RENDER_FORCED_SYNC_IF_REQ(device)
#endif

#define EXPLICIT_DESCRIPTORS_FREE	0

namespace vulkan
{
	using namespace math;

	static std::vector<char> readShaderFile(const std::string & filename)
	{
		std::ifstream file(filename, std::ios::ate | std::ios::binary);
		if (!file.is_open())
		{
			printf("Shader file %s not found!\n", filename.c_str());
			return std::vector<char>();
		}

		size_t fileSize = (size_t)file.tellg();
		std::vector<char> buffer(fileSize);
		file.seekg(0);
		file.read(buffer.data(), fileSize);
		file.close();

		return buffer;
	}

	void Wrapper::projPerspective(float fovRad, float aspect, float zNear, float zFar, float width, float height, Mat44 * projection)
	{
		projection->zero();

		Vec2 scale;
		scale.x = width * 1.0f / tanf(fovRad / 2);
		scale.y = height * aspect * scale.x;

		float zDiff = zFar - zNear;

		// Note the minus sign for the y-scale. This is due to Vulkan's normalized device coordinate system
		//	being right handed while having Z looking away from the viewer. This would require either X to go left,
		//	or for Y to go down.

		projection->_00 = scale.x;
		projection->_11 = -scale.y;
		projection->_22 = -(zFar + zNear) / zDiff;
		projection->_23 = -2 * zFar * zNear / zDiff;
		projection->_32 = -1.0f;
	}

	void Wrapper::buildSupportedInstanceExtensionsList(bool printList)
	{
		uint32_t extensionCount = 0;

		// Rare edge case where between two calls, list could be altered,
		//	result could be VK_INCOMPLETE, this process should be continued until status is not VK_INCOMPLETE
		VkResult result;
		do
		{
			result = vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr);

			m_supportedExtensionsProps.resize(extensionCount);
			result = vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, m_supportedExtensionsProps.data());
		} while (result == VK_INCOMPLETE);

		if (printList)
		{
			printf("%d instance extensions supported\n", extensionCount);
			for (const VkExtensionProperties & extensionProp : m_supportedExtensionsProps)
			{
				printf("  %s\n", extensionProp.extensionName);
			}
		}
	}

	void Wrapper::buildRequiredInstanceExtensionsList(bool printList)
	{
		const char * requiredExtensions[] =
		{
			VK_KHR_SURFACE_EXTENSION_NAME,			//"VK_KHR_surface",
			VK_KHR_WIN32_SURFACE_EXTENSION_NAME		//"VK_KHR_win32_surface"
		};
		const uint32_t requiredExtensionCount = (uint32_t)(sizeof(requiredExtensions) / sizeof(const char *));

		for (uint32_t i = 0; i < requiredExtensionCount; ++i)
		{
			m_requiredExtensionNamesList.push_back(requiredExtensions[i]);
		}
		m_requiredExtensionNamesList.push_back(VK_EXT_DEBUG_REPORT_EXTENSION_NAME);

		if (printList)
		{
			printf("Required instance extensions:\n");
			for (uint32_t i = 0; i < requiredExtensionCount; ++i)
			{
				printf("  %s\n", requiredExtensions[i]);
			}
		}
	}

	bool Wrapper::initValidationLayers(bool areValidationLayersEnabled)
	{
		bool enableDebugCallback = false;
		bool enableValidationLayers = false;

		m_requiredInstanceValidationLayerNamesList.resize(0);
		m_requiredLogDevValidationLayerNamesList.resize(0);

		if (areValidationLayersEnabled)
		{
			enableDebugCallback = true;
			enableValidationLayers = true;
			m_requiredInstanceValidationLayerNamesList.push_back("VK_LAYER_LUNARG_standard_validation");
			m_requiredLogDevValidationLayerNamesList.push_back("VK_LAYER_LUNARG_standard_validation");
		}

		uint32_t layerCount = 0;
		std::vector<VkLayerProperties> availableLayers;

		// Rare edge case where between two calls, list could be altered,
		//	result could be VK_INCOMPLETE, this process should be continued until status is not VK_INCOMPLETE
		VkResult result;
		do
		{
			result = vkEnumerateInstanceLayerProperties(&layerCount, nullptr);

			availableLayers.resize(layerCount);
			result = vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());
		} while (result == VK_INCOMPLETE);

		for (const char * layerName : m_requiredInstanceValidationLayerNamesList)
		{
			bool layerFound = false;

			for (const auto& layerProperties : availableLayers)
			{
				if (strcmp(layerName, layerProperties.layerName) == 0)
				{
					layerFound = true;
					break;
				}
			}

			if (!layerFound)
			{
				printf("Required instance validation layer %s not found!\n", layerName);
			}
		}

		for (const char * layerName : m_requiredLogDevValidationLayerNamesList)
		{
			bool layerFound = false;

			for (const auto& layerProperties : availableLayers)
			{
				if (strcmp(layerName, layerProperties.layerName) == 0)
				{
					layerFound = true;
					break;
				}
			}

			if (!layerFound)
			{
				printf("Required logical device validation layer %s not found!\n", layerName);
			}
		}

		return enableDebugCallback;
	}

	void Wrapper::initDebugCallback(PFN_vkDebugReportCallbackEXT debugCallback)
	{
		VkDebugReportCallbackCreateInfoEXT debugCallbackCreateInfo = {};
		debugCallbackCreateInfo.sType = VK_STRUCTURE_TYPE_DEBUG_REPORT_CALLBACK_CREATE_INFO_EXT;
		debugCallbackCreateInfo.flags = VK_DEBUG_REPORT_ERROR_BIT_EXT | VK_DEBUG_REPORT_WARNING_BIT_EXT;
		debugCallbackCreateInfo.pfnCallback = debugCallback;

		auto vkCreateDebugReportCallbackEXT_pfn = (PFN_vkCreateDebugReportCallbackEXT)vkGetInstanceProcAddr(m_vkInstance, "vkCreateDebugReportCallbackEXT");
		if (vkCreateDebugReportCallbackEXT_pfn != nullptr)
		{
			vkCreateDebugReportCallbackEXT_pfn(m_vkInstance, &debugCallbackCreateInfo, getVkAllocator(), &m_debugCallbackDesc);
			m_debugCallbackInitialized = true;
		}
		else
		{
			printf("Debug callback extension not present!\n");
		}
	}
	void Wrapper::deinitDebugCallback()
	{
		if (m_debugCallbackInitialized)
		{
			auto vkDestroyDebugReportCallbackEXT_pfn = (PFN_vkDestroyDebugReportCallbackEXT)vkGetInstanceProcAddr(m_vkInstance, "vkDestroyDebugReportCallbackEXT");
			if (vkDestroyDebugReportCallbackEXT_pfn != nullptr)
			{
				vkDestroyDebugReportCallbackEXT_pfn(m_vkInstance, m_debugCallbackDesc, getVkAllocator());
				m_debugCallbackInitialized = false;
			}
			else
			{
				// TODO: error
				printf("Debug callback extension not present!\n");
			}
		}
	}

	void Wrapper::initInstance()
	{
		VkApplicationInfo appInfo = {};
		appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
		appInfo.pApplicationName = "Hello Triangle";
		appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
		appInfo.pEngineName = "No Engine";
		appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
		appInfo.apiVersion = VK_API_VERSION_1_0;

		VkInstanceCreateInfo createInfo = {};
		createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
		createInfo.pApplicationInfo = &appInfo;
		createInfo.enabledExtensionCount = (uint32_t)m_requiredExtensionNamesList.size();
		createInfo.ppEnabledExtensionNames = m_requiredExtensionNamesList.data();
		if (m_requiredInstanceValidationLayerNamesList.size() > 0)
		{
			createInfo.enabledLayerCount = static_cast<uint32_t>(m_requiredInstanceValidationLayerNamesList.size());
			createInfo.ppEnabledLayerNames = m_requiredInstanceValidationLayerNamesList.data();
		}
		else
		{
			createInfo.enabledLayerCount = 0;
			createInfo.ppEnabledLayerNames = nullptr;
		}

		VkResult status;
		if ((status = vkCreateInstance(&createInfo, getVkAllocator(), &m_vkInstance)) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create VkInstance: %d\n", (int)status);
		}
	}
	void Wrapper::deinitInstance()
	{
		vkDestroyInstance(m_vkInstance, getVkAllocator());
		m_vkInstance = VK_NULL_HANDLE;
	}

	// TODO: implement through getQueueFamilyIndex (request idx if required and check if it's valid)
	// static
	bool Wrapper::checkQueuesPresence(const VkPhysicalDevice & physDev, const VkSurfaceKHR & surface, bool needsGraphics, bool needsPresent, bool needsMemoryTransfer, bool needsCompute)
	{
		bool hasGraphics = false;
		bool hasMemoryTransfer = false;
		bool hasCompute = false;
		bool hasPresent = false;

		uint32_t queueFamilyCount = 0;
		vkGetPhysicalDeviceQueueFamilyProperties(physDev, &queueFamilyCount, nullptr);

		std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
		vkGetPhysicalDeviceQueueFamilyProperties(physDev, &queueFamilyCount, queueFamilies.data());

		for (size_t qIdx = 0, qIdxEnd = queueFamilies.size(); qIdx < qIdxEnd; ++qIdx)
		{
			const VkQueueFamilyProperties & queueFamily = queueFamilies[qIdx];

			if (queueFamily.queueCount > 0 && (queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT))
			{
				hasGraphics = true;
			}
			if (queueFamily.queueCount > 0 && (queueFamily.queueFlags & VK_QUEUE_TRANSFER_BIT))
			{
				hasMemoryTransfer = true;
			}
			if (queueFamily.queueCount > 0 && (queueFamily.queueFlags & VK_QUEUE_COMPUTE_BIT))
			{
				hasCompute = true;
			}

			VkBool32 isPresentSupported = false;
			vkGetPhysicalDeviceSurfaceSupportKHR(physDev, (uint32_t)qIdx, surface, &isPresentSupported);
			if (queueFamily.queueCount > 0 && isPresentSupported)
			{
				hasPresent = true;
			}
		}

		return
			(!needsGraphics || hasGraphics) && 
			(!needsPresent || hasPresent) && 
			(!needsMemoryTransfer || hasMemoryTransfer) && 
			(!needsCompute || hasCompute);
	};

	// static
	Wrapper::VulkanPhysicalDeviceData::SurfaceInfo Wrapper::queryDeviceSurfaceInfo(const VkPhysicalDevice & physDev, const VkSurfaceKHR & surface)
	{
		VulkanPhysicalDeviceData::SurfaceInfo deviceSurfaceInfo;

		vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physDev, surface, &deviceSurfaceInfo.capabilities);

		uint32_t formatCount;
		vkGetPhysicalDeviceSurfaceFormatsKHR(physDev, surface, &formatCount, nullptr);

		deviceSurfaceInfo.formats.resize(formatCount);
		vkGetPhysicalDeviceSurfaceFormatsKHR(physDev, surface, &formatCount, deviceSurfaceInfo.formats.data());

		uint32_t presentModeCount;
		vkGetPhysicalDeviceSurfacePresentModesKHR(physDev, surface, &presentModeCount, nullptr);

		deviceSurfaceInfo.presentModes.resize(presentModeCount);
		vkGetPhysicalDeviceSurfacePresentModesKHR(physDev, surface, &presentModeCount, deviceSurfaceInfo.presentModes.data());

		return deviceSurfaceInfo;
	}

	// static
	bool Wrapper::checkPhysicalDevice(const VkPhysicalDevice & physDev, const std::vector<const char *> & requiredExtensionNamesList, const VkSurfaceKHR & surface)
	{
		VkPhysicalDeviceProperties deviceProperties;
		vkGetPhysicalDeviceProperties(physDev, &deviceProperties);

		VkPhysicalDeviceFeatures deviceFeatures;
		vkGetPhysicalDeviceFeatures(physDev, &deviceFeatures);

		// Only allowed to run on physical GPUs
		//	(but maybe allow to run on CPUs as well? not important atm)
		if (deviceProperties.deviceType != VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU &&
			deviceProperties.deviceType != VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU)
		{
			return false;
		}
		if (!deviceFeatures.geometryShader)
		{
			return false;
		}

		// TODO:
		//	* prefer discrete over integrated
		//	* prefer devices with presentable queue == graphics queue (possibly increased performance?)
		if (!checkQueuesPresence(physDev, surface, true, true, false, false))
		{
			return false;
		}

		// Check for required extensions support
		std::vector<VkExtensionProperties> curDeviceSupportedExtensions;
		getGenericSupportedDeviceExtensionsList(physDev, &curDeviceSupportedExtensions);

		bool anyExtensionNotFound = false;
		for (const VkExtensionProperties & curDevProp : curDeviceSupportedExtensions)
		{
			bool extensionFound = false;
			for (const char * reqExtName : requiredExtensionNamesList)
			{
				if (strcmp(curDevProp.extensionName, reqExtName) == 0)
				{
					extensionFound = true;
					break;
				}
			}

			if (!extensionFound)
			{
				anyExtensionNotFound = true;
				break;
			}
		}

		if (!anyExtensionNotFound)
		{
			return false;
		}

		// Check for surface parameters support
		VulkanPhysicalDeviceData::SurfaceInfo deviceSurfaceInfo = queryDeviceSurfaceInfo(physDev, surface);
		if (deviceSurfaceInfo.formats.empty() || deviceSurfaceInfo.presentModes.empty())
		{
			return false;
		}

		return true;
	};

	void Wrapper::selectPhysicalDevice()
	{
		uint32_t physicalDeviceCount = 0;
		std::vector<VkPhysicalDevice> physicalDevices;

		// Rare edge case where between two calls, list could be altered,
		//	result could be VK_INCOMPLETE, this process should be continued until status is not VK_INCOMPLETE
		VkResult result;
		do
		{
			result = vkEnumeratePhysicalDevices(m_vkInstance, &physicalDeviceCount, nullptr);

			if (physicalDeviceCount == 0)
			{
				// TODO: error
				printf("No physical devices with Vulkan support found!\n");
			}

			physicalDevices.resize(physicalDeviceCount);
			result = vkEnumeratePhysicalDevices(m_vkInstance, &physicalDeviceCount, physicalDevices.data());
		} while (result == VK_INCOMPLETE);

		buildRequiredDeviceExtensionsList(true);

		for (const auto & physDev : physicalDevices)
		{
			if (checkPhysicalDevice(physDev, m_vkPhysicalDeviceData.requiredExtensionNamesList, m_vkPresentableSurface))
			{
				// We'll use the first device that meets our expectations
				//	TODO: prefer discrete GPU to the integrated GPU
				m_vkPhysicalDeviceData.vkHandle = physDev;
				break;
			}
		}

		if (m_vkPhysicalDeviceData.vkHandle == VK_NULL_HANDLE)
		{
			// TODO: error
			printf("No physical device meets the requirements!\n");
		}

		vkGetPhysicalDeviceFeatures(m_vkPhysicalDeviceData.vkHandle, &m_vkPhysicalDeviceData.deviceFeatures);
		vkGetPhysicalDeviceProperties(m_vkPhysicalDeviceData.vkHandle, &m_vkPhysicalDeviceData.deviceProperties);		

		printf("\n> GPU: %s\n\n", m_vkPhysicalDeviceData.deviceProperties.deviceName);

		// Fill in the physical device info
		buildSupportedDeviceExtensionsList(true);
	}

	void Wrapper::initLogicalDevice()
	{
		auto getQueueFamilyIndex = [](const VkPhysicalDevice & physDev, const VkQueueFlagBits & queueKindFlag) -> int
		{
			uint32_t queueFamilyCount = 0;
			vkGetPhysicalDeviceQueueFamilyProperties(physDev, &queueFamilyCount, nullptr);

			std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
			vkGetPhysicalDeviceQueueFamilyProperties(physDev, &queueFamilyCount, queueFamilies.data());

			for (size_t qIdx = 0, qIdxEnd = queueFamilies.size(); qIdx < qIdxEnd; ++qIdx)
			{
				const VkQueueFamilyProperties & queueFamily = queueFamilies[qIdx];

				if (queueFamily.queueCount > 0 && (queueFamily.queueFlags & queueKindFlag))
				{
					return (int)qIdx;
				}
			}

			return -1;
		};

		VkSurfaceKHR createdPresentableSurface = m_vkPresentableSurface;
		auto getPresentingQueueFamilyIndex = [&createdPresentableSurface](const VkPhysicalDevice & physDev) -> int
		{
			uint32_t queueFamilyCount = 0;
			vkGetPhysicalDeviceQueueFamilyProperties(physDev, &queueFamilyCount, nullptr);

			std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
			vkGetPhysicalDeviceQueueFamilyProperties(physDev, &queueFamilyCount, queueFamilies.data());

			for (size_t qIdx = 0, qIdxEnd = queueFamilies.size(); qIdx < qIdxEnd; ++qIdx)
			{
				const VkQueueFamilyProperties & queueFamily = queueFamilies[qIdx];

				VkBool32 isPresentSupported = false;
				vkGetPhysicalDeviceSurfaceSupportKHR(physDev, (uint32_t)qIdx, createdPresentableSurface, &isPresentSupported);
				if (queueFamily.queueCount > 0 && isPresentSupported)
				{
					return (int)qIdx;
				}
			}

			return -1;
		};

		VkPhysicalDeviceFeatures physicalDeviceFeatures = {};
		physicalDeviceFeatures.samplerAnisotropy = VK_TRUE;

		float queuePriority = 1.0f;
		int graphicsQueueFamilyIndex = getQueueFamilyIndex(m_vkPhysicalDeviceData.vkHandle, VK_QUEUE_GRAPHICS_BIT);
		int presentingQueueFamilyIndex = getPresentingQueueFamilyIndex(m_vkPhysicalDeviceData.vkHandle);

		std::vector<int> queuesRequired;
		queuesRequired.push_back(graphicsQueueFamilyIndex);
		if (presentingQueueFamilyIndex != graphicsQueueFamilyIndex)
			queuesRequired.push_back(presentingQueueFamilyIndex);

		std::vector<VkDeviceQueueCreateInfo> logicalDeviceQueueCreateInfos;
		for (const auto & qIdx : queuesRequired)
		{
			VkDeviceQueueCreateInfo logicalDeviceQueueCreateInfo = {};
			logicalDeviceQueueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
			logicalDeviceQueueCreateInfo.queueFamilyIndex = qIdx;
			logicalDeviceQueueCreateInfo.queueCount = 1;
			logicalDeviceQueueCreateInfo.pQueuePriorities = &queuePriority;
			logicalDeviceQueueCreateInfos.push_back(logicalDeviceQueueCreateInfo);
		}

		VkDeviceCreateInfo logicalDeviceCreateInfo = {};
		logicalDeviceCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
		logicalDeviceCreateInfo.pQueueCreateInfos = logicalDeviceQueueCreateInfos.data();
		logicalDeviceCreateInfo.queueCreateInfoCount = (uint32_t)logicalDeviceQueueCreateInfos.size();
		logicalDeviceCreateInfo.pEnabledFeatures = &physicalDeviceFeatures;
		logicalDeviceCreateInfo.enabledExtensionCount = (uint32_t)m_vkPhysicalDeviceData.requiredExtensionNamesList.size();
		logicalDeviceCreateInfo.ppEnabledExtensionNames = m_vkPhysicalDeviceData.requiredExtensionNamesList.data();
		if (m_requiredLogDevValidationLayerNamesList.size() > 0)
		{
			// Setting device layer names is deprecated and ignored actually
			logicalDeviceCreateInfo.enabledLayerCount = static_cast<uint32_t>(m_requiredLogDevValidationLayerNamesList.size());
			logicalDeviceCreateInfo.ppEnabledLayerNames = m_requiredLogDevValidationLayerNamesList.data();
		}
		else
		{
			logicalDeviceCreateInfo.enabledLayerCount = 0;
			logicalDeviceCreateInfo.ppEnabledLayerNames = nullptr;
		}

		if (vkCreateDevice(m_vkPhysicalDeviceData.vkHandle, &logicalDeviceCreateInfo, getVkAllocator(), &m_vkLogicalDeviceData.vkHandle) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create logical device!\n");
		}

		m_vkLogicalDeviceData.graphicsQueueFamilyIndex = graphicsQueueFamilyIndex;
		m_vkLogicalDeviceData.presentingQueueFamilyIndex = presentingQueueFamilyIndex;
		vkGetDeviceQueue(m_vkLogicalDeviceData.vkHandle, m_vkLogicalDeviceData.graphicsQueueFamilyIndex, 0, &m_vkLogicalDeviceData.graphicsQueue);
		vkGetDeviceQueue(m_vkLogicalDeviceData.vkHandle, m_vkLogicalDeviceData.presentingQueueFamilyIndex, 0, &m_vkLogicalDeviceData.presentingQueue);
	}
	void Wrapper::deinitLogicalDevice()
	{
		vkDestroyDevice(m_vkLogicalDeviceData.vkHandle, getVkAllocator());
		m_vkLogicalDeviceData.vkHandle = VK_NULL_HANDLE;
	}

	void Wrapper::initWindowSurface(HWND hWnd)
	{
		VkWin32SurfaceCreateInfoKHR createInfo = {};
		createInfo.sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR;
		createInfo.hwnd = hWnd;
		createInfo.hinstance = GetModuleHandle(nullptr);

#if 0
		auto vkCreateWin32SurfaceKHR_pfn = (PFN_vkCreateWin32SurfaceKHR)vkGetInstanceProcAddr(m_vkInstance, "vkCreateWin32SurfaceKHR");
		if (vkCreateWin32SurfaceKHR_pfn != nullptr)
		{
			if (vkCreateWin32SurfaceKHR_pfn(m_vkInstance, &createInfo, getVkAllocator(), &m_vkPresentableSurface) != VK_SUCCESS)
			{
				// TODO: error
				printf("Win32 surface creation failed!\n");
			}
		}
		else
		{
			// TODO: error
			printf("CreateWin32SurfaceKHR function is not found!\n");
		}
#else
		if (vkCreateWin32SurfaceKHR(m_vkInstance, &createInfo, getVkAllocator(), &m_vkPresentableSurface) != VK_SUCCESS)
		{
			// TODO: error
			printf("Win32 surface creation failed!\n");
		}
#endif
	}
	void Wrapper::deinitWindowSurface()
	{
		vkDestroySurfaceKHR(m_vkInstance, m_vkPresentableSurface, getVkAllocator());
		m_vkPresentableSurface = VK_NULL_HANDLE;
	}

	void Wrapper::buildSupportedDeviceExtensionsList(bool printList)
	{
		getGenericSupportedDeviceExtensionsList(m_vkPhysicalDeviceData.vkHandle, &m_vkPhysicalDeviceData.supportedExtensionsProps);

		if (printList)
		{
			printf("%d device extensions supported\n", (int)m_vkPhysicalDeviceData.supportedExtensionsProps.size());
			for (const VkExtensionProperties & extensionProp : m_vkPhysicalDeviceData.supportedExtensionsProps)
			{
				printf("  %s\n", extensionProp.extensionName);
			}
		}
	}

	void Wrapper::buildRequiredDeviceExtensionsList(bool printList)
	{
		const char * requiredExtensions[] =
		{
			VK_KHR_SWAPCHAIN_EXTENSION_NAME			//"VK_KHR_swapchain"
		};
		const uint32_t requiredExtensionCount = (uint32_t)(sizeof(requiredExtensions) / sizeof(const char *));

		for (uint32_t i = 0; i < requiredExtensionCount; ++i)
		{
			m_vkPhysicalDeviceData.requiredExtensionNamesList.push_back(requiredExtensions[i]);
		}

		if (printList)
		{
			printf("Required device extensions:\n");
			for (uint32_t i = 0; i < requiredExtensionCount; ++i)
			{
				printf("  %s\n", requiredExtensions[i]);
			}
		}
	}

	// static
	VkExtent2D Wrapper::selectPresentableSurfaceExtents(const VkSurfaceCapabilitiesKHR & capabilities, uint32_t w, uint32_t h)
	{
		// Special case: window manager doesn't care about extent being same as window size
		if (capabilities.currentExtent.width == std::numeric_limits<uint32_t>::max() &&
			capabilities.currentExtent.height == std::numeric_limits<uint32_t>::max())
		{
			VkExtent2D actualExtent = { w, h };

			actualExtent.width = std::max(capabilities.minImageExtent.width, std::min(capabilities.maxImageExtent.width, actualExtent.width));
			actualExtent.height = std::max(capabilities.minImageExtent.height, std::min(capabilities.maxImageExtent.height, actualExtent.height));

			return actualExtent;
		}		

		return capabilities.currentExtent;
	}

	// static
	VkSurfaceFormatKHR Wrapper::selectPresentableSurfaceFormat(const std::vector<VkSurfaceFormatKHR> & availableFormats)
	{
		VkSurfaceFormatKHR desiredFormat = { VK_FORMAT_B8G8R8A8_UNORM, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR };

		// Special case: device doesn't care about format selection
		if (availableFormats.size() == 1 && availableFormats[0].format == VK_FORMAT_UNDEFINED)
		{
			return desiredFormat;
		}

		for (const VkSurfaceFormatKHR & availableFormat : availableFormats)
		{
			if (availableFormat.format == desiredFormat.format && availableFormat.colorSpace == desiredFormat.colorSpace)
			{
				return desiredFormat;
			}
		}

		return availableFormats[0];
	}

	// static
	VkPresentModeKHR Wrapper::selectPresentMode(const std::vector<VkPresentModeKHR> & availablePresentModes)
	{
		for (const VkPresentModeKHR & availablePresentMode : availablePresentModes)
		{
			// Present as fast as possible (with tearing)
			if (availablePresentMode == VK_PRESENT_MODE_IMMEDIATE_KHR)
			{
				return availablePresentMode;
			}
			else if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR)
			{
				return availablePresentMode;
			}
			// We could select VK_PRESENT_MODE_MAILBOX_KHR first,
			//	but it facilitates discarding frames, which means potentially wasted effort
		}

		// VK_PRESENT_MODE_FIFO_KHR is guaranteed to be present by the spec
		return VK_PRESENT_MODE_FIFO_KHR;
	}

	void Wrapper::initSwapchain()
	{
		m_vkSwapchainData.supportsCapture = true;
		m_vkSwapchainData.captureFormat = VK_FORMAT_R8G8B8A8_UNORM;

		m_vkPhysicalDeviceData.surfaceInfo = queryDeviceSurfaceInfo(m_vkPhysicalDeviceData.vkHandle, m_vkPresentableSurface);

		VkExtent2D presentableSurfaceExtents = selectPresentableSurfaceExtents(m_vkPhysicalDeviceData.surfaceInfo.capabilities, (uint32_t)m_windowWidth, (uint32_t)m_windowHeight);
		VkSurfaceFormatKHR presentableSurfaceFormat = selectPresentableSurfaceFormat(m_vkPhysicalDeviceData.surfaceInfo.formats);
		VkPresentModeKHR presentMode = selectPresentMode(m_vkPhysicalDeviceData.surfaceInfo.presentModes);

		if (0)
		{
			printf("Initializing swapchain of size %dx%d\n", presentableSurfaceExtents.width, presentableSurfaceExtents.height);
		}

		uint32_t minImageCount = m_vkPhysicalDeviceData.surfaceInfo.capabilities.minImageCount;
		// maxImageCount == 0 means there's no limits other than the available memory
		if (m_vkPhysicalDeviceData.surfaceInfo.capabilities.maxImageCount != 0 && minImageCount > m_vkPhysicalDeviceData.surfaceInfo.capabilities.maxImageCount)
		{
			// TODO: warning
			printf("Swapchain max image count is limiting: requested %d, max is %d\n", minImageCount, m_vkPhysicalDeviceData.surfaceInfo.capabilities.maxImageCount);

			minImageCount = m_vkPhysicalDeviceData.surfaceInfo.capabilities.maxImageCount;
		}

		VkSwapchainCreateInfoKHR swapchainCreateInfo = {};
		swapchainCreateInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
		swapchainCreateInfo.surface = m_vkPresentableSurface;
		swapchainCreateInfo.minImageCount = minImageCount;
		swapchainCreateInfo.imageFormat = presentableSurfaceFormat.format;
		swapchainCreateInfo.imageColorSpace = presentableSurfaceFormat.colorSpace;
		swapchainCreateInfo.imageExtent = presentableSurfaceExtents;
		swapchainCreateInfo.imageArrayLayers = 1;
		swapchainCreateInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

		vkGetPhysicalDeviceFormatProperties(m_vkPhysicalDeviceData.vkHandle, presentableSurfaceFormat.format, &m_vkSwapchainData.formatProps);
		vkGetPhysicalDeviceFormatProperties(m_vkPhysicalDeviceData.vkHandle, m_vkSwapchainData.captureFormat, &m_vkSwapchainData.captureFormatProps);
		if (m_vkSwapchainData.supportsCapture)
		{
			if (m_vkSwapchainData.formatProps.optimalTilingFeatures & VK_FORMAT_FEATURE_BLIT_DST_BIT)
			{
				swapchainCreateInfo.imageUsage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
			}
			else
			{
				m_vkSwapchainData.supportsCapture = false;
			}
		}

		uint32_t queueFamilyIndices[] = { (uint32_t)m_vkLogicalDeviceData.graphicsQueueFamilyIndex, (uint32_t)m_vkLogicalDeviceData.presentingQueueFamilyIndex};
		if (queueFamilyIndices[0] != queueFamilyIndices[1])
		{
			// If the graphics and presenting queues are different, go down the simplest path to avoid explicit management headache
			swapchainCreateInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
			swapchainCreateInfo.queueFamilyIndexCount = 2;
			swapchainCreateInfo.pQueueFamilyIndices = queueFamilyIndices;
		}
		else
		{
			swapchainCreateInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
			// Exclusive mode doesn't require explicit queue family list
			swapchainCreateInfo.queueFamilyIndexCount = 0;
			swapchainCreateInfo.pQueueFamilyIndices = nullptr;
		}

		swapchainCreateInfo.preTransform = m_vkPhysicalDeviceData.surfaceInfo.capabilities.currentTransform;
		swapchainCreateInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
		swapchainCreateInfo.presentMode = presentMode;
		swapchainCreateInfo.clipped = VK_TRUE;
		swapchainCreateInfo.oldSwapchain = VK_NULL_HANDLE;

		if (vkCreateSwapchainKHR(m_vkLogicalDeviceData.vkHandle, &swapchainCreateInfo, getVkAllocator(), &m_vkSwapchainData.vkHandle) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create swap chain!\n");
		}

		m_vkSwapchainData.format = presentableSurfaceFormat.format;
		m_vkSwapchainData.extent = presentableSurfaceExtents;
		m_vkSwapchainData.colorSpace = presentableSurfaceFormat.colorSpace;

		uint32_t imageCount;
		vkGetSwapchainImagesKHR(m_vkLogicalDeviceData.vkHandle, m_vkSwapchainData.vkHandle, &imageCount, nullptr);
		m_vkSwapchainData.images.resize(imageCount);
		vkGetSwapchainImagesKHR(m_vkLogicalDeviceData.vkHandle, m_vkSwapchainData.vkHandle, &imageCount, m_vkSwapchainData.images.data());

		m_vkSwapchainData.imageViews.resize(imageCount);

		for (uint32_t imgIdx = 0; imgIdx < imageCount; ++imgIdx)
		{
			VkImageView imageView;
			imageView = createImageView2D(
							m_vkLogicalDeviceData.vkHandle, 
							m_vkSwapchainData.images[imgIdx],
							m_vkSwapchainData.format,
							VK_IMAGE_ASPECT_COLOR_BIT,
							getVkAllocator()
							);
			m_vkSwapchainData.imageViews[imgIdx] = imageView;
		}
	}
	void Wrapper::deinitSwapchain()
	{
		for (uint32_t imgIdx = 0, imgIdxEnd = (uint32_t)m_vkSwapchainData.imageViews.size(); imgIdx < imgIdxEnd; ++imgIdx)
		{
			vkDestroyImageView(m_vkLogicalDeviceData.vkHandle, m_vkSwapchainData.imageViews[imgIdx], getVkAllocator());
		}
		m_vkSwapchainData.imageViews.resize(0);

		vkDestroySwapchainKHR(m_vkLogicalDeviceData.vkHandle, m_vkSwapchainData.vkHandle, getVkAllocator());
		m_vkSwapchainData.vkHandle = VK_NULL_HANDLE;
	}

	void Wrapper::reinitSwapchain()
	{
		// At the moment, swapchain recreation requires full cease of rendering operations
		//	while it is possible to change swapchain mid-rendering, by keeping old swapchain for a while,
		//	and passing it to `VkSwapchainCreateInfoKHR` as well.
		vkDeviceWaitIdle(m_vkLogicalDeviceData.vkHandle);

		// Destroy everything swapchain-related (including buffers that depend on the swapchain extents)
		// Note on the swapchain command buffers: do not need swapchain command buffers recreation
		//	We allocate exactly one swapchain (primary) command buffer, and then update it with the swapchain image info

		//deinitShadowMapGenFramebuffers();
		deinitZPrePassFramebuffers();
		deinitSwapchainFramebuffers();
		deinitDebugPipelineState();
		deinitDepthOnlyPipelineState();

		// Theoretically we could avoid recreating the whole pipeline by using the pipeline dynamic states
		deinitPipelineState();
		deinitZPrePassRenderPass();	// Seems like these are not required here
		deinitRenderPass();			// Seems like these are not required here
		deinitDepthBuffer();
		deinitSwapchain();

		///////////////////////////////////////////////////////////////////////

		initSwapchain();
		initDepthBuffer();
		initRenderPass();			// Seems like these are not required here
		initZPrePassRenderPass();	// Seems like these are not required here
		updateDescriptorSetBindings();
		updateZPrePassDescriptorSetBindings();

		// Theoretically we could avoid recreating the whole pipeline by using the pipeline dynamic states
		initPipelineState();
		initDepthOnlyPipelineState();

		initDebugPipelineState();

		initSwapchainFramebuffers();
		initZPrePassFramebuffers();
	}

	void Wrapper::storeSwapchainImage(const wchar_t * filename)
	{
		if (!m_vkSwapchainData.supportsCapture)
			return;

		// Capture active swapchain image and store it into the TGA file
		// The function does the following:
		//	1. Creates proxy image
		//	2. Blits (preferrably) or copies contents of the swapchain image to the proxy image,
		//		taking into account layout transitions
		//	3. Reads back contents of the proxy image to the CPU memory
		//	4. Stores read back data into a file

		bool supportsBlit = true;
		if (!(m_vkSwapchainData.formatProps.optimalTilingFeatures & VK_FORMAT_FEATURE_BLIT_SRC_BIT))
		{
			supportsBlit = false;
		}

		if (!(m_vkSwapchainData.captureFormatProps.linearTilingFeatures & VK_FORMAT_FEATURE_BLIT_DST_BIT))
		{
			supportsBlit = false;
		}

		// Proxy image creation
		VkImageCreateInfo imageCreateInfo = {};
		imageCreateInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
		imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
		imageCreateInfo.format = m_vkSwapchainData.captureFormat;
		imageCreateInfo.extent.width = m_vkSwapchainData.extent.width;
		imageCreateInfo.extent.height = m_vkSwapchainData.extent.height;
		imageCreateInfo.extent.depth = 1;
		imageCreateInfo.arrayLayers = 1;
		imageCreateInfo.mipLevels = 1;
		imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
		imageCreateInfo.tiling = VK_IMAGE_TILING_LINEAR;
		imageCreateInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;

		VkResult result;

		VkImage capturedImage;
		result = vkCreateImage(m_vkLogicalDeviceData.vkHandle, &imageCreateInfo, getVkAllocator(), &capturedImage);
		if (result != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create image for capture!\n");
		}

		// Proxy image device memory allocation
		VkMemoryRequirements memoryRequirements;
		vkGetImageMemoryRequirements(m_vkLogicalDeviceData.vkHandle, capturedImage, &memoryRequirements);

		VkMemoryAllocateInfo memoryAllocateInfo = {};
		memoryAllocateInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		memoryAllocateInfo.allocationSize = memoryRequirements.size;
		memoryAllocateInfo.memoryTypeIndex = findMemoryType(m_vkPhysicalDeviceData.vkHandle, memoryRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

		VkDeviceMemory capturedImageDeviceMemory;
		result = vkAllocateMemory(m_vkLogicalDeviceData.vkHandle, &memoryAllocateInfo, getVkAllocator(), &capturedImageDeviceMemory);
		if (result != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to allocate memory for captured image!\n");
		}
		
		result = vkBindImageMemory(m_vkLogicalDeviceData.vkHandle, capturedImage, capturedImageDeviceMemory, 0);
		if (result != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to bind memory for captured image!\n");
		}

		VkImage presentedImage = m_vkSwapchainData.images[m_vkSwapchainData.imageIndexInSwapchain];

		VkCommandBuffer transientCommandBuffer;
		transientCommandBuffer = beginTransientCommandBuffer();
		{
			// Layout transitions prior to copying
			VkImageSubresourceRange imageSubresourceRange;
			imageSubresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
			imageSubresourceRange.baseMipLevel = 0;
			imageSubresourceRange.levelCount = 1;
			imageSubresourceRange.baseArrayLayer = 0;
			imageSubresourceRange.layerCount = 1;

			// Proxy image layout is optimial for transfer destination
			insertImageMemoryBarrierBuf(
				transientCommandBuffer,
				capturedImage,
				0,
				VK_ACCESS_TRANSFER_WRITE_BIT,
				VK_PIPELINE_STAGE_TRANSFER_BIT,
				VK_PIPELINE_STAGE_TRANSFER_BIT,
				0,
				VK_IMAGE_LAYOUT_UNDEFINED,
				VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
				imageSubresourceRange
				);

			// Transfer swapchain image layout to transfer source
			insertImageMemoryBarrierBuf(
				transientCommandBuffer,
				presentedImage,
				VK_ACCESS_MEMORY_READ_BIT,
				VK_ACCESS_TRANSFER_READ_BIT,
				VK_PIPELINE_STAGE_TRANSFER_BIT,
				VK_PIPELINE_STAGE_TRANSFER_BIT,
				0,
				VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
				VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
				imageSubresourceRange
				);

			if (supportsBlit)
			{
				// Blit does format conversion automatically

				VkOffset3D blitRegion3D[2];
				// Offset
				blitRegion3D[0].x = 0;
				blitRegion3D[0].y = 0;
				blitRegion3D[0].z = 0;
				// Size
				blitRegion3D[1].x = m_vkSwapchainData.extent.width;
				blitRegion3D[1].y = m_vkSwapchainData.extent.height;
				blitRegion3D[1].z = 1;

				VkImageBlit imageBlit = {};
				imageBlit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
				imageBlit.srcSubresource.layerCount = 1;
				imageBlit.srcOffsets[0] = blitRegion3D[0];
				imageBlit.srcOffsets[1] = blitRegion3D[1];
				imageBlit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
				imageBlit.dstSubresource.layerCount = 1;
				imageBlit.dstOffsets[0] = blitRegion3D[0];
				imageBlit.dstOffsets[1] = blitRegion3D[1];

				// Issue the blit command
				vkCmdBlitImage(
					transientCommandBuffer,
					presentedImage,
					VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
					capturedImage,
					VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
					1,
					&imageBlit,
					VK_FILTER_NEAREST
					);
			}
			else
			{
				// TODO: copying requires manual format conversion

				VkImageCopy imageCopy = {};
				imageCopy.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
				imageCopy.srcSubresource.layerCount = 1;
				imageCopy.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
				imageCopy.dstSubresource.layerCount = 1;
				imageCopy.extent.width = m_vkSwapchainData.extent.width;
				imageCopy.extent.height = m_vkSwapchainData.extent.height;
				imageCopy.extent.depth = 1;

				// Issue the copy command
				vkCmdCopyImage(
					transientCommandBuffer,
					presentedImage,
					VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
					capturedImage,
					VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
					1,
					&imageCopy
					);
			}

			// Transfer proxy image layout to GENERAL to be able to copy data onto CPU
			insertImageMemoryBarrierBuf(
				transientCommandBuffer,
				capturedImage,
				VK_ACCESS_TRANSFER_WRITE_BIT,
				VK_ACCESS_MEMORY_READ_BIT,
				VK_PIPELINE_STAGE_TRANSFER_BIT,
				VK_PIPELINE_STAGE_TRANSFER_BIT,
				0,
				VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
				VK_IMAGE_LAYOUT_GENERAL,
				imageSubresourceRange
				);

			// Transfer swapchain image layout back to the PRESENT_SRC
			insertImageMemoryBarrierBuf(
				transientCommandBuffer,
				presentedImage,
				VK_ACCESS_TRANSFER_READ_BIT,
				VK_ACCESS_MEMORY_READ_BIT,
				VK_PIPELINE_STAGE_TRANSFER_BIT,
				VK_PIPELINE_STAGE_TRANSFER_BIT,
				0,
				VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
				VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
				imageSubresourceRange
				);
		}
		endTransientCommandBuffer(transientCommandBuffer);

		// Retrieve subresource layout data - offset and row pitch,
		//	it is possible that pixel data is not contiguous
		VkImageSubresource imageSubresource = {};
		imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;

		VkSubresourceLayout subresourceLayout;
		vkGetImageSubresourceLayout(m_vkLogicalDeviceData.vkHandle, capturedImage, &imageSubresource, &subresourceLayout);

		const unsigned char * sparseCaptureData;
		vkMapMemory(m_vkLogicalDeviceData.vkHandle, capturedImageDeviceMemory, 0, VK_WHOLE_SIZE, 0, (void**)&sparseCaptureData);
		sparseCaptureData += subresourceLayout.offset;

		// Read the proxy image data into the CPU memory
		// Note: Vulkan NDC have Y pointing downwards, hence need to flip Y
		const uint32_t imgBPP = 4;
		const uint32_t imgWidth = m_vkSwapchainData.extent.width;
		const uint32_t imgHeight = m_vkSwapchainData.extent.height;
		const size_t imgTotalSize = imgWidth * imgHeight * imgBPP;
		const size_t imgRowSize = imgWidth * imgBPP;
		unsigned char * imgData = new unsigned char [imgTotalSize];
		unsigned char * imgDataRow = imgData + imgTotalSize - imgRowSize;
		const unsigned char * sparseCaptureDataRow = sparseCaptureData;
		for (uint32_t y = 0; y < imgHeight; y++) 
		{
			memcpy(imgDataRow, sparseCaptureDataRow, imgWidth * imgBPP * sizeof(unsigned char));
			sparseCaptureDataRow += subresourceLayout.rowPitch;
			imgDataRow -= imgRowSize;
		}

		vkUnmapMemory(m_vkLogicalDeviceData.vkHandle, capturedImageDeviceMemory);

		// Release proxy image resources
		vkFreeMemory(m_vkLogicalDeviceData.vkHandle, capturedImageDeviceMemory, getVkAllocator());
		vkDestroyImage(m_vkLogicalDeviceData.vkHandle, capturedImage, getVkAllocator());

		formats::ImagePixelFormat pixelFormat = formats::ImagePixelFormat::eRGBA;
		if (!supportsBlit && (m_vkSwapchainData.format == VK_FORMAT_B8G8R8A8_UNORM))
		{
			pixelFormat = formats::ImagePixelFormat::eBGRA;
		}

		// Save file
		const size_t filenameMBSByteSize = 128;
		char filenameMBS[filenameMBSByteSize];
		size_t numConverted = 0;
		wcstombs_s(&numConverted, filenameMBS, filename, filenameMBSByteSize);
		formats::writeTGA(filenameMBS, imgData, imgWidth, imgHeight, imgBPP * 8, pixelFormat);

		printf("Capture: %s\n", filenameMBS);

		delete [] imgData;
	}

	VkShaderModule Wrapper::initShaderModule(const std::vector<char> & shaderByteCode)
	{
		VkShaderModuleCreateInfo shaderModuleCreateInfo = {};
		shaderModuleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
		shaderModuleCreateInfo.codeSize = shaderByteCode.size();
		shaderModuleCreateInfo.pCode = reinterpret_cast<const uint32_t*>(shaderByteCode.data());

		VkShaderModule shaderModule;
		if (vkCreateShaderModule(m_vkLogicalDeviceData.vkHandle, &shaderModuleCreateInfo, getVkAllocator(), &shaderModule) != VK_SUCCESS)
		{
			printf("Failed to create shader module %d!\n", (int)shaderByteCode.size());
			return VK_NULL_HANDLE;
		}

		return shaderModule;
	}
	void Wrapper::deinitShaderModules()
	{
		for (const VkShaderModule & vkShaderModule : m_vkShaderModules)
		{
			vkDestroyShaderModule(m_vkLogicalDeviceData.vkHandle, vkShaderModule, getVkAllocator());
		}
		m_vkShaderModules.resize(0);
	}

	void Wrapper::initShadowMapGenRenderPass()
	{
		// Attachments (depth only)
		const uint32_t numAttachments = 1;
		VkAttachmentDescription attachmentDescriptions[numAttachments];

		VkAttachmentDescription & depthAttachmentDescription = attachmentDescriptions[numAttachments-1];
		depthAttachmentDescription = {};
		depthAttachmentDescription.format = m_vkShadowMapDepthFormat;
		depthAttachmentDescription.samples = VK_SAMPLE_COUNT_1_BIT;
		depthAttachmentDescription.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		depthAttachmentDescription.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		depthAttachmentDescription.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
		depthAttachmentDescription.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
		depthAttachmentDescription.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;						// We're going to clear the depth buffer, so data shouldn't be preserved
		depthAttachmentDescription.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

		VkAttachmentReference depthAttachmentRef = {};
		depthAttachmentRef.attachment = numAttachments-1;
		depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;


		// Subpass
		VkSubpassDescription subpassDescription = {};
		subpassDescription.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
		subpassDescription.colorAttachmentCount = 0;
		subpassDescription.pColorAttachments = nullptr;
		subpassDescription.pDepthStencilAttachment = &depthAttachmentRef;

		// We need dependencies to transform the SM depth buffer
		//	from the actual depth buffer to the shader read only resource
		const uint32_t numDependencies = 2;
		VkSubpassDependency subpassDependencies[numDependencies];

		VkSubpassDependency & depBegToDepthBuffer = subpassDependencies[0];
		depBegToDepthBuffer = {};
		depBegToDepthBuffer.srcSubpass = VK_SUBPASS_EXTERNAL;
		depBegToDepthBuffer.dstSubpass = 0;
		depBegToDepthBuffer.srcStageMask = VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT;
		depBegToDepthBuffer.dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
		depBegToDepthBuffer.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
		depBegToDepthBuffer.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

		VkSubpassDependency & depEndFromDepthBuffer = subpassDependencies[1];
		depEndFromDepthBuffer = {};
		depEndFromDepthBuffer.srcSubpass = 0;
		depEndFromDepthBuffer.dstSubpass = VK_SUBPASS_EXTERNAL;
		depEndFromDepthBuffer.srcStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
		depEndFromDepthBuffer.dstStageMask = VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT;
		depEndFromDepthBuffer.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		depEndFromDepthBuffer.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;


		// Render pass
		VkRenderPassCreateInfo renderPassCreateInfo = {};
		renderPassCreateInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
		renderPassCreateInfo.attachmentCount = numAttachments;
		renderPassCreateInfo.pAttachments = attachmentDescriptions;
		renderPassCreateInfo.subpassCount = 1;
		renderPassCreateInfo.pSubpasses = &subpassDescription;
		renderPassCreateInfo.dependencyCount = numDependencies;
		renderPassCreateInfo.pDependencies = subpassDependencies;

		if (vkCreateRenderPass(m_vkLogicalDeviceData.vkHandle, &renderPassCreateInfo, getVkAllocator(), &m_vkShadowMapGenRenderPass) != VK_SUCCESS)
		{
			printf("Failed to create render pass!\n");
		}
	}
	void Wrapper::deinitShadowMapGenRenderPass()
	{
		vkDestroyRenderPass(m_vkLogicalDeviceData.vkHandle, m_vkShadowMapGenRenderPass, getVkAllocator());
		m_vkShadowMapGenRenderPass = VK_NULL_HANDLE;
	}

	void Wrapper::initZPrePassRenderPass()
	{
		// Attachments (depth only)
		const uint32_t numAttachments = 1;
		VkAttachmentDescription attachmentDescriptions[numAttachments];

		VkAttachmentDescription & depthAttachmentDescription = attachmentDescriptions[numAttachments-1];
		depthAttachmentDescription = {};
		depthAttachmentDescription.format = m_vkDepthFormat;
		depthAttachmentDescription.samples = VK_SAMPLE_COUNT_1_BIT;
		depthAttachmentDescription.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		depthAttachmentDescription.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		depthAttachmentDescription.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
		depthAttachmentDescription.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
		depthAttachmentDescription.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;						// We're going to clear the depth buffer, so data shouldn't be preserved
		depthAttachmentDescription.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

		VkAttachmentReference depthAttachmentRef = {};
		depthAttachmentRef.attachment = numAttachments-1;
		depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;


		// Subpass
		VkSubpassDescription subpassDescription = {};
		subpassDescription.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
		subpassDescription.colorAttachmentCount = 0;
		subpassDescription.pColorAttachments = nullptr;
		subpassDescription.pDepthStencilAttachment = &depthAttachmentRef;


		// Here we don't exactly need the subpass dependency, as we have layout transition for the depth buffer
		//	which requires the Vulkan implementation (driver) to insert an implicit subpass dependency, if
		//	there is no explicit one.
		// But for the sake of clarity, the explicit dependency is created, which theoretically shouldn't hurt.
		// Note: here the framebuffer region dependency is used (VK_DEPENDENCY_BY_REGION_BIT)
		//	this means that we can start reading from the framebuffer region as soon as it's ready
		const uint32_t numDependencies = 2;
		VkSubpassDependency subpassDependencies[numDependencies];

		VkSubpassDependency & depBegToDepthBuffer = subpassDependencies[0];
		depBegToDepthBuffer = {};
		depBegToDepthBuffer.srcSubpass = VK_SUBPASS_EXTERNAL;
		depBegToDepthBuffer.dstSubpass = 0;
		depBegToDepthBuffer.srcStageMask = VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT;
		depBegToDepthBuffer.dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
		depBegToDepthBuffer.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		depBegToDepthBuffer.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		depBegToDepthBuffer.dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		VkSubpassDependency & depEndFromDepthBuffer = subpassDependencies[1];
		depEndFromDepthBuffer = {};
		depEndFromDepthBuffer.srcSubpass = 0;
		depEndFromDepthBuffer.dstSubpass = VK_SUBPASS_EXTERNAL;
		depEndFromDepthBuffer.srcStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
		depEndFromDepthBuffer.dstStageMask = VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT;
		depEndFromDepthBuffer.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		depEndFromDepthBuffer.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		depEndFromDepthBuffer.dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;


		// Render pass
		VkRenderPassCreateInfo renderPassCreateInfo = {};
		renderPassCreateInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
		renderPassCreateInfo.attachmentCount = numAttachments;
		renderPassCreateInfo.pAttachments = attachmentDescriptions;
		renderPassCreateInfo.subpassCount = 1;
		renderPassCreateInfo.pSubpasses = &subpassDescription;
		renderPassCreateInfo.dependencyCount = numDependencies;
		renderPassCreateInfo.pDependencies = subpassDependencies;

		if (vkCreateRenderPass(m_vkLogicalDeviceData.vkHandle, &renderPassCreateInfo, getVkAllocator(), &m_vkZPrePassRenderPass) != VK_SUCCESS)
		{
			printf("Failed to create render pass!\n");
		}
	}
	void Wrapper::deinitZPrePassRenderPass()
	{
		vkDestroyRenderPass(m_vkLogicalDeviceData.vkHandle, m_vkZPrePassRenderPass, getVkAllocator());
		m_vkZPrePassRenderPass = VK_NULL_HANDLE;
	}

	void Wrapper::initRenderPass()
	{
		// Attachments (color, depth)
		const uint32_t numAttachments = 2;
		VkAttachmentDescription attachmentDescriptions[numAttachments];

		VkAttachmentDescription & colorAttachmentDescription = attachmentDescriptions[0];
		colorAttachmentDescription = {};
		colorAttachmentDescription.format = m_vkSwapchainData.format;
		colorAttachmentDescription.samples = VK_SAMPLE_COUNT_1_BIT;
		colorAttachmentDescription.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		colorAttachmentDescription.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		colorAttachmentDescription.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
		colorAttachmentDescription.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
		colorAttachmentDescription.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		colorAttachmentDescription.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

		VkAttachmentReference colorAttachmentRef = {};
		colorAttachmentRef.attachment = 0;
		colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

		// Since the depth buffer was already written in the z-prepass stage, we cannot use LAYOUT_UNDFINED
		//	in the initialLayout, as this doesn't guarantee that data will be preserved. So instead,
		//	we're using the same layout that z-prepass render pass uses
		VkAttachmentDescription & depthAttachmentDescription = attachmentDescriptions[numAttachments-1];
		depthAttachmentDescription = {};
		depthAttachmentDescription.format = m_vkDepthFormat;
		depthAttachmentDescription.samples = VK_SAMPLE_COUNT_1_BIT;
		depthAttachmentDescription.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
		depthAttachmentDescription.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		depthAttachmentDescription.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
		depthAttachmentDescription.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
		depthAttachmentDescription.initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;	// We need to preserve contents of the depth buffer
		depthAttachmentDescription.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

		VkAttachmentReference depthAttachmentRef = {};
		depthAttachmentRef.attachment = numAttachments-1;
		depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;


		// Subpass
		VkSubpassDescription subpassDescription = {};
		subpassDescription.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
		subpassDescription.colorAttachmentCount = 1;
		subpassDescription.pColorAttachments = &colorAttachmentRef;
		subpassDescription.pDepthStencilAttachment = &depthAttachmentRef;

		// This set of dependencies shows that we're re-using the depth buffer that was
		//	modified by some subpass in other render pass (external to the current rennder pass)
		// Additional depepndency pair is for layout transition of the color attachment (presentable<->attachment)
		// Note: here the framebuffer region dependency is used (VK_DEPENDENCY_BY_REGION_BIT)
		//	this means that we can start reading from the framebuffer region as soon as it's ready

		const uint32_t numDependencies = 4;
		VkSubpassDependency subpassDependencies[numDependencies];

		VkSubpassDependency & depBegToDepthBuffer = subpassDependencies[0];
		depBegToDepthBuffer = {};
		depBegToDepthBuffer.srcSubpass = VK_SUBPASS_EXTERNAL;
		depBegToDepthBuffer.dstSubpass = 0;
		depBegToDepthBuffer.srcStageMask = VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT;
		depBegToDepthBuffer.dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
		depBegToDepthBuffer.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		depBegToDepthBuffer.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		depBegToDepthBuffer.dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		VkSubpassDependency & depEndFromDepthBuffer = subpassDependencies[1];
		depEndFromDepthBuffer = {};
		depEndFromDepthBuffer.srcSubpass = 0;
		depEndFromDepthBuffer.dstSubpass = VK_SUBPASS_EXTERNAL;
		depEndFromDepthBuffer.srcStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
		depEndFromDepthBuffer.dstStageMask = VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT;
		depEndFromDepthBuffer.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		depEndFromDepthBuffer.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		depEndFromDepthBuffer.dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		VkSubpassDependency & depBegToColorBuffer = subpassDependencies[2];
		depBegToColorBuffer = {};
		depBegToColorBuffer.srcSubpass = VK_SUBPASS_EXTERNAL;
		depBegToColorBuffer.dstSubpass = 0;
		depBegToColorBuffer.srcStageMask = VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT;
		depBegToColorBuffer.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		depBegToColorBuffer.srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		depBegToColorBuffer.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

		VkSubpassDependency & depEndFromColorBuffer = subpassDependencies[3];
		depEndFromColorBuffer = {};
		depEndFromColorBuffer.srcSubpass = 0;
		depEndFromColorBuffer.dstSubpass = VK_SUBPASS_EXTERNAL;
		depEndFromColorBuffer.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		depEndFromColorBuffer.dstStageMask = VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT;
		depEndFromColorBuffer.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		depEndFromColorBuffer.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;

		// Render pass
		VkRenderPassCreateInfo renderPassCreateInfo = {};
		renderPassCreateInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
		renderPassCreateInfo.attachmentCount = numAttachments;
		renderPassCreateInfo.pAttachments = attachmentDescriptions;
		renderPassCreateInfo.subpassCount = 1;
		renderPassCreateInfo.pSubpasses = &subpassDescription;
		renderPassCreateInfo.dependencyCount = numDependencies;
		renderPassCreateInfo.pDependencies = subpassDependencies;

		if (vkCreateRenderPass(m_vkLogicalDeviceData.vkHandle, &renderPassCreateInfo, getVkAllocator(), &m_vkRenderPass) != VK_SUCCESS)
		{
			printf("Failed to create render pass!\n");
		}
	}
	void Wrapper::deinitRenderPass()
	{
		vkDestroyRenderPass(m_vkLogicalDeviceData.vkHandle, m_vkRenderPass, getVkAllocator());
		m_vkRenderPass = VK_NULL_HANDLE;
	}

	void Wrapper::getVertexInputDescriptions(VkVertexInputBindingDescription * bindingDescr, VkVertexInputAttributeDescription attribsDescr[], int numAttribs)
	{
		if (bindingDescr == nullptr || attribsDescr == nullptr || numAttribs != 4)
		{
			// TODO: error
			printf("Wrong parameters supplied to the getVertexInputDescriptions!\n");
			assert(false && "Wrong parameters supplied to the getVertexInputDescriptions!\n");
		}

		*bindingDescr = {};
		bindingDescr->binding = 0;
		bindingDescr->stride = sizeof(Vertex);
		bindingDescr->inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

		// Position
		attribsDescr[0].binding = 0;
		attribsDescr[0].location = 0;
		attribsDescr[0].format = VK_FORMAT_R32G32B32_SFLOAT;
		attribsDescr[0].offset = offsetof(Vertex, pos);
		// Normal
		attribsDescr[1].binding = 0;
		attribsDescr[1].location = 1;
		attribsDescr[1].format = VK_FORMAT_R32G32B32_SFLOAT;
		attribsDescr[1].offset = offsetof(Vertex, nrm);
		// Color
		attribsDescr[2].binding = 0;
		attribsDescr[2].location = 2;
		attribsDescr[2].format = VK_FORMAT_R32G32B32A32_SFLOAT;
		attribsDescr[2].offset = offsetof(Vertex, col);
		// Texture coordinates
		attribsDescr[3].binding = 0;
		attribsDescr[3].location = 3;
		attribsDescr[3].format = VK_FORMAT_R32G32_SFLOAT;
		attribsDescr[3].offset = offsetof(Vertex, tc);
	}

	void Wrapper::getLinePointInputDescriptions(VkVertexInputBindingDescription * bindingDescr, VkVertexInputAttributeDescription attribsDescr[], int numAttribs)
	{
		if (bindingDescr == nullptr || attribsDescr == nullptr || numAttribs != 2)
		{
			// TODO: error
			printf("Wrong parameters supplied to the getVertexInputDescriptions!\n");
			assert(false && "Wrong parameters supplied to the getVertexInputDescriptions!\n");
		}

		*bindingDescr = {};
		bindingDescr->binding = 0;
		bindingDescr->stride = sizeof(LinePoint);
		bindingDescr->inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

		// Position
		attribsDescr[0].binding = 0;
		attribsDescr[0].location = 0;
		attribsDescr[0].format = VK_FORMAT_R32G32B32_SFLOAT;
		attribsDescr[0].offset = offsetof(LinePoint, pos);
		// Color
		attribsDescr[1].binding = 0;
		attribsDescr[1].location = 1;
		attribsDescr[1].format = VK_FORMAT_R32G32B32A32_SFLOAT;
		attribsDescr[1].offset = offsetof(LinePoint, col);
	}

	void Wrapper::initDepthOnlyPipelineState()
	{
		VkPipelineShaderStageCreateInfo vertShaderStageInfo = {};
		vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
		vertShaderStageInfo.module = m_vkShaderModules[4];
		vertShaderStageInfo.pName = "main";
		vertShaderStageInfo.pSpecializationInfo = nullptr;

		VkPipelineShaderStageCreateInfo fragShaderStageInfo = {};
		fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
		fragShaderStageInfo.module = m_vkShaderModules[5];
		fragShaderStageInfo.pName = "main";
		fragShaderStageInfo.pSpecializationInfo = nullptr;

		VkPipelineShaderStageCreateInfo shaderStages[] = { vertShaderStageInfo, fragShaderStageInfo };

		// Vertex Buffers
		VkVertexInputBindingDescription bindingDescription = {};
		const int numAttribs = 4;
		VkVertexInputAttributeDescription attributeDescriptions[numAttribs];
		getVertexInputDescriptions(&bindingDescription, attributeDescriptions, sizeof(attributeDescriptions)/sizeof(VkVertexInputAttributeDescription));

		VkPipelineVertexInputStateCreateInfo vertexInputStateCreateInfo = {};
		vertexInputStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
		vertexInputStateCreateInfo.vertexBindingDescriptionCount = 1;
		vertexInputStateCreateInfo.pVertexBindingDescriptions = &bindingDescription;
		vertexInputStateCreateInfo.vertexAttributeDescriptionCount = (uint32_t)numAttribs;
		vertexInputStateCreateInfo.pVertexAttributeDescriptions = attributeDescriptions;

		VkPipelineInputAssemblyStateCreateInfo inputAssemblyStateCreateInfo = {};
		inputAssemblyStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
		inputAssemblyStateCreateInfo.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
		inputAssemblyStateCreateInfo.primitiveRestartEnable = VK_FALSE;

		// Viewport and scissor: this will be dynamic state
		VkPipelineViewportStateCreateInfo viewportStateCreateInfo = {};
		viewportStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
		viewportStateCreateInfo.viewportCount = 1;
		viewportStateCreateInfo.pViewports = nullptr;
		viewportStateCreateInfo.scissorCount = 1;
		viewportStateCreateInfo.pScissors = nullptr;

		// Rasterizer
		VkPipelineRasterizationStateCreateInfo rasterizationStateCreateInfo = getDefaultRasterizationStateCreateInfo();
		rasterizationStateCreateInfo.depthBiasEnable = VK_TRUE;
		// MSAA
		VkPipelineMultisampleStateCreateInfo multisampleStateCreateInfo = getDefaultMultisampleStateCreateInfo();
		// Depth/Stencil
		VkPipelineDepthStencilStateCreateInfo depthStencilStateCreateInfo = getDefaultDepthStencilStateCreateInfo();
		// Blending
		VkPipelineColorBlendStateCreateInfo blendStateCreateInfo = getDefaultColorBlendStateCreateInfo();
		blendStateCreateInfo.attachmentCount = 0;
		blendStateCreateInfo.pAttachments = nullptr;

		VkPushConstantRange pushConstantRange = {};
		pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
		pushConstantRange.offset = 0;
		pushConstantRange.size = sizeof(ModelPushConstant);

		if (pushConstantRange.size > m_vkPhysicalDeviceData.deviceProperties.limits.maxPushConstantsSize)
		{
			// TODO: error
			printf("Physical device doesn't support push constants of this size: %d > %d\n",
				pushConstantRange.size,
				m_vkPhysicalDeviceData.deviceProperties.limits.maxPushConstantsSize
			);
		}

		VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = {};
		pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
		pipelineLayoutCreateInfo.setLayoutCount = 1;
		pipelineLayoutCreateInfo.pSetLayouts = &m_vkDepthOnlyShaderDescriptorSetLayout;
		pipelineLayoutCreateInfo.pushConstantRangeCount = 1;
		pipelineLayoutCreateInfo.pPushConstantRanges = &pushConstantRange;

		if (vkCreatePipelineLayout(m_vkLogicalDeviceData.vkHandle, &pipelineLayoutCreateInfo, getVkAllocator(), &m_vkDepthOnlyPipelineLayout) != VK_SUCCESS)
		{
			printf("Failed to create pipeline layout!\n");
		}

		// The shadow-only pass is used for 1) z-prepass and 2) shadow maps
		//	they are of different sizes, so we need to make viewport ans scissor dynamic
		const uint32_t numDynamicStates = 3;
		VkDynamicState dynamicStates[numDynamicStates];
		dynamicStates[0] = VK_DYNAMIC_STATE_VIEWPORT;
		dynamicStates[1] = VK_DYNAMIC_STATE_SCISSOR;
		dynamicStates[2] = VK_DYNAMIC_STATE_DEPTH_BIAS;

		VkPipelineDynamicStateCreateInfo dynamicState = {};
		dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
		dynamicState.pDynamicStates = dynamicStates;
		dynamicState.dynamicStateCount = numDynamicStates;


		VkGraphicsPipelineCreateInfo graphicsPipelineCreateInfo = {};
		graphicsPipelineCreateInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
		graphicsPipelineCreateInfo.stageCount = 2;
		graphicsPipelineCreateInfo.pStages = shaderStages;
		graphicsPipelineCreateInfo.pVertexInputState = &vertexInputStateCreateInfo;
		graphicsPipelineCreateInfo.pInputAssemblyState = &inputAssemblyStateCreateInfo;
		graphicsPipelineCreateInfo.pViewportState = &viewportStateCreateInfo;
		graphicsPipelineCreateInfo.pRasterizationState = &rasterizationStateCreateInfo;
		graphicsPipelineCreateInfo.pMultisampleState = &multisampleStateCreateInfo;
		graphicsPipelineCreateInfo.pDepthStencilState = &depthStencilStateCreateInfo;
		graphicsPipelineCreateInfo.pColorBlendState = &blendStateCreateInfo;
		graphicsPipelineCreateInfo.pDynamicState = &dynamicState;
		graphicsPipelineCreateInfo.layout = m_vkDepthOnlyPipelineLayout;
		// This pipeline is used in m_vkShadowMapGenRenderPass and m_vkZPrePassRenderPass
		//	but they are compatible:
		//	"their corresponding color, input, resolve, and depth/stencil attachment references are compatible"
		graphicsPipelineCreateInfo.renderPass = m_vkShadowMapGenRenderPass;
		graphicsPipelineCreateInfo.subpass = 0;
		graphicsPipelineCreateInfo.basePipelineHandle = VK_NULL_HANDLE;
		graphicsPipelineCreateInfo.basePipelineIndex = -1;

		if (vkCreateGraphicsPipelines(m_vkLogicalDeviceData.vkHandle, VK_NULL_HANDLE, 1, &graphicsPipelineCreateInfo, getVkAllocator(), &m_vkDepthOnlyGraphicsPipeline) != VK_SUCCESS)
		{
			printf("Failed to create graphics pipeline!\n");
		}
	}
	void Wrapper::deinitDepthOnlyPipelineState()
	{
		vkDestroyPipeline(m_vkLogicalDeviceData.vkHandle, m_vkDepthOnlyGraphicsPipeline, getVkAllocator());
		m_vkDepthOnlyGraphicsPipeline = VK_NULL_HANDLE;
		vkDestroyPipelineLayout(m_vkLogicalDeviceData.vkHandle, m_vkDepthOnlyPipelineLayout, getVkAllocator());
		m_vkDepthOnlyPipelineLayout = VK_NULL_HANDLE;
	}

	// Initializes graphics pipeline and render passes
	void Wrapper::initPipelineState()
	{
		VkPipelineShaderStageCreateInfo vertShaderStageInfo = {};
		vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
		vertShaderStageInfo.module = m_vkShaderModules[0];
		vertShaderStageInfo.pName = "main";
		vertShaderStageInfo.pSpecializationInfo = nullptr;

		VkPipelineShaderStageCreateInfo fragShaderStageInfo = {};
		fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
		fragShaderStageInfo.module = m_vkShaderModules[1];
		fragShaderStageInfo.pName = "main";
		fragShaderStageInfo.pSpecializationInfo = nullptr;

		VkPipelineShaderStageCreateInfo shaderStages[] = { vertShaderStageInfo, fragShaderStageInfo };

		// Vertex Buffers
		VkVertexInputBindingDescription bindingDescription = {};
		const int numAttribs = 4;
		VkVertexInputAttributeDescription attributeDescriptions[numAttribs];
		getVertexInputDescriptions(&bindingDescription, attributeDescriptions, sizeof(attributeDescriptions)/sizeof(VkVertexInputAttributeDescription));

		VkPipelineVertexInputStateCreateInfo vertexInputStateCreateInfo = {};
		vertexInputStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
		vertexInputStateCreateInfo.vertexBindingDescriptionCount = 1;
		vertexInputStateCreateInfo.pVertexBindingDescriptions = &bindingDescription;
		vertexInputStateCreateInfo.vertexAttributeDescriptionCount = (uint32_t)numAttribs;
		vertexInputStateCreateInfo.pVertexAttributeDescriptions = attributeDescriptions;

		VkPipelineInputAssemblyStateCreateInfo inputAssemblyStateCreateInfo = {};
		inputAssemblyStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
		inputAssemblyStateCreateInfo.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
		inputAssemblyStateCreateInfo.primitiveRestartEnable = VK_FALSE;

		// Viewport and scissor state
		VkPipelineViewportStateCreateInfo viewportStateCreateInfo = getDefaultViewportStateCreateInfo(m_vkSwapchainData.extent);

		// Rasterizer
		VkPipelineRasterizationStateCreateInfo rasterizationStateCreateInfo = getDefaultRasterizationStateCreateInfo();
		// MSAA
		VkPipelineMultisampleStateCreateInfo multisampleStateCreateInfo = getDefaultMultisampleStateCreateInfo();
		// Depth/Stencil
		VkPipelineDepthStencilStateCreateInfo depthStencilStateCreateInfo = getDefaultDepthStencilStateCreateInfo();
		depthStencilStateCreateInfo.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
		// Blending
		VkPipelineColorBlendAttachmentState blendAttachmentState = getDefaultColorBlendAttachmentState();
		VkPipelineColorBlendStateCreateInfo blendStateCreateInfo = getDefaultColorBlendStateCreateInfo();
		blendStateCreateInfo.attachmentCount = 1;
		blendStateCreateInfo.pAttachments = &blendAttachmentState;

		VkPushConstantRange pushConstantRange = {};
		pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
		pushConstantRange.offset = 0;
		pushConstantRange.size = sizeof(ModelPushConstant);

		if (pushConstantRange.size > m_vkPhysicalDeviceData.deviceProperties.limits.maxPushConstantsSize)
		{
			// TODO: error
			printf("Physical device doesn't support push constants of this size: %d > %d\n",
				pushConstantRange.size,
				m_vkPhysicalDeviceData.deviceProperties.limits.maxPushConstantsSize
				);
		}

		VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = {};
		pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
		pipelineLayoutCreateInfo.setLayoutCount = 1;
		pipelineLayoutCreateInfo.pSetLayouts = &m_vkFwdShadingDescriptorSetLayout;
		pipelineLayoutCreateInfo.pushConstantRangeCount = 1;
		pipelineLayoutCreateInfo.pPushConstantRanges = &pushConstantRange;

		if (vkCreatePipelineLayout(m_vkLogicalDeviceData.vkHandle, &pipelineLayoutCreateInfo, getVkAllocator(), &m_vkFwdShadingPipelineLayout) != VK_SUCCESS)
		{
			printf("Failed to create pipeline layout!\n");
		}

		VkGraphicsPipelineCreateInfo graphicsPipelineCreateInfo = {};
		graphicsPipelineCreateInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
		graphicsPipelineCreateInfo.stageCount = 2;
		graphicsPipelineCreateInfo.pStages = shaderStages;
		graphicsPipelineCreateInfo.pVertexInputState = &vertexInputStateCreateInfo;
		graphicsPipelineCreateInfo.pInputAssemblyState = &inputAssemblyStateCreateInfo;
		graphicsPipelineCreateInfo.pViewportState = &viewportStateCreateInfo;
		graphicsPipelineCreateInfo.pRasterizationState = &rasterizationStateCreateInfo;
		graphicsPipelineCreateInfo.pMultisampleState = &multisampleStateCreateInfo;
		graphicsPipelineCreateInfo.pDepthStencilState = &depthStencilStateCreateInfo;
		graphicsPipelineCreateInfo.pColorBlendState = &blendStateCreateInfo;
		graphicsPipelineCreateInfo.pDynamicState = nullptr;
		graphicsPipelineCreateInfo.layout = m_vkFwdShadingPipelineLayout;
		graphicsPipelineCreateInfo.renderPass = m_vkRenderPass;
		graphicsPipelineCreateInfo.subpass = 0;
		graphicsPipelineCreateInfo.basePipelineHandle = VK_NULL_HANDLE;
		graphicsPipelineCreateInfo.basePipelineIndex = -1;

		if (vkCreateGraphicsPipelines(m_vkLogicalDeviceData.vkHandle, VK_NULL_HANDLE, 1, &graphicsPipelineCreateInfo, getVkAllocator(), &m_vkFwdShadingGraphicsPipeline) != VK_SUCCESS)
		{
			printf("Failed to create graphics pipeline!\n");
		}
	}
	void Wrapper::deinitPipelineState()
	{
		vkDestroyPipeline(m_vkLogicalDeviceData.vkHandle, m_vkFwdShadingGraphicsPipeline, getVkAllocator());
		m_vkFwdShadingGraphicsPipeline = VK_NULL_HANDLE;
		vkDestroyPipelineLayout(m_vkLogicalDeviceData.vkHandle, m_vkFwdShadingPipelineLayout, getVkAllocator());
		m_vkFwdShadingPipelineLayout = VK_NULL_HANDLE;
	}

	void Wrapper::initDebugPipelineState()
	{
		VkPipelineShaderStageCreateInfo vertShaderStageInfo = {};
		vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
		vertShaderStageInfo.module = m_vkShaderModules[2];
		vertShaderStageInfo.pName = "main";
		vertShaderStageInfo.pSpecializationInfo = nullptr;

		VkPipelineShaderStageCreateInfo fragShaderStageInfo = {};
		fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
		fragShaderStageInfo.module = m_vkShaderModules[3];
		fragShaderStageInfo.pName = "main";
		fragShaderStageInfo.pSpecializationInfo = nullptr;

		VkPipelineShaderStageCreateInfo shaderStages[] = { vertShaderStageInfo, fragShaderStageInfo };

		// Vertex Buffers
		VkVertexInputBindingDescription bindingDescription = {};
		const int numAttribs = 2;
		VkVertexInputAttributeDescription attributeDescriptions[numAttribs];
		getLinePointInputDescriptions(&bindingDescription, attributeDescriptions, sizeof(attributeDescriptions)/sizeof(VkVertexInputAttributeDescription));

		VkPipelineVertexInputStateCreateInfo vertexInputStateCreateInfo = {};
		vertexInputStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
		vertexInputStateCreateInfo.vertexBindingDescriptionCount = 1;
		vertexInputStateCreateInfo.pVertexBindingDescriptions = &bindingDescription;
		vertexInputStateCreateInfo.vertexAttributeDescriptionCount = (uint32_t)numAttribs;
		vertexInputStateCreateInfo.pVertexAttributeDescriptions = attributeDescriptions;

		VkPipelineInputAssemblyStateCreateInfo inputAssemblyStateCreateInfo = {};
		inputAssemblyStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
		inputAssemblyStateCreateInfo.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
		inputAssemblyStateCreateInfo.primitiveRestartEnable = VK_FALSE;

		// Viewport and scissor state
		VkPipelineViewportStateCreateInfo viewportStateCreateInfo = getDefaultViewportStateCreateInfo(m_vkSwapchainData.extent);

		// Rasterizer
		VkPipelineRasterizationStateCreateInfo rasterizationStateCreateInfo = getDefaultRasterizationStateCreateInfo();
		rasterizationStateCreateInfo.cullMode = VK_CULL_MODE_NONE;
		// MSAA
		VkPipelineMultisampleStateCreateInfo multisampleStateCreateInfo = getDefaultMultisampleStateCreateInfo();
		// Depth/Stencil
		VkPipelineDepthStencilStateCreateInfo depthStencilStateCreateInfo = getDefaultDepthStencilStateCreateInfo();
		// Blending
		VkPipelineColorBlendAttachmentState blendAttachmentState = getDefaultColorBlendAttachmentState();
		VkPipelineColorBlendStateCreateInfo blendStateCreateInfo = getDefaultColorBlendStateCreateInfo();
		blendStateCreateInfo.attachmentCount = 1;
		blendStateCreateInfo.pAttachments = &blendAttachmentState;

		VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = {};
		pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
		pipelineLayoutCreateInfo.setLayoutCount = 1;
		pipelineLayoutCreateInfo.pSetLayouts = &m_vkFwdShadingDescriptorSetLayout;
		pipelineLayoutCreateInfo.pushConstantRangeCount = 0;
		pipelineLayoutCreateInfo.pPushConstantRanges = nullptr;

		if (vkCreatePipelineLayout(m_vkLogicalDeviceData.vkHandle, &pipelineLayoutCreateInfo, getVkAllocator(), &m_vkDebugVisPipelineLayout) != VK_SUCCESS)
		{
			printf("Failed to create pipeline layout!\n");
		}

		VkGraphicsPipelineCreateInfo graphicsPipelineCreateInfo = {};
		graphicsPipelineCreateInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
		graphicsPipelineCreateInfo.stageCount = 2;
		graphicsPipelineCreateInfo.pStages = shaderStages;
		graphicsPipelineCreateInfo.pVertexInputState = &vertexInputStateCreateInfo;
		graphicsPipelineCreateInfo.pInputAssemblyState = &inputAssemblyStateCreateInfo;
		graphicsPipelineCreateInfo.pViewportState = &viewportStateCreateInfo;
		graphicsPipelineCreateInfo.pRasterizationState = &rasterizationStateCreateInfo;
		graphicsPipelineCreateInfo.pMultisampleState = &multisampleStateCreateInfo;
		graphicsPipelineCreateInfo.pDepthStencilState = &depthStencilStateCreateInfo;
		graphicsPipelineCreateInfo.pColorBlendState = &blendStateCreateInfo;
		graphicsPipelineCreateInfo.pDynamicState = nullptr;
		graphicsPipelineCreateInfo.layout = m_vkDebugVisPipelineLayout;
		graphicsPipelineCreateInfo.renderPass = m_vkRenderPass;
		graphicsPipelineCreateInfo.subpass = 0;
		graphicsPipelineCreateInfo.basePipelineHandle = VK_NULL_HANDLE;
		graphicsPipelineCreateInfo.basePipelineIndex = -1;

		if (vkCreateGraphicsPipelines(m_vkLogicalDeviceData.vkHandle, VK_NULL_HANDLE, 1, &graphicsPipelineCreateInfo, getVkAllocator(), &m_vkDebugVisGraphicsPipeline) != VK_SUCCESS)
		{
			printf("Failed to create graphics pipeline!\n");
		}
	}
	void Wrapper::deinitDebugPipelineState()
	{
		vkDestroyPipeline(m_vkLogicalDeviceData.vkHandle, m_vkDebugVisGraphicsPipeline, getVkAllocator());
		m_vkDebugVisGraphicsPipeline = VK_NULL_HANDLE;
		vkDestroyPipelineLayout(m_vkLogicalDeviceData.vkHandle, m_vkDebugVisPipelineLayout, getVkAllocator());
		m_vkDebugVisPipelineLayout = VK_NULL_HANDLE;
	}

	void Wrapper::initShadowMapDepthBuffer()
	{
		m_vkShadowMapDepthFormat = m_vkDepthFormat;

		createImage(
			m_vkPhysicalDeviceData.vkHandle,
			m_vkLogicalDeviceData.vkHandle,
			m_smWidth,
			m_smHeight,
			m_vkShadowMapDepthFormat,
			VK_IMAGE_TILING_OPTIMAL,
			VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
			&m_vkShadowMapDepthImage,
			&m_vkShadowMapDepthDeviceMemory,
			getVkAllocator()
			);
		m_vkShadowMapDepthImageView = createImageView2D(
			m_vkLogicalDeviceData.vkHandle,
			m_vkShadowMapDepthImage,
			m_vkShadowMapDepthFormat,
			VK_IMAGE_ASPECT_DEPTH_BIT,
			getVkAllocator()
			);

		transitionImageLayout(
			m_vkShadowMapDepthImage,
			m_vkShadowMapDepthFormat,
			VK_IMAGE_LAYOUT_UNDEFINED,
			VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL
			);
	}
	void Wrapper::deinitShadowMapDepthBuffer()
	{
		vkDestroyImageView(m_vkLogicalDeviceData.vkHandle, m_vkShadowMapDepthImageView, getVkAllocator());
		m_vkShadowMapDepthImageView = VK_NULL_HANDLE;
		vkDestroyImage(m_vkLogicalDeviceData.vkHandle, m_vkShadowMapDepthImage, getVkAllocator());
		m_vkShadowMapDepthImage = VK_NULL_HANDLE;
		vkFreeMemory(m_vkLogicalDeviceData.vkHandle, m_vkShadowMapDepthDeviceMemory, getVkAllocator());
		m_vkShadowMapDepthDeviceMemory = VK_NULL_HANDLE;
	}

	void Wrapper::initShadowMapGenFramebuffers()
	{
		const uint32_t numAttachments = 1;
		VkImageView attachments[numAttachments] =
		{
			m_vkShadowMapDepthImageView
		};

		VkFramebufferCreateInfo framebufferCreateInfo = {};
		framebufferCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		framebufferCreateInfo.renderPass = m_vkShadowMapGenRenderPass;
		framebufferCreateInfo.attachmentCount = numAttachments;
		framebufferCreateInfo.pAttachments = attachments;
		framebufferCreateInfo.width = m_smWidth;
		framebufferCreateInfo.height = m_smHeight;
		framebufferCreateInfo.layers = 1;

		if (vkCreateFramebuffer(m_vkLogicalDeviceData.vkHandle, &framebufferCreateInfo, getVkAllocator(), &m_vkShadowMapGenFramebuffer) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create shadowmap framebuffer!\n");
		}
	}
	void Wrapper::deinitShadowMapGenFramebuffers()
	{
		vkDestroyFramebuffer(m_vkLogicalDeviceData.vkHandle, m_vkShadowMapGenFramebuffer, getVkAllocator());
		m_vkShadowMapGenFramebuffer = VK_NULL_HANDLE;
	}

	void Wrapper::initZPrePassFramebuffers()
	{
		const uint32_t numAttachments = 1;
		VkImageView attachments[numAttachments] =
		{
			m_vkDepthImageView
		};

		VkFramebufferCreateInfo framebufferCreateInfo = {};
		framebufferCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		framebufferCreateInfo.renderPass = m_vkZPrePassRenderPass;
		framebufferCreateInfo.attachmentCount = numAttachments;
		framebufferCreateInfo.pAttachments = attachments;
		framebufferCreateInfo.width = m_vkSwapchainData.extent.width;
		framebufferCreateInfo.height = m_vkSwapchainData.extent.height;
		framebufferCreateInfo.layers = 1;

		if (vkCreateFramebuffer(m_vkLogicalDeviceData.vkHandle, &framebufferCreateInfo, getVkAllocator(), &m_vkZPrePassFramebuffer) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create z-prepass framebuffer!\n");
		}
	}
	void Wrapper::deinitZPrePassFramebuffers()
	{
		vkDestroyFramebuffer(m_vkLogicalDeviceData.vkHandle, m_vkZPrePassFramebuffer, getVkAllocator());
		m_vkZPrePassFramebuffer = VK_NULL_HANDLE;
	}

	void Wrapper::initSwapchainFramebuffers()
	{
		m_vkSwapchainData.framebuffers.resize(m_vkSwapchainData.imageViews.size());

		for (size_t i = 0, iend = m_vkSwapchainData.framebuffers.size(); i < iend; i++)
		{
			const uint32_t numAttachments = 2;
			VkImageView attachments[numAttachments] =
			{
				m_vkSwapchainData.imageViews[i],
				m_vkDepthImageView
			};

			VkFramebufferCreateInfo framebufferCreateInfo = {};
			framebufferCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
			framebufferCreateInfo.renderPass = m_vkRenderPass;
			framebufferCreateInfo.attachmentCount = numAttachments;
			framebufferCreateInfo.pAttachments = attachments;
			framebufferCreateInfo.width = m_vkSwapchainData.extent.width;
			framebufferCreateInfo.height = m_vkSwapchainData.extent.height;
			framebufferCreateInfo.layers = 1;

			if (vkCreateFramebuffer(m_vkLogicalDeviceData.vkHandle, &framebufferCreateInfo, getVkAllocator(), &m_vkSwapchainData.framebuffers[i]) != VK_SUCCESS)
			{
				// TODO: error
				printf("Failed to create swapchain framebuffer for image view %lld!\n", (uint64_t)(attachments[0]));
			}
		}
	}
	void Wrapper::deinitSwapchainFramebuffers()
	{
		for (const VkFramebuffer & swapchainFramebuffer : m_vkSwapchainData.framebuffers)
		{
			vkDestroyFramebuffer(m_vkLogicalDeviceData.vkHandle, swapchainFramebuffer, getVkAllocator());
		}
		m_vkSwapchainData.framebuffers.resize(0);
	}

	void Wrapper::initCommandPool()
	{
		VkCommandPoolCreateInfo commandPoolCreateInfo = {};
		commandPoolCreateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
		commandPoolCreateInfo.queueFamilyIndex = m_vkLogicalDeviceData.graphicsQueueFamilyIndex;
		// Set the VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT so that the same command buffer could be 'updated',
		//	e.g. with secondary buffer inheritance info
		commandPoolCreateInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

		if (vkCreateCommandPool(m_vkLogicalDeviceData.vkHandle, &commandPoolCreateInfo, getVkAllocator(), &m_vkMainThreadCommandPool) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create graphics command pool!\n");
		}
	}
	void Wrapper::deinitCommandPool()
	{
		// Command buffers deallocation happens automatically on VkCommandPool destruction
		vkDestroyCommandPool(m_vkLogicalDeviceData.vkHandle, m_vkMainThreadCommandPool, getVkAllocator());
		m_vkMainThreadCommandPool = VK_NULL_HANDLE;
	}

	void Wrapper::initDepthBuffer()
	{
		m_vkDepthFormat =
			findSupportedFormat(
				m_vkPhysicalDeviceData.vkHandle,
				{VK_FORMAT_D24_UNORM_S8_UINT, VK_FORMAT_D32_SFLOAT_S8_UINT},
				VK_IMAGE_TILING_OPTIMAL,
				VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT
				);

		createImage(
			m_vkPhysicalDeviceData.vkHandle,
			m_vkLogicalDeviceData.vkHandle,
			m_vkSwapchainData.extent.width,
			m_vkSwapchainData.extent.height,
			m_vkDepthFormat,
			VK_IMAGE_TILING_OPTIMAL,
			VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
			&m_vkDepthImage,
			&m_vkDepthDeviceMemory,
			getVkAllocator()
			);
		m_vkDepthImageView = createImageView2D(
			m_vkLogicalDeviceData.vkHandle,
			m_vkDepthImage,
			m_vkDepthFormat,
			VK_IMAGE_ASPECT_DEPTH_BIT,
			getVkAllocator()
			);

		transitionImageLayout(
			m_vkDepthImage,
			m_vkDepthFormat,
			VK_IMAGE_LAYOUT_UNDEFINED,
			VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL
			);
	}
	void Wrapper::deinitDepthBuffer()
	{
		vkDestroyImageView(m_vkLogicalDeviceData.vkHandle, m_vkDepthImageView, getVkAllocator());
		m_vkDepthImageView = VK_NULL_HANDLE;
		vkDestroyImage(m_vkLogicalDeviceData.vkHandle, m_vkDepthImage, getVkAllocator());
		m_vkDepthImage = VK_NULL_HANDLE;
		vkFreeMemory(m_vkLogicalDeviceData.vkHandle, m_vkDepthDeviceMemory, getVkAllocator());
		m_vkDepthDeviceMemory = VK_NULL_HANDLE;
	}

	VkCommandPool Wrapper::getTransientCommandPool()
	{
		// TODO: here we use m_vkMainThreadCommandPool which is generalized command pool
		//	it should be better to sue command pool specialized for temporary operations like that,
		//	created with a `VK_COMMAND_POOL_CREATE_TRANSIENT_BIT` flag
		return m_vkMainThreadCommandPool;
	}

	VkCommandBuffer Wrapper::beginTransientCommandBuffer()
	{
		VkCommandPool transientCommandPool = getTransientCommandPool();

		VkCommandBufferAllocateInfo commandBufferAllocateInfo = {};
		commandBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		commandBufferAllocateInfo.commandPool = transientCommandPool;
		commandBufferAllocateInfo.commandBufferCount = 1;

		VkCommandBuffer commandBuffer;
		vkAllocateCommandBuffers(m_vkLogicalDeviceData.vkHandle, &commandBufferAllocateInfo, &commandBuffer);

		VkCommandBufferBeginInfo commandBufferBeginInfo = {};
		commandBufferBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		commandBufferBeginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

		vkBeginCommandBuffer(commandBuffer, &commandBufferBeginInfo);

		return commandBuffer;
	}

	void Wrapper::endTransientCommandBuffer(const VkCommandBuffer & transientCommandBuffer)
	{
		vkEndCommandBuffer(transientCommandBuffer);

		VkSubmitInfo submitInfo = {};
		submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
		submitInfo.commandBufferCount = 1;
		submitInfo.pCommandBuffers = &transientCommandBuffer;

		vkQueueSubmit(m_vkLogicalDeviceData.graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);

		// TODO: instead of waiting the queue to completely go idle,
		//	we can create specific fence and wait for it
		vkQueueWaitIdle(m_vkLogicalDeviceData.graphicsQueue);

		VkCommandPool transientCommandPool = getTransientCommandPool();
		vkFreeCommandBuffers(m_vkLogicalDeviceData.vkHandle, transientCommandPool, 1, &transientCommandBuffer);
	}

	void Wrapper::copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size)
	{
		VkCommandBuffer transientCommandBuffer;

		transientCommandBuffer = beginTransientCommandBuffer();
		{
			VkBufferCopy bufferCopyRegion = {};
			bufferCopyRegion.srcOffset = 0;
			bufferCopyRegion.dstOffset = 0;
			bufferCopyRegion.size = size;
			vkCmdCopyBuffer(transientCommandBuffer, srcBuffer, dstBuffer, 1, &bufferCopyRegion);
		}
		endTransientCommandBuffer(transientCommandBuffer);
	}

	void Wrapper::insertImageMemoryBarrierBuf(
		const VkCommandBuffer & commandBuffer,
		VkImage image,
		VkAccessFlags srcAccessMask,
		VkAccessFlags dstAccessMask,
		VkPipelineStageFlags srcSyncStageMask,
		VkPipelineStageFlags dstSyncStageMask,
		VkDependencyFlags dependencyFlags,
		VkImageLayout oldImageLayout,
		VkImageLayout newImageLayout,
		const VkImageSubresourceRange & imageSubresourceRange
		)
	{
		VkImageMemoryBarrier imageMemoryBarrier = {};
		imageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		imageMemoryBarrier.srcAccessMask = srcAccessMask;
		imageMemoryBarrier.dstAccessMask = dstAccessMask;
		imageMemoryBarrier.oldLayout = oldImageLayout;
		imageMemoryBarrier.newLayout = newImageLayout;
		// In case of transitioning queue families, these fields should be used
		//	In case no ownership transition is intended, they should be both VK_QUEUE_FAMILY_IGNORED
		imageMemoryBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		imageMemoryBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		imageMemoryBarrier.image = image;
		imageMemoryBarrier.subresourceRange = imageSubresourceRange;

		const uint32_t memoryBarrierCount = 0;
		const VkMemoryBarrier * pMemoryBarriers = nullptr;

		const uint32_t bufferMemoryBarrierCount = 0;
		const VkBufferMemoryBarrier * pBufferMemoryBarriers = nullptr;
			
		const uint32_t imageMemoryBarrierCount = 1;
		const VkImageMemoryBarrier pImageMemoryBarriers[] = { imageMemoryBarrier };

		vkCmdPipelineBarrier(
			commandBuffer,
			srcSyncStageMask,
			dstSyncStageMask,
			dependencyFlags,
			memoryBarrierCount,
			pMemoryBarriers,
			bufferMemoryBarrierCount,
			pBufferMemoryBarriers,
			imageMemoryBarrierCount,
			pImageMemoryBarriers
			);
	}

	void Wrapper::transitionImageLayout(
		VkImage image,
		VkFormat format,
		VkImageLayout oldImageLayout,
		VkImageLayout newImageLayout
		)
	{
		VkCommandBuffer transientCommandBuffer;

		VkAccessFlags srcAccessMask = 0;
		VkAccessFlags dstAccessMask = 0;
		VkPipelineStageFlags srcSyncStageMask = 0;
		VkPipelineStageFlags dstSyncStageMask = 0;

		VkImageSubresourceRange imageSubresourceRange = {};
		imageSubresourceRange.baseMipLevel = 0;
		imageSubresourceRange.levelCount = 1;
		imageSubresourceRange.baseArrayLayer = 0;
		imageSubresourceRange.layerCount = 1;

		if (newImageLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)
		{
			imageSubresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;

			if (hasStencilComponent(format))
			{
				imageSubresourceRange.aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
			}
		}
		else
		{
			imageSubresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		}

		// Try to infer synchronization logic from the layouts supplied
		if (oldImageLayout == VK_IMAGE_LAYOUT_UNDEFINED && newImageLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)
		{
			srcAccessMask = 0;
			dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

			srcSyncStageMask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
			dstSyncStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
		}
		else if (oldImageLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newImageLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
		{
			srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
			dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

			srcSyncStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
			dstSyncStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
		}
		else if (oldImageLayout == VK_IMAGE_LAYOUT_UNDEFINED && newImageLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)
		{
			srcAccessMask = 0;
			dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

			srcSyncStageMask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
			dstSyncStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
		}
		else
		{
			// TODO: error
			printf("Transition logic doesn't support this pair of layouts");
			assert(false && "Transition logic doesn't support this pair of layouts");
		}

		transientCommandBuffer = beginTransientCommandBuffer();
		{
			insertImageMemoryBarrierBuf(
				transientCommandBuffer,
				image,
				srcAccessMask,
				dstAccessMask,
				srcSyncStageMask,
				dstSyncStageMask,
				0,
				oldImageLayout,
				newImageLayout,
				imageSubresourceRange
				);
		}
		endTransientCommandBuffer(transientCommandBuffer);
	}

	void Wrapper::copyBufferToImage(uint32_t width, uint32_t height, VkBuffer buffer, VkImage image)
	{
		VkCommandBuffer transientCommandBuffer;

		transientCommandBuffer = beginTransientCommandBuffer();
		{
			VkBufferImageCopy bufferImageCopyRegion = {};
			bufferImageCopyRegion.bufferOffset = 0;
			bufferImageCopyRegion.bufferRowLength = 0;
			bufferImageCopyRegion.bufferImageHeight = 0;

			bufferImageCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
			bufferImageCopyRegion.imageSubresource.mipLevel = 0;
			bufferImageCopyRegion.imageSubresource.baseArrayLayer = 0;
			bufferImageCopyRegion.imageSubresource.layerCount = 1;

			bufferImageCopyRegion.imageOffset = {0, 0, 0};
			bufferImageCopyRegion.imageExtent = {width, height, 1};

			vkCmdCopyBufferToImage(
				transientCommandBuffer,
				buffer,
				image,
				VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
				1,
				&bufferImageCopyRegion
				);
		}
		endTransientCommandBuffer(transientCommandBuffer);
	}

	void Wrapper::initTextureResource(TextureResource * textureResource, uint32_t width, uint32_t height, void * data)
	{
		textureResource->width = width;
		textureResource->height = height;

		const int imgSize = width*height*4;
		const size_t imgBufferSize = width*height*4*sizeof(unsigned char);

		textureResource->imageFormat = VK_FORMAT_R8G8B8A8_UNORM;

		VkBuffer stagingBuffer;
		VkDeviceMemory stagingBufferDeviceMemory;

		createBuffer(
			m_vkPhysicalDeviceData.vkHandle,
			m_vkLogicalDeviceData.vkHandle,
			(VkDeviceSize)imgBufferSize,
			VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			&stagingBuffer,
			&stagingBufferDeviceMemory,
			getVkAllocator()
			);

		void * stagingData = nullptr;
		vkMapMemory(m_vkLogicalDeviceData.vkHandle, stagingBufferDeviceMemory, 0, (VkDeviceSize)imgBufferSize, 0, &stagingData);
		memcpy(stagingData, data, imgBufferSize);
		vkUnmapMemory(m_vkLogicalDeviceData.vkHandle, stagingBufferDeviceMemory);

		createImage(
			m_vkPhysicalDeviceData.vkHandle,
			m_vkLogicalDeviceData.vkHandle,
			(uint32_t)textureResource->width,
			(uint32_t)textureResource->height,
			textureResource->imageFormat,
			VK_IMAGE_TILING_OPTIMAL,
			VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
			&textureResource->image,
			&textureResource->imageDeviceMemory,
			getVkAllocator()
			);

		/*
		// TODO: move copyBuffer to staging here, and take subresourceLayout.rowPitch into account
		//	actually, seems like it is not needed since staging buffer is used which is linear,
		//	and further layout transition handles the rest
		VkSubresourceLayout subresourceLayout;
		VkImageSubresource imageSubresource;
		imageSubresource.arrayLayer = 0;
		imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		imageSubresource.mipLevel = 0;
		vkGetImageSubresourceLayout(m_vkLogicalDeviceData.vkHandle, m_vkTextureImage, *imageSubresource, &subresourceLayout);
		//*/

		// Transition image layout for transfer-friendly, not caring about the current image contents
		transitionImageLayout(
			textureResource->image,
			textureResource->imageFormat,
			VK_IMAGE_LAYOUT_UNDEFINED,
			VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL
			);
		copyBufferToImage(
			(uint32_t)textureResource->width,
			(uint32_t)textureResource->height,
			stagingBuffer,
			textureResource->image
			);
		// Transition image layout for optimal access from the shader
		transitionImageLayout(
			textureResource->image,
			textureResource->imageFormat,
			VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
			VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
			);

		vkDestroyBuffer(m_vkLogicalDeviceData.vkHandle, stagingBuffer, getVkAllocator());
		vkFreeMemory(m_vkLogicalDeviceData.vkHandle, stagingBufferDeviceMemory, getVkAllocator());

		textureResource->imageView = createImageView2D(
				m_vkLogicalDeviceData.vkHandle,
				textureResource->image,
				textureResource->imageFormat,
				VK_IMAGE_ASPECT_COLOR_BIT,
				getVkAllocator()
				);		
	}

	void Wrapper::deinitTextureResource(TextureResource * textureResource)
	{
		vkDestroyImageView(m_vkLogicalDeviceData.vkHandle, textureResource->imageView, getVkAllocator());
		textureResource->imageView = VK_NULL_HANDLE;

		vkDestroyImage(m_vkLogicalDeviceData.vkHandle, textureResource->image, getVkAllocator());
		textureResource->image = VK_NULL_HANDLE;
		vkFreeMemory(m_vkLogicalDeviceData.vkHandle, textureResource->imageDeviceMemory, getVkAllocator());
		textureResource->imageDeviceMemory = VK_NULL_HANDLE;
	}

	void Wrapper::initAlbedoTexResource()
	{
		const int imgSizeW = 256;
		const int imgSizeH = 256;
		const int imgSize = imgSizeW*imgSizeH*4;
		const size_t imgBufferSize = imgSizeW*imgSizeH*4*sizeof(unsigned char);
		unsigned char * imgData = new unsigned char[imgSize];

		if (imgData == nullptr)
		{
			// TODO: error
			printf("Failed to allocate memory for texture!\n");
			return;
		}

		int yAdd = 0;
		for (int y = 0; y < imgSizeH; ++y)
		{
			for (int x = 0; x < imgSizeW; ++x)
			{
				int pixelOffset = ((x + yAdd) << 2);
#if 0
				imgData[pixelOffset  ] = (rand() % 255);
				imgData[pixelOffset+1] = (rand() % 255);
				imgData[pixelOffset+2] = (rand() % 255);
				imgData[pixelOffset+3] = (rand() % 255);
#else
				unsigned char pixelVal = (x ^ y);
				imgData[pixelOffset  ] = pixelVal;
				imgData[pixelOffset+1] = pixelVal;
				imgData[pixelOffset+2] = pixelVal;
				imgData[pixelOffset+3] = pixelVal;
#endif
			}
			yAdd += imgSizeW;
		}

		initTextureResource(&m_albedoTex, imgSizeW, imgSizeH, imgData);

		delete [] imgData;
	}
	void Wrapper::deinitAlbedoTexResource()
	{
		deinitTextureResource(&m_albedoTex);
	}

	void Wrapper::initLightProjTexResource()
	{
		const int imgSizeW = 256;
		const int imgSizeH = 256;
		const int imgSize = imgSizeW*imgSizeH*4;
		const size_t imgBufferSize = imgSizeW*imgSizeH*4*sizeof(unsigned char);
		unsigned char * imgData = new unsigned char[imgSize];

		if (imgData == nullptr)
		{
			// TODO: error
			printf("Failed to allocate memory for texture!\n");
			return;
		}

		int yAdd = 0;
		const int imgCenterX = imgSizeW / 2;
		const int imgCenterY = imgSizeH / 2;
		const float invRadX = 1.0f / (float)(imgSizeW - imgCenterX);
		const float invRadY = 1.0f / (float)(imgSizeH - imgCenterY);
		const float mul = 10.0f;
		for (int y = 0; y < imgSizeH; ++y)
		{
			for (int x = 0; x < imgSizeW; ++x)
			{
				int pixelOffset = ((x + yAdd) << 2);
				float dist = sqrtf(sqr((x - imgCenterX)*invRadX) + sqr((y - imgCenterY)*invRadY));
				int intensity = (int)(255*mul*clamp((1.0f - dist), 0.0f, 1.0f));
				intensity = clamp(intensity, 0, 255);
				imgData[pixelOffset  ] = intensity;
				imgData[pixelOffset+1] = intensity;
				imgData[pixelOffset+2] = intensity;
				imgData[pixelOffset+3] = intensity;
			}
			yAdd += imgSizeW;
		}

		initTextureResource(&m_lightProjTex, imgSizeW, imgSizeH, imgData);

		delete [] imgData;
	}
	void Wrapper::deinitLightProjTexResource()
	{
		deinitTextureResource(&m_lightProjTex);
	}

	void Wrapper::initTextureSampler()
	{
		VkSamplerCreateInfo samplerCreateInfo = {};
		samplerCreateInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
		samplerCreateInfo.magFilter = VK_FILTER_LINEAR;
		samplerCreateInfo.minFilter = VK_FILTER_LINEAR;
		samplerCreateInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
		samplerCreateInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
		samplerCreateInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
		if (m_vkPhysicalDeviceData.deviceFeatures.samplerAnisotropy)
		{
			samplerCreateInfo.anisotropyEnable = VK_TRUE;
			samplerCreateInfo.maxAnisotropy = 16;
		}
		else
		{
			samplerCreateInfo.anisotropyEnable = VK_FALSE;
			samplerCreateInfo.maxAnisotropy = 1;
		}
		samplerCreateInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
		samplerCreateInfo.unnormalizedCoordinates = VK_FALSE;
		samplerCreateInfo.compareEnable = VK_FALSE;
		samplerCreateInfo.compareOp = VK_COMPARE_OP_ALWAYS;
		samplerCreateInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
		samplerCreateInfo.mipLodBias = 0.0f;
		samplerCreateInfo.minLod = 0.0f;
		samplerCreateInfo.maxLod = 0.0f;

		if (vkCreateSampler(m_vkLogicalDeviceData.vkHandle, &samplerCreateInfo, getVkAllocator(), &m_vkTextureSampler) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create texture sampler!\n");
		}
	}
	void Wrapper::deinitTextureSampler()
	{
		vkDestroySampler(m_vkLogicalDeviceData.vkHandle, m_vkTextureSampler, getVkAllocator());
		m_vkTextureSampler = VK_NULL_HANDLE;
	}

	void Wrapper::initDepthTextureSampler()
	{
		VkSamplerCreateInfo samplerCreateInfo = {};
		samplerCreateInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
		samplerCreateInfo.magFilter = VK_FILTER_LINEAR;
		samplerCreateInfo.minFilter = VK_FILTER_LINEAR;
		samplerCreateInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
		samplerCreateInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
		samplerCreateInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
		samplerCreateInfo.anisotropyEnable = VK_FALSE;
		samplerCreateInfo.maxAnisotropy = 1;
		samplerCreateInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
		samplerCreateInfo.unnormalizedCoordinates = VK_FALSE;
		samplerCreateInfo.compareEnable = VK_TRUE;
		samplerCreateInfo.compareOp = VK_COMPARE_OP_LESS;
		samplerCreateInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
		samplerCreateInfo.mipLodBias = 0.0f;
		samplerCreateInfo.minLod = 0.0f;
		samplerCreateInfo.maxLod = 0.0f;

		if (vkCreateSampler(m_vkLogicalDeviceData.vkHandle, &samplerCreateInfo, getVkAllocator(), &m_vkDepthTextureSampler) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create shadow texture sampler!\n");
		}
	}
	void Wrapper::deinitDepthTextureSampler()
	{
		vkDestroySampler(m_vkLogicalDeviceData.vkHandle, m_vkDepthTextureSampler, getVkAllocator());
		m_vkDepthTextureSampler = VK_NULL_HANDLE;
	}

	void Wrapper::initFSQuadBuffers()
	{
#define SINGLE_QUAD 1

		//
		// Vertex buffer
		// 
		{
#if (SINGLE_QUAD == 1)
			const size_t numVertices = 4;
			Vertex vertices[numVertices];
			vertices[0] = { Vec3C(-1.0f, -1.0f, 0.0f), Vec3C(0.0f, 0.0f, 1.0f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f), Vec2C(0.0f, 1.0f) };
			vertices[1] = { Vec3C( 1.0f, -1.0f, 0.0f), Vec3C(0.0f, 0.0f, 1.0f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f), Vec2C(1.0f, 1.0f) };
			vertices[2] = { Vec3C(-1.0f,  1.0f, 0.0f), Vec3C(0.0f, 0.0f, 1.0f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f), Vec2C(0.0f, 0.0f) };
			vertices[3] = { Vec3C( 1.0f,  1.0f, 0.0f), Vec3C(0.0f, 0.0f, 1.0f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f), Vec2C(1.0f, 0.0f) };
#else
			const size_t numVertices = 8;
			Vertex vertices[numVertices];

			Vec3 offset1 = Vec3C( 0.25f,  0.25f, 0.25f);
			Vec3 size1 = Vec3C(0.5f, 0.5f, 0.5f);
			vertices[0] = { Vec3C(-size1.x, -size1.y, 0.0f) + offset1, Vec3C(0.0f, 0.0f, 1.0f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f), Vec2C(0.0f, 1.0f) };
			vertices[1] = { Vec3C( size1.x, -size1.y, 0.0f) + offset1, Vec3C(0.0f, 0.0f, 1.0f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f), Vec2C(1.0f, 1.0f) };
			vertices[2] = { Vec3C(-size1.x,  size1.y, 0.0f) + offset1, Vec3C(0.0f, 0.0f, 1.0f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f), Vec2C(0.0f, 0.0f) };
			vertices[3] = { Vec3C( size1.x,  size1.y, 0.0f) + offset1, Vec3C(0.0f, 0.0f, 1.0f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f), Vec2C(1.0f, 0.0f) };

			Vec3 offset2 = Vec3C(-0.25f, -0.25f, 0.5f);
			Vec3 size2 = Vec3C(0.5f, 0.5f, 0.5f);
			vertices[4] = { Vec3C(-size2.x, -size2.y, 0.0f) + offset2, Vec3C(0.0f, 0.0f, 1.0f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f), Vec2C(0.0f, 1.0f) };
			vertices[5] = { Vec3C( size2.x, -size2.y, 0.0f) + offset2, Vec3C(0.0f, 0.0f, 1.0f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f), Vec2C(1.0f, 1.0f) };
			vertices[6] = { Vec3C(-size2.x,  size2.y, 0.0f) + offset2, Vec3C(0.0f, 0.0f, 1.0f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f), Vec2C(0.0f, 0.0f) };
			vertices[7] = { Vec3C( size2.x,  size2.y, 0.0f) + offset2, Vec3C(0.0f, 0.0f, 1.0f), Vec4C(1.0f, 1.0f, 1.0f, 1.0f), Vec2C(1.0f, 0.0f) };
#endif

			m_vkTriangleVerticesCount = numVertices;

			size_t triangleVertexBufferSize = sizeof(vertices[0]) * numVertices;

			VkBuffer stagingBuffer;
			VkDeviceMemory stagingBufferDeviceMemory;
			createBuffer(
				m_vkPhysicalDeviceData.vkHandle,
				m_vkLogicalDeviceData.vkHandle,
				(VkDeviceSize)triangleVertexBufferSize,
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				&stagingBuffer,
				&stagingBufferDeviceMemory,
				getVkAllocator()
				);

			// Fill in the Vertex Buffer
		
			/*
			(obsolete warning, we're using staging buffer now)
			// WARNING: to make sure the memory is seen by the device immediately, we're using VK_MEMORY_PROPERTY_HOST_COHERENT_BIT for the allocation
			//	but faster and probably more widespread way of doing so would be to utilize `vkFlushMappedMemoryRanges`/`vkInvalidateMappedMemoryRanges`
			*/

			void * data = nullptr;
			vkMapMemory(m_vkLogicalDeviceData.vkHandle, stagingBufferDeviceMemory, 0, (VkDeviceSize)triangleVertexBufferSize, 0, &data);
			memcpy(data, vertices, triangleVertexBufferSize);
			vkUnmapMemory(m_vkLogicalDeviceData.vkHandle, stagingBufferDeviceMemory);

			createBuffer(
				m_vkPhysicalDeviceData.vkHandle,
				m_vkLogicalDeviceData.vkHandle,
				(VkDeviceSize)triangleVertexBufferSize,
				VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				&m_vkTriangleVertexBuffer,
				&m_vkTriangleVertexBufferDeviceMemory,
				getVkAllocator()
				);

			copyBuffer(stagingBuffer, m_vkTriangleVertexBuffer, (VkDeviceSize)triangleVertexBufferSize);

			vkDestroyBuffer(m_vkLogicalDeviceData.vkHandle, stagingBuffer, getVkAllocator());
			vkFreeMemory(m_vkLogicalDeviceData.vkHandle, stagingBufferDeviceMemory, getVkAllocator());
		}

		//
		// Index buffer
		// 
		{
#if (SINGLE_QUAD == 1)
			const size_t numIndices = 6;
			uint16_t indices[numIndices] = 
			{
				0, 1, 2, 2, 1, 3
			};
#else
			const size_t numIndices = 12;
			uint16_t indices[numIndices] = 
			{
				0, 1, 2, 2, 1, 3,
				4, 5, 6, 6, 5, 7,
			};
#endif
			
			m_vkTriangleIndexBufferType = VK_INDEX_TYPE_UINT16;
			m_vkTriangleIndicesCount = numIndices;

			size_t triangleIndexBufferSize = sizeof(indices[0]) * numIndices;

			VkBuffer stagingBuffer;
			VkDeviceMemory stagingBufferDeviceMemory;
			createBuffer(
				m_vkPhysicalDeviceData.vkHandle,
				m_vkLogicalDeviceData.vkHandle,
				(VkDeviceSize)triangleIndexBufferSize,
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				&stagingBuffer,
				&stagingBufferDeviceMemory,
				getVkAllocator()
				);

			// Fill in the buffer

			void * data = nullptr;
			vkMapMemory(m_vkLogicalDeviceData.vkHandle, stagingBufferDeviceMemory, 0, (VkDeviceSize)triangleIndexBufferSize, 0, &data);
			memcpy(data, indices, triangleIndexBufferSize);
			vkUnmapMemory(m_vkLogicalDeviceData.vkHandle, stagingBufferDeviceMemory);

			createBuffer(
				m_vkPhysicalDeviceData.vkHandle,
				m_vkLogicalDeviceData.vkHandle,
				(VkDeviceSize)triangleIndexBufferSize,
				VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				&m_vkTriangleIndexBuffer,
				&m_vkTriangleIndexBufferDeviceMemory,
				getVkAllocator()
				);

			copyBuffer(stagingBuffer, m_vkTriangleIndexBuffer, (VkDeviceSize)triangleIndexBufferSize);

			vkDestroyBuffer(m_vkLogicalDeviceData.vkHandle, stagingBuffer, getVkAllocator());
			vkFreeMemory(m_vkLogicalDeviceData.vkHandle, stagingBufferDeviceMemory, getVkAllocator());
		}
	}
	void Wrapper::deinitFSQuadBuffers()
	{
		vkDestroyBuffer(m_vkLogicalDeviceData.vkHandle, m_vkTriangleIndexBuffer, getVkAllocator());
		m_vkTriangleIndexBuffer = VK_NULL_HANDLE;
		vkFreeMemory(m_vkLogicalDeviceData.vkHandle, m_vkTriangleIndexBufferDeviceMemory, getVkAllocator());
		m_vkTriangleIndexBufferDeviceMemory = VK_NULL_HANDLE;

		vkDestroyBuffer(m_vkLogicalDeviceData.vkHandle, m_vkTriangleVertexBuffer, getVkAllocator());
		m_vkTriangleVertexBuffer = VK_NULL_HANDLE;
		vkFreeMemory(m_vkLogicalDeviceData.vkHandle, m_vkTriangleVertexBufferDeviceMemory, getVkAllocator());
		m_vkTriangleVertexBufferDeviceMemory = VK_NULL_HANDLE;
	}

	void Wrapper::initDepthOnlyDescriptorSetLayout()
	{
		const uint32_t numBindings = 2;
		VkDescriptorSetLayoutBinding bindings[numBindings];

		VkDescriptorSetLayoutBinding & uboDescriptorSetLayoutBinding = bindings[0];
		uboDescriptorSetLayoutBinding = { };
		uboDescriptorSetLayoutBinding.binding = SH_BIND_GLOBAL_CONSTANTS;
		uboDescriptorSetLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		uboDescriptorSetLayoutBinding.descriptorCount = 1;
		uboDescriptorSetLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
		uboDescriptorSetLayoutBinding.pImmutableSamplers = nullptr;

		VkDescriptorSetLayoutBinding & transformUBODescriptorSetLayoutBinding = bindings[1];
		transformUBODescriptorSetLayoutBinding = { };
		transformUBODescriptorSetLayoutBinding.binding = SH_BIND_TRANSFORM;
		transformUBODescriptorSetLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		transformUBODescriptorSetLayoutBinding.descriptorCount = 1;
		transformUBODescriptorSetLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
		transformUBODescriptorSetLayoutBinding.pImmutableSamplers = nullptr;

		VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
		descriptorSetLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
		descriptorSetLayoutCreateInfo.bindingCount = numBindings;
		descriptorSetLayoutCreateInfo.pBindings = bindings;

		VkResult result = vkCreateDescriptorSetLayout(
								m_vkLogicalDeviceData.vkHandle,
								&descriptorSetLayoutCreateInfo,
								getVkAllocator(),
								&m_vkDepthOnlyShaderDescriptorSetLayout
								);
		if (result != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create depth-only UBO descriptor set layout!\n");
		}
	}
	void Wrapper::deinitDepthOnlyDescriptorSetLayout()
	{
		vkDestroyDescriptorSetLayout(m_vkLogicalDeviceData.vkHandle, m_vkDepthOnlyShaderDescriptorSetLayout, getVkAllocator());
		m_vkDepthOnlyShaderDescriptorSetLayout = VK_NULL_HANDLE;
	}

	void Wrapper::initDescriptorSetLayout()
	{
		const uint32_t numBindings = 6;
		VkDescriptorSetLayoutBinding bindings[numBindings];

		VkDescriptorSetLayoutBinding & globalConstUBODescriptorSetLayoutBinding = bindings[0];
		globalConstUBODescriptorSetLayoutBinding = { };
		globalConstUBODescriptorSetLayoutBinding.binding = SH_BIND_GLOBAL_CONSTANTS;
		globalConstUBODescriptorSetLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		globalConstUBODescriptorSetLayoutBinding.descriptorCount = 1;
		globalConstUBODescriptorSetLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
		globalConstUBODescriptorSetLayoutBinding.pImmutableSamplers = nullptr;

		VkDescriptorSetLayoutBinding & transformUBODescriptorSetLayoutBinding = bindings[1];
		transformUBODescriptorSetLayoutBinding = { };
		transformUBODescriptorSetLayoutBinding.binding = SH_BIND_TRANSFORM;
		transformUBODescriptorSetLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		transformUBODescriptorSetLayoutBinding.descriptorCount = 1;
		transformUBODescriptorSetLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
		transformUBODescriptorSetLayoutBinding.pImmutableSamplers = nullptr;

		// Material albedo texture
		VkDescriptorSetLayoutBinding & samplerDescriptorSetLayoutBinding = bindings[2];
		samplerDescriptorSetLayoutBinding = { };
		samplerDescriptorSetLayoutBinding.binding = SH_BIND_FWDSH_ALBEDO_TEX;
		samplerDescriptorSetLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		samplerDescriptorSetLayoutBinding.descriptorCount = 1;
		samplerDescriptorSetLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
		samplerDescriptorSetLayoutBinding.pImmutableSamplers = nullptr;

		// Shadowmap depth
		VkDescriptorSetLayoutBinding & depthSamplerDescriptorSetLayoutBinding = bindings[3];
		depthSamplerDescriptorSetLayoutBinding = { };
		depthSamplerDescriptorSetLayoutBinding.binding = SH_BIND_FWDSH_SHADOWMAP_TEX;
		depthSamplerDescriptorSetLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		depthSamplerDescriptorSetLayoutBinding.descriptorCount = 1;
		depthSamplerDescriptorSetLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
		depthSamplerDescriptorSetLayoutBinding.pImmutableSamplers = nullptr;

		// Light projection texture
		VkDescriptorSetLayoutBinding & samplerLightDescriptorSetLayoutBinding = bindings[4];
		samplerLightDescriptorSetLayoutBinding = { };
		samplerLightDescriptorSetLayoutBinding.binding = SH_BIND_FWDSH_LIGHTPROJ_TEX;
		samplerLightDescriptorSetLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		samplerLightDescriptorSetLayoutBinding.descriptorCount = 1;
		samplerLightDescriptorSetLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
		samplerLightDescriptorSetLayoutBinding.pImmutableSamplers = nullptr;

		VkDescriptorSetLayoutBinding & lightMatrixUBODescriptorSetLayoutBinding = bindings[5];
		lightMatrixUBODescriptorSetLayoutBinding = { };
		lightMatrixUBODescriptorSetLayoutBinding.binding = SH_BIND_FWDSH_LIGHTMATRIX;
		lightMatrixUBODescriptorSetLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		lightMatrixUBODescriptorSetLayoutBinding.descriptorCount = 1;
		lightMatrixUBODescriptorSetLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
		lightMatrixUBODescriptorSetLayoutBinding.pImmutableSamplers = nullptr;

		VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
		descriptorSetLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
		descriptorSetLayoutCreateInfo.bindingCount = numBindings;
		descriptorSetLayoutCreateInfo.pBindings = bindings;

		VkResult result = vkCreateDescriptorSetLayout(
								m_vkLogicalDeviceData.vkHandle,
								&descriptorSetLayoutCreateInfo,
								getVkAllocator(),
								&m_vkFwdShadingDescriptorSetLayout
								);
		if (result != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create UBO descriptor set layout!\n");
		}
	}
	void Wrapper::deinitDescriptorSetLayout()
	{
		vkDestroyDescriptorSetLayout(m_vkLogicalDeviceData.vkHandle, m_vkFwdShadingDescriptorSetLayout, getVkAllocator());
		m_vkFwdShadingDescriptorSetLayout = VK_NULL_HANDLE;
	}

	void Wrapper::initShadowMapGenTransformUBO()
	{
		size_t bufferSize = sizeof(TransformUBO);
		createBuffer(
			m_vkPhysicalDeviceData.vkHandle,
			m_vkLogicalDeviceData.vkHandle,
			(VkDeviceSize)bufferSize,
			VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			&m_vkShadowMapGenTransformUBOBuffer,
			&m_vkShadowMapGenTransformUBOBufferDeviceMemory,
			getVkAllocator()
			);
	}
	void Wrapper::deinitShadowMapGenTransformUBO()
	{
		vkDestroyBuffer(m_vkLogicalDeviceData.vkHandle, m_vkShadowMapGenTransformUBOBuffer, getVkAllocator());
		m_vkShadowMapGenTransformUBOBuffer = VK_NULL_HANDLE;
		vkFreeMemory(m_vkLogicalDeviceData.vkHandle, m_vkShadowMapGenTransformUBOBufferDeviceMemory, getVkAllocator());
		m_vkShadowMapGenTransformUBOBufferDeviceMemory = VK_NULL_HANDLE;
	}

	void Wrapper::initGlobalConstantsUBO()
	{
		size_t bufferSize = sizeof(GlobalConstantsUBO);
		createBuffer(
			m_vkPhysicalDeviceData.vkHandle,
			m_vkLogicalDeviceData.vkHandle,
			(VkDeviceSize)bufferSize,
			VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			&m_vkGlobalConstantsUBOBuffer,
			&m_vkGlobalConstantsUBOBufferDeviceMemory,
			getVkAllocator()
			);
	}
	void Wrapper::deinitGlobalConstantsUBO()
	{
		vkDestroyBuffer(m_vkLogicalDeviceData.vkHandle, m_vkGlobalConstantsUBOBuffer, getVkAllocator());
		m_vkGlobalConstantsUBOBuffer = VK_NULL_HANDLE;
		vkFreeMemory(m_vkLogicalDeviceData.vkHandle, m_vkGlobalConstantsUBOBufferDeviceMemory, getVkAllocator());
		m_vkGlobalConstantsUBOBufferDeviceMemory = VK_NULL_HANDLE;
	}

	void Wrapper::initTransformUBO()
	{
		size_t bufferSize = sizeof(TransformUBO);
		createBuffer(
			m_vkPhysicalDeviceData.vkHandle,
			m_vkLogicalDeviceData.vkHandle,
			(VkDeviceSize)bufferSize,
			VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			&m_vkTransformUBOBuffer,
			&m_vkTransformUBOBufferDeviceMemory,
			getVkAllocator()
			);
	}
	void Wrapper::deinitTransformUBO()
	{
		vkDestroyBuffer(m_vkLogicalDeviceData.vkHandle, m_vkTransformUBOBuffer, getVkAllocator());
		m_vkTransformUBOBuffer = VK_NULL_HANDLE;
		vkFreeMemory(m_vkLogicalDeviceData.vkHandle, m_vkTransformUBOBufferDeviceMemory, getVkAllocator());
		m_vkTransformUBOBufferDeviceMemory = VK_NULL_HANDLE;
	}

	void Wrapper::initDescriptorPool()
	{
		const uint32_t descriptorPoolSizesNum = 2;
		VkDescriptorPoolSize descriptorPoolSizes[descriptorPoolSizesNum];

		VkDescriptorPoolSize & uboDescriptorPoolSize = descriptorPoolSizes[0];
		uboDescriptorPoolSize = { };
		uboDescriptorPoolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		uboDescriptorPoolSize.descriptorCount = 7;

		VkDescriptorPoolSize & samplerDescriptorPoolSize = descriptorPoolSizes[1];
		samplerDescriptorPoolSize = { };
		samplerDescriptorPoolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		samplerDescriptorPoolSize.descriptorCount = 3;

		VkDescriptorPoolCreateInfo descriptorPoolCreateInfo = {};
		descriptorPoolCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
#if (EXPLICIT_DESCRIPTORS_FREE == 1)
		descriptorPoolCreateInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
#else
		// Curently we're not willing to remove any descriptor sets explicitly, hence for now the flag is 0
		descriptorPoolCreateInfo.flags = (VkDescriptorPoolCreateFlags)0;
#endif
		descriptorPoolCreateInfo.maxSets = 3;
		descriptorPoolCreateInfo.poolSizeCount = descriptorPoolSizesNum;
		descriptorPoolCreateInfo.pPoolSizes = descriptorPoolSizes;

		VkResult result = vkCreateDescriptorPool(
								m_vkLogicalDeviceData.vkHandle,
								&descriptorPoolCreateInfo,
								getVkAllocator(),
								&m_vkDescriptorPool
								);
		if (result != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create descriptor pool!\n");
		}
	}
	void Wrapper::deinitDescriptorPool()
	{
		vkDestroyDescriptorPool(m_vkLogicalDeviceData.vkHandle, m_vkDescriptorPool, getVkAllocator());
		m_vkDescriptorPool = VK_NULL_HANDLE;
	}

	void Wrapper::initShadowMapGenDescriptorSet()
	{
		VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
		descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
		descriptorSetAllocateInfo.descriptorPool = m_vkDescriptorPool;
		descriptorSetAllocateInfo.descriptorSetCount = 1;
		descriptorSetAllocateInfo.pSetLayouts = &m_vkDepthOnlyShaderDescriptorSetLayout;

		VkResult result = vkAllocateDescriptorSets(
								m_vkLogicalDeviceData.vkHandle,
								&descriptorSetAllocateInfo,
								&m_vkShadowMapGenDescriptorSet
								);
		if (result != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to allocate descriptor set!\n");
		}
	}
	void Wrapper::updateShadowMapGenDescriptorSetBindings()
	{
		VkDescriptorBufferInfo descriptorUBOBufferInfo = {};
		descriptorUBOBufferInfo.buffer = m_vkGlobalConstantsUBOBuffer;
		descriptorUBOBufferInfo.offset = 0;
		descriptorUBOBufferInfo.range = (VkDeviceSize)sizeof(GlobalConstantsUBO);

		VkDescriptorBufferInfo descriptorTransformUBOBufferInfo = {};
		descriptorTransformUBOBufferInfo.buffer = m_vkShadowMapGenTransformUBOBuffer;
		descriptorTransformUBOBufferInfo.offset = 0;
		descriptorTransformUBOBufferInfo.range = (VkDeviceSize)sizeof(TransformUBO);

		const uint32_t writeDescriptorSetsNum = 2;
		VkWriteDescriptorSet writeDescriptorSets[writeDescriptorSetsNum];

		VkWriteDescriptorSet & uboWriteDescriptorSet = writeDescriptorSets[0];
		uboWriteDescriptorSet = { };
		uboWriteDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		uboWriteDescriptorSet.dstSet = m_vkShadowMapGenDescriptorSet;
		uboWriteDescriptorSet.dstBinding = SH_BIND_GLOBAL_CONSTANTS;
		uboWriteDescriptorSet.dstArrayElement = 0;
		uboWriteDescriptorSet.descriptorCount = 1;
		uboWriteDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		uboWriteDescriptorSet.pImageInfo = nullptr;
		uboWriteDescriptorSet.pBufferInfo = &descriptorUBOBufferInfo;
		uboWriteDescriptorSet.pTexelBufferView = nullptr;

		VkWriteDescriptorSet & transformUBOWriteDescriptorSet = writeDescriptorSets[1];
		transformUBOWriteDescriptorSet = { };
		transformUBOWriteDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		transformUBOWriteDescriptorSet.dstSet = m_vkShadowMapGenDescriptorSet;
		transformUBOWriteDescriptorSet.dstBinding = SH_BIND_TRANSFORM;
		transformUBOWriteDescriptorSet.dstArrayElement = 0;
		transformUBOWriteDescriptorSet.descriptorCount = 1;
		transformUBOWriteDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		transformUBOWriteDescriptorSet.pImageInfo = nullptr;
		transformUBOWriteDescriptorSet.pBufferInfo = &descriptorTransformUBOBufferInfo;
		transformUBOWriteDescriptorSet.pTexelBufferView = nullptr;

		vkUpdateDescriptorSets(m_vkLogicalDeviceData.vkHandle, writeDescriptorSetsNum, writeDescriptorSets, 0, nullptr);
	}
	void Wrapper::deinitShadowMapGenDescriptorSet()
	{
#if (EXPLICIT_DESCRIPTORS_FREE == 1)
		vkFreeDescriptorSets(m_vkLogicalDeviceData.vkHandle, m_vkDescriptorPool, 1, &m_vkShadowMapGenDescriptorSet);
		m_vkShadowMapGenDescriptorSet = VK_NULL_HANDLE;
#else
		// No need to explicitly deallocate descriptor set since its lifetime
		//	is equal to lifetime of the descrptor set pool
#endif
	}

	void Wrapper::initZPrePassDescriptorSet()
	{
		VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
		descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
		descriptorSetAllocateInfo.descriptorPool = m_vkDescriptorPool;
		descriptorSetAllocateInfo.descriptorSetCount = 1;
		descriptorSetAllocateInfo.pSetLayouts = &m_vkDepthOnlyShaderDescriptorSetLayout;

		VkResult result = vkAllocateDescriptorSets(
			m_vkLogicalDeviceData.vkHandle,
			&descriptorSetAllocateInfo,
			&m_vkZPrePassDescriptorSet
			);
		if (result != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to allocate descriptor set!\n");
		}
	}
	void Wrapper::updateZPrePassDescriptorSetBindings()
	{
		VkDescriptorBufferInfo descriptorUBOBufferInfo = {};
		descriptorUBOBufferInfo.buffer = m_vkGlobalConstantsUBOBuffer;
		descriptorUBOBufferInfo.offset = 0;
		descriptorUBOBufferInfo.range = (VkDeviceSize)sizeof(GlobalConstantsUBO);

		VkDescriptorBufferInfo descriptorTransformUBOBufferInfo = {};
		descriptorTransformUBOBufferInfo.buffer = m_vkTransformUBOBuffer;
		descriptorTransformUBOBufferInfo.offset = 0;
		descriptorTransformUBOBufferInfo.range = (VkDeviceSize)sizeof(TransformUBO);

		const uint32_t writeDescriptorSetsNum = 2;
		VkWriteDescriptorSet writeDescriptorSets[writeDescriptorSetsNum];

		VkWriteDescriptorSet & uboWriteDescriptorSet = writeDescriptorSets[0];
		uboWriteDescriptorSet = { };
		uboWriteDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		uboWriteDescriptorSet.dstSet = m_vkZPrePassDescriptorSet;
		uboWriteDescriptorSet.dstBinding = SH_BIND_GLOBAL_CONSTANTS;
		uboWriteDescriptorSet.dstArrayElement = 0;
		uboWriteDescriptorSet.descriptorCount = 1;
		uboWriteDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		uboWriteDescriptorSet.pImageInfo = nullptr;
		uboWriteDescriptorSet.pBufferInfo = &descriptorUBOBufferInfo;
		uboWriteDescriptorSet.pTexelBufferView = nullptr;

		VkWriteDescriptorSet & transformUBOWriteDescriptorSet = writeDescriptorSets[1];
		transformUBOWriteDescriptorSet = { };
		transformUBOWriteDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		transformUBOWriteDescriptorSet.dstSet = m_vkZPrePassDescriptorSet;
		transformUBOWriteDescriptorSet.dstBinding = SH_BIND_TRANSFORM;
		transformUBOWriteDescriptorSet.dstArrayElement = 0;
		transformUBOWriteDescriptorSet.descriptorCount = 1;
		transformUBOWriteDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		transformUBOWriteDescriptorSet.pImageInfo = nullptr;
		transformUBOWriteDescriptorSet.pBufferInfo = &descriptorTransformUBOBufferInfo;
		transformUBOWriteDescriptorSet.pTexelBufferView = nullptr;

		vkUpdateDescriptorSets(m_vkLogicalDeviceData.vkHandle, writeDescriptorSetsNum, writeDescriptorSets, 0, nullptr);
	}
	void Wrapper::deinitZPrePassDescriptorSet()
	{
#if (EXPLICIT_DESCRIPTORS_FREE == 1)
		vkFreeDescriptorSets(m_vkLogicalDeviceData.vkHandle, m_vkDescriptorPool, 1, &m_vkZPrePassDescriptorSet);
		m_vkZPrePassDescriptorSet = VK_NULL_HANDLE;
#else
		// No need to explicitly deallocate descriptor set since its lifetime
		//	is equal to lifetime of the descrptor set pool
#endif
	}

	void Wrapper::initDescriptorSet()
	{
		VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
		descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
		descriptorSetAllocateInfo.descriptorPool = m_vkDescriptorPool;
		descriptorSetAllocateInfo.descriptorSetCount = 1;
		descriptorSetAllocateInfo.pSetLayouts = &m_vkFwdShadingDescriptorSetLayout;

		VkResult result = vkAllocateDescriptorSets(
								m_vkLogicalDeviceData.vkHandle,
								&descriptorSetAllocateInfo,
								&m_vkDescriptorSet
								);
		if (result != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to allocate descriptor set!\n");
		}
	}
	void Wrapper::updateDescriptorSetBindings()
	{
		VkDescriptorBufferInfo descriptorUBOBufferInfo = {};
		descriptorUBOBufferInfo.buffer = m_vkGlobalConstantsUBOBuffer;
		descriptorUBOBufferInfo.offset = 0;
		descriptorUBOBufferInfo.range = (VkDeviceSize)sizeof(GlobalConstantsUBO);

		VkDescriptorBufferInfo descriptorTransformUBOBufferInfo = {};
		descriptorTransformUBOBufferInfo.buffer = m_vkTransformUBOBuffer;
		descriptorTransformUBOBufferInfo.offset = 0;
		descriptorTransformUBOBufferInfo.range = (VkDeviceSize)sizeof(TransformUBO);

		VkDescriptorImageInfo descriptorImageInfo = {};
		descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		descriptorImageInfo.imageView = m_albedoTex.imageView;
		descriptorImageInfo.sampler = m_vkTextureSampler;

		VkDescriptorImageInfo descriptorDepthImageInfo = {};
		descriptorDepthImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		descriptorDepthImageInfo.imageView = m_vkShadowMapDepthImageView;
		descriptorDepthImageInfo.sampler = m_vkDepthTextureSampler;

		VkDescriptorImageInfo descriptorLightImageInfo = {};
		descriptorLightImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		descriptorLightImageInfo.imageView = m_lightProjTex.imageView;
		descriptorLightImageInfo.sampler = m_vkTextureSampler;

		VkDescriptorBufferInfo descriptorLightMatrixUBOBufferInfo = {};
		descriptorLightMatrixUBOBufferInfo.buffer = m_vkShadowMapGenTransformUBOBuffer;
		descriptorLightMatrixUBOBufferInfo.offset = 0;
		descriptorLightMatrixUBOBufferInfo.range = (VkDeviceSize)sizeof(TransformUBO);

		const uint32_t writeDescriptorSetsNum = 6;
		VkWriteDescriptorSet writeDescriptorSets[writeDescriptorSetsNum];

		VkWriteDescriptorSet & uboWriteDescriptorSet = writeDescriptorSets[0];
		uboWriteDescriptorSet = { };
		uboWriteDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		uboWriteDescriptorSet.dstSet = m_vkDescriptorSet;
		uboWriteDescriptorSet.dstBinding = SH_BIND_GLOBAL_CONSTANTS;
		uboWriteDescriptorSet.dstArrayElement = 0;
		uboWriteDescriptorSet.descriptorCount = 1;
		uboWriteDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		uboWriteDescriptorSet.pImageInfo = nullptr;
		uboWriteDescriptorSet.pBufferInfo = &descriptorUBOBufferInfo;
		uboWriteDescriptorSet.pTexelBufferView = nullptr;

		VkWriteDescriptorSet & transformUBOWriteDescriptorSet = writeDescriptorSets[1];
		transformUBOWriteDescriptorSet = { };
		transformUBOWriteDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		transformUBOWriteDescriptorSet.dstSet = m_vkDescriptorSet;
		transformUBOWriteDescriptorSet.dstBinding = SH_BIND_TRANSFORM;
		transformUBOWriteDescriptorSet.dstArrayElement = 0;
		transformUBOWriteDescriptorSet.descriptorCount = 1;
		transformUBOWriteDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		transformUBOWriteDescriptorSet.pImageInfo = nullptr;
		transformUBOWriteDescriptorSet.pBufferInfo = &descriptorTransformUBOBufferInfo;
		transformUBOWriteDescriptorSet.pTexelBufferView = nullptr;

		VkWriteDescriptorSet & samplerWriteDescriptorSet = writeDescriptorSets[2];
		samplerWriteDescriptorSet = { };
		samplerWriteDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		samplerWriteDescriptorSet.dstSet = m_vkDescriptorSet;
		samplerWriteDescriptorSet.dstBinding = SH_BIND_FWDSH_ALBEDO_TEX;
		samplerWriteDescriptorSet.dstArrayElement = 0;
		samplerWriteDescriptorSet.descriptorCount = 1;
		samplerWriteDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		samplerWriteDescriptorSet.pImageInfo = &descriptorImageInfo;
		samplerWriteDescriptorSet.pBufferInfo = nullptr;
		samplerWriteDescriptorSet.pTexelBufferView = nullptr;

		VkWriteDescriptorSet & samplerDepthImageDescriptor = writeDescriptorSets[3];
		samplerDepthImageDescriptor = { };
		samplerDepthImageDescriptor.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		samplerDepthImageDescriptor.dstSet = m_vkDescriptorSet;
		samplerDepthImageDescriptor.dstBinding = SH_BIND_FWDSH_SHADOWMAP_TEX;
		samplerDepthImageDescriptor.dstArrayElement = 0;
		samplerDepthImageDescriptor.descriptorCount = 1;
		samplerDepthImageDescriptor.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		samplerDepthImageDescriptor.pImageInfo = &descriptorDepthImageInfo;
		samplerDepthImageDescriptor.pBufferInfo = nullptr;
		samplerDepthImageDescriptor.pTexelBufferView = nullptr;

		VkWriteDescriptorSet & samplerLightWriteDescriptorSet = writeDescriptorSets[4];
		samplerLightWriteDescriptorSet = { };
		samplerLightWriteDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		samplerLightWriteDescriptorSet.dstSet = m_vkDescriptorSet;
		samplerLightWriteDescriptorSet.dstBinding = SH_BIND_FWDSH_LIGHTPROJ_TEX;
		samplerLightWriteDescriptorSet.dstArrayElement = 0;
		samplerLightWriteDescriptorSet.descriptorCount = 1;
		samplerLightWriteDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		samplerLightWriteDescriptorSet.pImageInfo = &descriptorLightImageInfo;
		samplerLightWriteDescriptorSet.pBufferInfo = nullptr;
		samplerLightWriteDescriptorSet.pTexelBufferView = nullptr;

		VkWriteDescriptorSet & lightMatrixUBOWriteDescriptorSet = writeDescriptorSets[5];
		lightMatrixUBOWriteDescriptorSet = { };
		lightMatrixUBOWriteDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		lightMatrixUBOWriteDescriptorSet.dstSet = m_vkDescriptorSet;
		lightMatrixUBOWriteDescriptorSet.dstBinding = SH_BIND_FWDSH_LIGHTMATRIX;
		lightMatrixUBOWriteDescriptorSet.dstArrayElement = 0;
		lightMatrixUBOWriteDescriptorSet.descriptorCount = 1;
		lightMatrixUBOWriteDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		lightMatrixUBOWriteDescriptorSet.pImageInfo = nullptr;
		lightMatrixUBOWriteDescriptorSet.pBufferInfo = &descriptorLightMatrixUBOBufferInfo;
		lightMatrixUBOWriteDescriptorSet.pTexelBufferView = nullptr;

		vkUpdateDescriptorSets(m_vkLogicalDeviceData.vkHandle, writeDescriptorSetsNum, writeDescriptorSets, 0, nullptr);
	}
	void Wrapper::deinitDescriptorSet()
	{
#if (EXPLICIT_DESCRIPTORS_FREE == 1)
		vkFreeDescriptorSets(m_vkLogicalDeviceData.vkHandle, m_vkDescriptorPool, 1, &m_vkDescriptorSet);
		m_vkDescriptorSet = VK_NULL_HANDLE;
#else
		// No need to explicitly deallocate descriptor set since its lifetime
		//	is equal to lifetime of the descrptor set pool
#endif
	}

	void Wrapper::buildShadowMapGenCommandBuffer()
	{
		VkCommandBufferAllocateInfo commandBufferAllocateInfo = {};
		commandBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		commandBufferAllocateInfo.commandPool = m_vkMainThreadCommandPool;
		commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		commandBufferAllocateInfo.commandBufferCount = 1;

		if (vkAllocateCommandBuffers(m_vkLogicalDeviceData.vkHandle, &commandBufferAllocateInfo, &m_vkShadowMapGenCommandBuffer) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to allocate shadow map command buffer!\n");
		}
	}
	void Wrapper::updateShadowMapGenCommandBuffers()
	{
		VkCommandBuffer curCmdBuf = m_vkShadowMapGenCommandBuffer;

		{
			VkCommandBufferBeginInfo commandBufferBeginInfo = {};
			commandBufferBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
			// We're using `VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT` here to show that this
			//	command buffer will only be submitted once, to allow driver optimizxations;
			//	this is indeed the case, since we're re-generating the command buffer each frame, and only call it once per frame
			commandBufferBeginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
			commandBufferBeginInfo.pInheritanceInfo = nullptr;

			vkBeginCommandBuffer(curCmdBuf, &commandBufferBeginInfo);

			const uint32_t numAttachments = 1;
			VkClearValue clearValues[numAttachments];
			clearValues[numAttachments-1].depthStencil = { 1.0f, 0 };

			VkRenderPassBeginInfo renderPassBeginInfo = {};
			renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
			renderPassBeginInfo.renderPass = m_vkShadowMapGenRenderPass;
			renderPassBeginInfo.framebuffer = m_vkShadowMapGenFramebuffer;
			renderPassBeginInfo.renderArea.offset = { 0, 0 };
			renderPassBeginInfo.renderArea.extent = { m_smWidth, m_smHeight };
			renderPassBeginInfo.clearValueCount = numAttachments;
			renderPassBeginInfo.pClearValues = clearValues;

			// The render pass begin will automatically translate layout of the shadowmap depth buffer,
			//	(UNDEFINED -> DEPTH_STENCIL_ATTACHMENT_OPTIMAL) and clear its values
			// But to read this buffer from shader, we need to change the layout via the barrier
			vkCmdBeginRenderPass(curCmdBuf, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

			// Update dynamic viewport state
			VkViewport viewport = {};
			viewport.x = 0.0f;
			viewport.y = 0.0f;
			viewport.width = (float)m_smWidth;
			viewport.height = (float)m_smHeight;
			viewport.minDepth = (float) 0.0f;
			viewport.maxDepth = (float) 1.0f;
			vkCmdSetViewport(curCmdBuf, 0, 1, &viewport);

			// Update dynamic scissor state
			VkRect2D scissor = {};
			scissor.offset = { 0, 0 };
			scissor.extent = { m_smWidth, m_smHeight };
			vkCmdSetScissor(curCmdBuf, 0, 1, &scissor);

			// We need to set depth bias to avoid self-shadowing artifacts
			vkCmdSetDepthBias(curCmdBuf, 1.5f, 0.0f, 2.5f);

			vkCmdBindPipeline(curCmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, m_vkDepthOnlyGraphicsPipeline);

			vkCmdBindDescriptorSets(
				curCmdBuf,
				VK_PIPELINE_BIND_POINT_GRAPHICS,
				m_vkDepthOnlyPipelineLayout,
				0,
				1,
				&m_vkShadowMapGenDescriptorSet,
				0,
				nullptr
				);

#if (SCENE == SCENE_QUAD)
#elif (SCENE == SCENE_MESH)
			for (int mi = 0, miEnd = (int)m_meshes.size(); mi < miEnd; ++mi)
			{
				ModelPushConstant pushConstant;

				const Mat44 & modelMat = m_meshes[mi]->getModelMatrix();
				pushConstant.model = modelMat.getTransposed();

				vkCmdPushConstants(
					curCmdBuf,
					m_vkDepthOnlyPipelineLayout,
					VK_SHADER_STAGE_VERTEX_BIT,
					0,
					sizeof(ModelPushConstant),
					&pushConstant
					);

				VkBuffer vertexBuffers[] = { m_meshes[mi]->getVertexBuffer() };
				VkDeviceSize offsets[] = { 0 };
				vkCmdBindVertexBuffers(curCmdBuf, 0, 1, vertexBuffers, offsets);
				vkCmdBindIndexBuffer(curCmdBuf, m_meshes[mi]->getIndexBuffer(), 0, m_meshes[mi]->getIndexBufferType());

				vkCmdDrawIndexed(curCmdBuf, (uint32_t)m_meshes[mi]->getIndicesCount(), 1, 0, 0, 0);
			}
#endif

			if (m_debugTrisMesh.getVerticesCount() > 0)
			{
				ModelPushConstant pushConstant;

				const Mat44 modelMat = m_debugTrisMesh.getModelMatrix();
				pushConstant.model = modelMat.getTransposed();

				vkCmdPushConstants(
					curCmdBuf,
					m_vkFwdShadingPipelineLayout,
					VK_SHADER_STAGE_VERTEX_BIT,
					0,
					sizeof(ModelPushConstant),
					&pushConstant
				);

				VkBuffer vertexBuffers[] = { m_debugTrisMesh.getVertexBuffer() };
				VkDeviceSize offsets[] = { 0 };
				vkCmdBindVertexBuffers(curCmdBuf, 0, 1, vertexBuffers, offsets);

				vkCmdDraw(curCmdBuf, (uint32_t)m_debugTrisMesh.getVerticesCount(), 1, 0, 0);
			}

			vkCmdEndRenderPass(curCmdBuf);

			VkImageSubresourceRange imageSubresourceRange;
			imageSubresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
			imageSubresourceRange.baseMipLevel = 0;
			imageSubresourceRange.levelCount = 1;
			imageSubresourceRange.baseArrayLayer = 0;
			imageSubresourceRange.layerCount = 1;

			if (hasStencilComponent(m_vkShadowMapDepthFormat))
			{
				imageSubresourceRange.aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
			}

			// Barrier that transfers layout of the shadowmap depth buffer from the attachment depth buffer
			//	to a shader resource; it is outside the render pass, hence the render pass doesn't need to
			//	include the subpass self-dependency
			insertImageMemoryBarrierBuf(
				curCmdBuf,
				m_vkShadowMapDepthImage,
				VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT,
				VK_ACCESS_SHADER_READ_BIT,
				VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
				VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
				0,
				VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
				VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
				imageSubresourceRange
				);

			if (vkEndCommandBuffer(curCmdBuf) != VK_SUCCESS)
			{
				// TODO: error
				printf("Failed to record shadowmap command buffer!\n");
			}
		}
	}
	void Wrapper::destroyShadowMapGenCommandBuffer()
	{
		vkFreeCommandBuffers(m_vkLogicalDeviceData.vkHandle, m_vkMainThreadCommandPool, 1, &m_vkShadowMapGenCommandBuffer);
	}

	void Wrapper::buildZPrePassCommandBuffer()
	{
		VkCommandBufferAllocateInfo commandBufferAllocateInfo = {};
		commandBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		commandBufferAllocateInfo.commandPool = m_vkMainThreadCommandPool;
		commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		commandBufferAllocateInfo.commandBufferCount = 1;

		if (vkAllocateCommandBuffers(m_vkLogicalDeviceData.vkHandle, &commandBufferAllocateInfo, &m_vkZPrePassCommandBuffer) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to allocate shadow map command buffer!\n");
		}
	}
	void Wrapper::updateZPrePassCommandBuffers()
	{
		VkCommandBuffer curCmdBuf = m_vkZPrePassCommandBuffer;

		{
			VkCommandBufferBeginInfo commandBufferBeginInfo = {};
			commandBufferBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
			// We're using `VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT` here to show that this
			//	command buffer will only be submitted once, to allow driver optimizations;
			//	this is indeed the case, since we're re-generating the command buffer each frame, and only call it once per frame
			commandBufferBeginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
			commandBufferBeginInfo.pInheritanceInfo = nullptr;

			vkBeginCommandBuffer(curCmdBuf, &commandBufferBeginInfo);

			const uint32_t numAttachments = 1;
			VkClearValue clearValues[numAttachments];
			clearValues[numAttachments-1].depthStencil = { 1.0f, 0 };

			VkRenderPassBeginInfo renderPassBeginInfo = {};
			renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
			renderPassBeginInfo.renderPass = m_vkZPrePassRenderPass;
			renderPassBeginInfo.framebuffer = m_vkZPrePassFramebuffer;
			renderPassBeginInfo.renderArea.offset = { 0, 0 };
			renderPassBeginInfo.renderArea.extent = m_vkSwapchainData.extent;
			renderPassBeginInfo.clearValueCount = numAttachments;
			renderPassBeginInfo.pClearValues = clearValues;

			vkCmdBeginRenderPass(curCmdBuf, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

			// Update dynamic viewport state
			VkViewport viewport = {};
			viewport.x = 0.0f;
			viewport.y = 0.0f;
			viewport.width = (float)m_vkSwapchainData.extent.width;
			viewport.height = (float)m_vkSwapchainData.extent.height;
			viewport.minDepth = (float) 0.0f;
			viewport.maxDepth = (float) 1.0f;
			vkCmdSetViewport(curCmdBuf, 0, 1, &viewport);

			// Update dynamic scissor state
			VkRect2D scissor = {};
			scissor.offset = { 0, 0 };
			scissor.extent = { m_vkSwapchainData.extent.width, m_vkSwapchainData.extent.height };
			vkCmdSetScissor(curCmdBuf, 0, 1, &scissor);

			// No depth bias for z-prepass
			vkCmdSetDepthBias(curCmdBuf, 0.0f, 0.0f, 0.0f);

			vkCmdBindPipeline(curCmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, m_vkDepthOnlyGraphicsPipeline);

			vkCmdBindDescriptorSets(
				curCmdBuf,
				VK_PIPELINE_BIND_POINT_GRAPHICS,
				m_vkDepthOnlyPipelineLayout,
				0,
				1,
				&m_vkZPrePassDescriptorSet,
				0,
				nullptr
				);

#if (SCENE == SCENE_QUAD)
#elif (SCENE == SCENE_MESH)
			for (int mi = 0, miEnd = (int)m_meshes.size(); mi < miEnd; ++mi)
			{
				ModelPushConstant pushConstant;

				const Mat44 & modelMat = m_meshes[mi]->getModelMatrix();
				pushConstant.model = modelMat.getTransposed();

				vkCmdPushConstants(
					curCmdBuf,
					m_vkDepthOnlyPipelineLayout,
					VK_SHADER_STAGE_VERTEX_BIT,
					0,
					sizeof(ModelPushConstant),
					&pushConstant
					);

				VkBuffer vertexBuffers[] = { m_meshes[mi]->getVertexBuffer() };
				VkDeviceSize offsets[] = { 0 };
				vkCmdBindVertexBuffers(curCmdBuf, 0, 1, vertexBuffers, offsets);
				vkCmdBindIndexBuffer(curCmdBuf, m_meshes[mi]->getIndexBuffer(), 0, m_meshes[mi]->getIndexBufferType());

				vkCmdDrawIndexed(curCmdBuf, (uint32_t)m_meshes[mi]->getIndicesCount(), 1, 0, 0, 0);
			}
#endif

			if (m_debugTrisMesh.getVerticesCount() > 0)
			{
				ModelPushConstant pushConstant;

				const Mat44 modelMat = m_debugTrisMesh.getModelMatrix();
				pushConstant.model = modelMat.getTransposed();

				vkCmdPushConstants(
					curCmdBuf,
					m_vkFwdShadingPipelineLayout,
					VK_SHADER_STAGE_VERTEX_BIT,
					0,
					sizeof(ModelPushConstant),
					&pushConstant
				);

				VkBuffer vertexBuffers[] = { m_debugTrisMesh.getVertexBuffer() };
				VkDeviceSize offsets[] = { 0 };
				vkCmdBindVertexBuffers(curCmdBuf, 0, 1, vertexBuffers, offsets);

				vkCmdDraw(curCmdBuf, (uint32_t)m_debugTrisMesh.getVerticesCount(), 1, 0, 0);
			}

			vkCmdEndRenderPass(curCmdBuf);

			if (vkEndCommandBuffer(curCmdBuf) != VK_SUCCESS)
			{
				// TODO: error
				printf("Failed to record z-prepass command buffer!\n");
			}
		}
	}
	void Wrapper::destroyZPrePassCommandBuffer()
	{
		vkFreeCommandBuffers(m_vkLogicalDeviceData.vkHandle, m_vkMainThreadCommandPool, 1, &m_vkZPrePassCommandBuffer);
	}

	void Wrapper::buildSecondaryCommandBuffers()
	{
		m_vkMainThreadCommandBuffers.resize(1);

		VkCommandBufferAllocateInfo commandBufferAllocateInfo = {};
		commandBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		commandBufferAllocateInfo.commandPool = m_vkMainThreadCommandPool;
		commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_SECONDARY;
		commandBufferAllocateInfo.commandBufferCount = (uint32_t)m_vkMainThreadCommandBuffers.size();

		if (vkAllocateCommandBuffers(m_vkLogicalDeviceData.vkHandle, &commandBufferAllocateInfo, m_vkMainThreadCommandBuffers.data()) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to allocate command buffers!\n");
		}
	}
	void Wrapper::updateSecondaryCommandBuffers(const VkCommandBufferInheritanceInfo & commandBufferInheritanceInfo)
	{
		VkCommandBuffer & curCmdBuf = m_vkMainThreadCommandBuffers[0];

		// Here, we are building the actual scene in this secondary command buffer
		//	and it receives information about the higher level render pass as a commandBufferInheritanceInfo

		{
			VkCommandBufferBeginInfo commandBufferBeginInfo = {};
			commandBufferBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
			// Here we set `VK_COMMAND_BUFFER_USAGE_RENDER_PASS_CONTINUE_BIT` flag to denote that
			//	this secondary command buffer is entirely within the render pass, otherwise it needs
			//	explicit render pass creation
			commandBufferBeginInfo.flags = VK_COMMAND_BUFFER_USAGE_RENDER_PASS_CONTINUE_BIT | VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
			commandBufferBeginInfo.pInheritanceInfo = &commandBufferInheritanceInfo;

			vkBeginCommandBuffer(curCmdBuf, &commandBufferBeginInfo);

			vkCmdBindPipeline(curCmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, m_vkFwdShadingGraphicsPipeline);

			vkCmdBindDescriptorSets(
				curCmdBuf,
				VK_PIPELINE_BIND_POINT_GRAPHICS,
				m_vkFwdShadingPipelineLayout,
				0,
				1,
				&m_vkDescriptorSet,
				0,
				nullptr
				);

#if (SCENE == SCENE_QUAD)
			{
				ModelPushConstant pushConstant;
				pushConstant.model.identity();

				vkCmdPushConstants(
					curCmdBuf,
					m_vkFwdShadingPipelineLayout,
					VK_SHADER_STAGE_VERTEX_BIT,
					0,
					sizeof(ModelPushConstant),
					&pushConstant
					);

				VkBuffer vertexBuffers[] = { m_vkTriangleVertexBuffer };
				VkDeviceSize offsets[] = { 0 };
				vkCmdBindVertexBuffers(curCmdBuf, 0, 1, vertexBuffers, offsets);
				vkCmdBindIndexBuffer(curCmdBuf, m_vkTriangleIndexBuffer, 0, m_vkTriangleIndexBufferType);
				vkCmdDrawIndexed(curCmdBuf, (uint32_t)m_vkTriangleIndicesCount, 1, 0, 0, 0);
			}
#elif (SCENE == SCENE_MESH)
			for (int mi = 0, miEnd = (int)m_meshes.size(); mi < miEnd; ++mi)
			{
				ModelPushConstant pushConstant;

				const Mat44 & modelMat = m_meshes[mi]->getModelMatrix();
				pushConstant.model = modelMat.getTransposed();

				vkCmdPushConstants(
					curCmdBuf,
					m_vkFwdShadingPipelineLayout,
					VK_SHADER_STAGE_VERTEX_BIT,
					0,
					sizeof(ModelPushConstant),
					&pushConstant
					);

				VkBuffer vertexBuffers[] = { m_meshes[mi]->getVertexBuffer() };
				VkDeviceSize offsets[] = { 0 };
				vkCmdBindVertexBuffers(curCmdBuf, 0, 1, vertexBuffers, offsets);
				vkCmdBindIndexBuffer(curCmdBuf, m_meshes[mi]->getIndexBuffer(), 0, m_meshes[mi]->getIndexBufferType());

				vkCmdDrawIndexed(curCmdBuf, (uint32_t)m_meshes[mi]->getIndicesCount(), 1, 0, 0, 0);
			}
#endif

			if (m_debugTrisMesh.getVerticesCount() > 0)
			{
				ModelPushConstant pushConstant;

				const Mat44 modelMat = m_debugTrisMesh.getModelMatrix();
				pushConstant.model = modelMat.getTransposed();

				vkCmdPushConstants(
					curCmdBuf,
					m_vkFwdShadingPipelineLayout,
					VK_SHADER_STAGE_VERTEX_BIT,
					0,
					sizeof(ModelPushConstant),
					&pushConstant
					);

				VkBuffer vertexBuffers[] = { m_debugTrisMesh.getVertexBuffer() };
				VkDeviceSize offsets[] = { 0 };
				vkCmdBindVertexBuffers(curCmdBuf, 0, 1, vertexBuffers, offsets);

				vkCmdDraw(curCmdBuf, (uint32_t)m_debugTrisMesh.getVerticesCount(), 1, 0, 0);
			}

			if (m_debugLinesMesh.getVerticesCount() > 0)
			{
				vkCmdBindPipeline(curCmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, m_vkDebugVisGraphicsPipeline);

				vkCmdBindDescriptorSets(
					curCmdBuf,
					VK_PIPELINE_BIND_POINT_GRAPHICS,
					m_vkDebugVisPipelineLayout,
					0,
					1,
					&m_vkDescriptorSet,
					0,
					nullptr
					);

				VkBuffer vertexBuffers[] = { m_debugLinesMesh.getVertexBuffer() };
				VkDeviceSize offsets[] = { 0 };
				vkCmdBindVertexBuffers(curCmdBuf, 0, 1, vertexBuffers, offsets);

				vkCmdDraw(curCmdBuf, (uint32_t)m_debugLinesMesh.getVerticesCount(), 1, 0, 0);
			}

			if (vkEndCommandBuffer(curCmdBuf) != VK_SUCCESS)
			{
				// TODO: error
				printf("Failed to record rendering command buffer!\n");
			}
		}
	}
	void Wrapper::destroySecondaryCommandBuffers()
	{
		vkFreeCommandBuffers(m_vkLogicalDeviceData.vkHandle, m_vkMainThreadCommandPool, (uint32_t)m_vkMainThreadCommandBuffers.size(), m_vkMainThreadCommandBuffers.data());
		m_vkMainThreadCommandBuffers.resize(0);
	}

	void Wrapper::buildSwapchainCommandBuffers()
	{
		// Command buffer outputs to a certain image, and since swapchain has several of them - we need several command buffers
		//m_vkMainThreadSwapchainCommandBuffers.resize(m_vkSwapchainData.framebuffers.size());
		m_vkMainThreadSwapchainCommandBuffers.resize(1);

		VkCommandBufferAllocateInfo commandBufferAllocateInfo = {};
		commandBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		commandBufferAllocateInfo.commandPool = m_vkMainThreadCommandPool;
		commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		commandBufferAllocateInfo.commandBufferCount = (uint32_t)m_vkMainThreadSwapchainCommandBuffers.size();

		if (vkAllocateCommandBuffers(m_vkLogicalDeviceData.vkHandle, &commandBufferAllocateInfo, m_vkMainThreadSwapchainCommandBuffers.data()) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to allocate swapchain command buffers!\n");
		}
	}
	void Wrapper::updateSwapchainCommandBuffers(const VkFramebuffer & framebuffer)
	{
		VkCommandBuffer & curSwapchainCmdBuf = m_vkMainThreadSwapchainCommandBuffers[0];

		// Here we update the swapchain (primary) command buffer, which only calls to the secondary command buffer
		//	and specifies rendering to the framebuffer passed as a parameter (in this exact case - swapchain framebuffer)

		{
			VkCommandBufferBeginInfo commandBufferBeginInfo = {};
			commandBufferBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
			// We're using `VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT` here to show that this
			//	command buffer will only be submitted once, to allow driver optimizxations;
			//	this is indeed the case, since we're re-generating the command buffer each frame, and only call it once per frame
			commandBufferBeginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
			commandBufferBeginInfo.pInheritanceInfo = nullptr;

			vkBeginCommandBuffer(curSwapchainCmdBuf, &commandBufferBeginInfo);

			const uint32_t numAttachments = 2;
			VkClearValue clearValues[numAttachments];
			clearValues[0].color = { 0.1f, 0.2f, 0.4f, 1.0f };
			clearValues[1].depthStencil = { 1.0f, 0 };			// Not important, depth has LOAD_OP_LOAD

			VkRenderPassBeginInfo renderPassBeginInfo = {};
			renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
			renderPassBeginInfo.renderPass = m_vkRenderPass;
			renderPassBeginInfo.framebuffer = framebuffer;
			renderPassBeginInfo.renderArea.offset = { 0, 0 };
			renderPassBeginInfo.renderArea.extent = m_vkSwapchainData.extent;
			renderPassBeginInfo.clearValueCount = numAttachments;
			renderPassBeginInfo.pClearValues = clearValues;

			// Here we changed `VK_SUBPASS_CONTENTS_INLINE` to `VK_SUBPASS_CONTENTS_SECONDARY_COMMAND_BUFFERS`
			//	to denote that we can use the secondary command buffers;
			//	This also means that within this render (sub-)pass, we can only call `vkCmdExecuteCommands` and nothing else
			vkCmdBeginRenderPass(curSwapchainCmdBuf, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_SECONDARY_COMMAND_BUFFERS);

			VkCommandBufferInheritanceInfo secondaryBufferInheritanceInfo = {};
			secondaryBufferInheritanceInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_INHERITANCE_INFO;
			secondaryBufferInheritanceInfo.renderPass = m_vkRenderPass;
			secondaryBufferInheritanceInfo.framebuffer = framebuffer;
			updateSecondaryCommandBuffers(secondaryBufferInheritanceInfo);

			vkCmdExecuteCommands(curSwapchainCmdBuf, (uint32_t)m_vkMainThreadCommandBuffers.size(), m_vkMainThreadCommandBuffers.data());

			vkCmdEndRenderPass(curSwapchainCmdBuf);

			if (vkEndCommandBuffer(curSwapchainCmdBuf) != VK_SUCCESS)
			{
				// TODO: error
				printf("Failed to record swapchain command buffer %zd!\n", (uint64_t)framebuffer);
			}
		}
	}
	void Wrapper::destroySwapchainCommandBuffers()
	{
		vkFreeCommandBuffers(m_vkLogicalDeviceData.vkHandle, m_vkMainThreadCommandPool, (uint32_t)m_vkMainThreadSwapchainCommandBuffers.size(), m_vkMainThreadSwapchainCommandBuffers.data());
		m_vkMainThreadSwapchainCommandBuffers.resize(0);
	}

	void Wrapper::initSemaphores()
	{
		VkSemaphoreCreateInfo semaphoreCreateInfo = {};
		semaphoreCreateInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

		if (vkCreateSemaphore(m_vkLogicalDeviceData.vkHandle, &semaphoreCreateInfo, getVkAllocator(), &m_vkSemaphoreImageAvailable) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create \"image available\" semaphore!\n");
		}
		if (vkCreateSemaphore(m_vkLogicalDeviceData.vkHandle, &semaphoreCreateInfo, getVkAllocator(), &m_vkSemaphoreZPrePassFinished) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create \"z-prepass finished\" semaphore!\n");
		}
		if (vkCreateSemaphore(m_vkLogicalDeviceData.vkHandle, &semaphoreCreateInfo, getVkAllocator(), &m_vkSemaphoreShadowMapGenFinished) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create \"shadowmap finished\" semaphore!\n");
		}
		if (vkCreateSemaphore(m_vkLogicalDeviceData.vkHandle, &semaphoreCreateInfo, getVkAllocator(), &m_vkSemaphoreRenderFinished) != VK_SUCCESS)
		{
			// TODO: error
			printf("Failed to create \"render finished\" semaphore!\n");
		}
	}
	
	void Wrapper::deinitSemaphores()
	{
		vkDestroySemaphore(m_vkLogicalDeviceData.vkHandle, m_vkSemaphoreRenderFinished, getVkAllocator());
		m_vkSemaphoreRenderFinished = VK_NULL_HANDLE;
		vkDestroySemaphore(m_vkLogicalDeviceData.vkHandle, m_vkSemaphoreShadowMapGenFinished, getVkAllocator());
		m_vkSemaphoreShadowMapGenFinished = VK_NULL_HANDLE;
		vkDestroySemaphore(m_vkLogicalDeviceData.vkHandle, m_vkSemaphoreZPrePassFinished, getVkAllocator());
		m_vkSemaphoreZPrePassFinished = VK_NULL_HANDLE;
		vkDestroySemaphore(m_vkLogicalDeviceData.vkHandle, m_vkSemaphoreImageAvailable, getVkAllocator());
		m_vkSemaphoreImageAvailable = VK_NULL_HANDLE;
	}

	void Wrapper::init(HWND hWnd, int width, int height)
	{
		m_hWnd = hWnd;
		m_windowWidth = width;
		m_windowHeight = height;

		m_viewMatrix.identity();

		m_viewPerspParameters.fovRad = PI / 2;
		m_viewPerspParameters.aspect = m_windowWidth / (float)m_windowHeight;
		m_viewPerspParameters.zNear = 0.1f;
		m_viewPerspParameters.zFar = 50.0f;
		m_viewPerspParameters.width = 1.0f;
		m_viewPerspParameters.height = 1.0f;

		buildRequiredInstanceExtensionsList(true);
		buildSupportedInstanceExtensionsList(true);

#ifdef NDEBUG
		m_enableValidationLayers = false;
#else
		m_enableValidationLayers = true;
#endif
		bool requiresDebugCallback = initValidationLayers(m_enableValidationLayers);

		initInstance();

		if (requiresDebugCallback)
		{
			initDebugCallback(m_vkDebugCallback);
		}

		initWindowSurface(hWnd);

		selectPhysicalDevice();

		initLogicalDevice();
		initCommandPool();

		initSwapchain();

		{
#if (SCENE == SCENE_QUAD)
			std::vector<char> vertShaderByteCode = readShaderFile("shaders/bin/test.vs.spv");
			std::vector<char> fragShaderByteCode = readShaderFile("shaders/bin/pathtracer.fs.spv");
			//std::vector<char> vertShaderByteCode = readShaderFile("shaders/bin/fs_blobs.vs.spv");
			//std::vector<char> fragShaderByteCode = readShaderFile("shaders/bin/fs_blobs.fs.spv");
#elif (SCENE == SCENE_MESH)
			std::vector<char> vertShaderByteCode = readShaderFile("shaders/bin/mesh.vs.spv");
			std::vector<char> fragShaderByteCode = readShaderFile("shaders/bin/mesh.fs.spv");
#endif

			VkShaderModule vertShaderModule = initShaderModule(vertShaderByteCode);
			VkShaderModule fragShaderModule = initShaderModule(fragShaderByteCode);

			m_vkShaderModules.push_back(vertShaderModule);
			m_vkShaderModules.push_back(fragShaderModule);
		}

		{
			std::vector<char> debugVisVSByteCode = readShaderFile("shaders/bin/debug_vis.vs.spv");
			std::vector<char> debugVisFSByteCode = readShaderFile("shaders/bin/debug_vis.fs.spv");

			VkShaderModule debugVisVertShaderModule = initShaderModule(debugVisVSByteCode);
			VkShaderModule debugVisFragShaderModule = initShaderModule(debugVisFSByteCode);

			m_vkShaderModules.push_back(debugVisVertShaderModule);
			m_vkShaderModules.push_back(debugVisFragShaderModule);
		}

		{
			std::vector<char> shadowmapVSByteCode = readShaderFile("shaders/bin/shadowmap.vs.spv");
			std::vector<char> shadowmapFSByteCode = readShaderFile("shaders/bin/shadowmap.fs.spv");

			VkShaderModule shadowmapVertShaderModule = initShaderModule(shadowmapVSByteCode);
			VkShaderModule shadowmapFragShaderModule = initShaderModule(shadowmapFSByteCode);

			m_vkShaderModules.push_back(shadowmapVertShaderModule);
			m_vkShaderModules.push_back(shadowmapFragShaderModule);
		}

		initDepthBuffer();
		initShadowMapDepthBuffer();
		initRenderPass();
		initZPrePassRenderPass();
		initShadowMapGenRenderPass();
		initDescriptorSetLayout();
		initDepthOnlyDescriptorSetLayout();
		initGlobalConstantsUBO();
		initTransformUBO();
		initShadowMapGenTransformUBO();
		initAlbedoTexResource();
		initLightProjTexResource();
		initTextureSampler();
		initDepthTextureSampler();
		initDescriptorPool();
		initDescriptorSet();
		updateDescriptorSetBindings();
		initZPrePassDescriptorSet();
		updateZPrePassDescriptorSetBindings();
		initShadowMapGenDescriptorSet();
		updateShadowMapGenDescriptorSetBindings();
		initPipelineState();
		initDepthOnlyPipelineState();

		// TODO: check if they should be Dynamic?
		m_debugTrisMesh.init(this);
		m_debugTrisMesh.initBuffers(BufferUsage::eStatic, 0, sizeof(Vertex), BufferUsage::eStatic, 0);

		m_debugLinesMesh.init(this);
		m_debugLinesMesh.initBuffers(BufferUsage::eStatic, 0, sizeof(LinePoint), BufferUsage::eStatic, 0);
		initDebugPipelineState();

		initSwapchainFramebuffers();
		initZPrePassFramebuffers();
		initShadowMapGenFramebuffers();

		initFSQuadBuffers();
		buildSecondaryCommandBuffers();
		buildSwapchainCommandBuffers();
		buildZPrePassCommandBuffer();
		buildShadowMapGenCommandBuffer();

		initSemaphores();
	}

	void Wrapper::deinit()
	{
		// Wait before the last frame is fully rendered
		vkDeviceWaitIdle(m_vkLogicalDeviceData.vkHandle);

		deinitSemaphores();

		// No need to call destroyCommandBuffers as this will be done automatically by Vulkan on command pool deinitialization

		for (int mi = 0, miEnd = (int)m_meshes.size(); mi < miEnd; ++mi)
		{
			m_meshes[mi]->deinit();
			destroyMesh(m_meshes[mi]);
		}
		m_meshes.resize(0);

		for (int tri = 0, triEnd = (int)m_textureResources.size(); tri < triEnd; ++tri)
		{
			deinitTextureResource(m_textureResources[tri]);
			destroyTextureResource(m_textureResources[tri]);
		}
		m_textureResources.resize(0);

		m_debugLinesMesh.deinit();
		m_debugTrisMesh.deinit();

		deinitFSQuadBuffers();

		deinitShadowMapGenFramebuffers();
		deinitZPrePassFramebuffers();
		deinitSwapchainFramebuffers();
		deinitDebugPipelineState();
		deinitDepthOnlyPipelineState();
		deinitPipelineState();
		deinitShadowMapGenDescriptorSet();
		deinitZPrePassDescriptorSet();
		deinitDescriptorSet();
		deinitDescriptorPool();
		deinitDepthTextureSampler();
		deinitTextureSampler();
		deinitLightProjTexResource();
		deinitAlbedoTexResource();
		deinitShadowMapGenTransformUBO();
		deinitTransformUBO();
		deinitGlobalConstantsUBO();
		deinitDepthOnlyDescriptorSetLayout();
		deinitDescriptorSetLayout();
		deinitShadowMapGenRenderPass();
		deinitZPrePassRenderPass();
		deinitRenderPass();
		deinitShadowMapDepthBuffer();
		deinitDepthBuffer();
		deinitShaderModules();
		deinitSwapchain();
		deinitCommandPool();
		deinitLogicalDevice();
		deinitDebugCallback();
		deinitWindowSurface();
		deinitInstance();
	}

	void Wrapper::beginFrame()
	{
		// Sync to the presenting queue here in order to allow CPU/GPU code to overlap
		vkQueueWaitIdle(m_vkLogicalDeviceData.presentingQueue);
	}

	void Wrapper::update()
	{
		m_elapsedTimeMS += m_dtMS;

		GlobalConstantsUBO ubo = {};
		ubo.time = (float)m_elapsedTimeMS;

		void * data;
		vkMapMemory(m_vkLogicalDeviceData.vkHandle, m_vkGlobalConstantsUBOBufferDeviceMemory, 0, sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(m_vkLogicalDeviceData.vkHandle, m_vkGlobalConstantsUBOBufferDeviceMemory);

		//vkMapMemory(m_vkLogicalDeviceData.vkHandle, m_vkGlobalConstantsUBOBufferDeviceMemory, 0, sizeof(ubo), 0, &data);
		//memcpy(data, &ubo, sizeof(ubo));
		//vkUnmapMemory(m_vkLogicalDeviceData.vkHandle, m_vkGlobalConstantsUBOBufferDeviceMemory);

		TransformUBO transformUBO = {};
		transformUBO.view = Mat44(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
			);
#if (SCENE == SCENE_MESH)
		transformUBO.view = m_viewMatrix;
		transformUBO.view.transpose();
#endif

		transformUBO.proj = Mat44(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, -1.f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
			);
#if (SCENE == SCENE_MESH)
		m_viewPerspParameters.aspect = m_vkSwapchainData.extent.width / (float)m_vkSwapchainData.extent.height;
		projPerspective(
			m_viewPerspParameters.fovRad,
			m_viewPerspParameters.aspect,
			m_viewPerspParameters.zNear,
			m_viewPerspParameters.zFar,
			m_viewPerspParameters.width,
			m_viewPerspParameters.height,
			&transformUBO.proj
			);
		transformUBO.proj.transpose();
#endif

		void * transformData;
		vkMapMemory(m_vkLogicalDeviceData.vkHandle, m_vkTransformUBOBufferDeviceMemory, 0, sizeof(transformUBO), 0, &transformData);
		memcpy(transformData, &transformUBO, sizeof(transformUBO));
		vkUnmapMemory(m_vkLogicalDeviceData.vkHandle, m_vkTransformUBOBufferDeviceMemory);

		{
			TransformUBO lightMatrixUBO = {};
			lightMatrixUBO.view.identity();

			lightMatrixUBO.view = m_lightViewMatrix;
			lightMatrixUBO.view.transpose();

#if (SCENE == SCENE_MESH)
			const float asp = m_smWidth / (float)m_smHeight;
			projPerspective(
				m_lightPerspParameters.fovRad,
				m_lightPerspParameters.aspect,
				m_lightPerspParameters.zNear,
				m_lightPerspParameters.zFar,
				m_lightPerspParameters.width,
				m_lightPerspParameters.height,
				&lightMatrixUBO.proj
				);
			lightMatrixUBO.proj.transpose();
#else
			lightMatrixUBO.proj = Mat44(
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, -1.f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
				);
#endif

			void * lightMatrixData;
			vkMapMemory(m_vkLogicalDeviceData.vkHandle, m_vkShadowMapGenTransformUBOBufferDeviceMemory, 0, sizeof(lightMatrixUBO), 0, &lightMatrixData);
			memcpy(lightMatrixData, &lightMatrixUBO, sizeof(transformUBO));
			vkUnmapMemory(m_vkLogicalDeviceData.vkHandle, m_vkShadowMapGenTransformUBOBufferDeviceMemory);
		}

		// Temporary sync point to allow for correct debug line vis buffer deletion
		m_debugTrisMesh.updateResizeVertexBuffer(m_debugTris.data(), (uint32_t)m_debugTris.size());
		m_debugTris.resize(0);

		m_debugLinesMesh.updateResizeVertexBuffer(m_debugLines.data(), (uint32_t)m_debugLines.size());
		m_debugLines.resize(0);

		m_dtMS = 0.0;
	}

	void Wrapper::render()
	{
		DBG_RENDER_FORCED_SYNC_IF_REQ(m_vkLogicalDeviceData.vkHandle);

		{
			VkResult result = vkAcquireNextImageKHR(
				m_vkLogicalDeviceData.vkHandle,
				m_vkSwapchainData.vkHandle,
				std::numeric_limits<uint64_t>::max(),
				m_vkSemaphoreImageAvailable,
				VK_NULL_HANDLE,
				&m_vkSwapchainData.imageIndexInSwapchain
				);

			// VK_SUBOPTIMAL_KHR can be reported here, and it is not exactly a very bad thing, so no actions on that at the moment
			if (result == VK_ERROR_OUT_OF_DATE_KHR)
			{
				reinitSwapchain();
				printf("Swapchain out of date!\n");
				return;
			}
			else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR)
			{
				// TODO: error
				printf("Failed to acquire next image!\n");
				return;
			}
		}

		DBG_RENDER_FORCED_SYNC_IF_REQ(m_vkLogicalDeviceData.vkHandle);

		updateZPrePassCommandBuffers();

		DBG_RENDER_FORCED_SYNC_IF_REQ(m_vkLogicalDeviceData.vkHandle);

#define ADDITIONAL_SEMAPHORES 1

		{
#if (ADDITIONAL_SEMAPHORES == 1)
			const uint32_t numWaitSemaphores = 1;
			VkSemaphore renderBegSemaphore[numWaitSemaphores] = { m_vkSemaphoreImageAvailable };
			VkPipelineStageFlags pipelineWaitStages[numWaitSemaphores] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };

			const uint32_t numSignalSemaphores = 1;
			VkSemaphore renderEndSemaphores[] = { m_vkSemaphoreZPrePassFinished };
#else
			const uint32_t numWaitSemaphores = 0;
			VkSemaphore * renderBegSemaphore = nullptr;
			VkPipelineStageFlags * pipelineWaitStages = nullptr;

			const uint32_t numSignalSemaphores = 0;
			VkSemaphore * renderEndSemaphores = nullptr;
#endif

			VkSubmitInfo submitInfo = {};
			submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
			submitInfo.waitSemaphoreCount = numWaitSemaphores;
			submitInfo.pWaitSemaphores = renderBegSemaphore;
			submitInfo.pWaitDstStageMask = pipelineWaitStages;
			submitInfo.commandBufferCount = 1;
			submitInfo.pCommandBuffers = &m_vkZPrePassCommandBuffer;
			submitInfo.signalSemaphoreCount = numSignalSemaphores;
			submitInfo.pSignalSemaphores = renderEndSemaphores;

			if (vkQueueSubmit(m_vkLogicalDeviceData.graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE) != VK_SUCCESS)
			{
				// TODO: warning
				printf("Failed to submit z-prepass command buffer!\n");
			}
		}

		DBG_RENDER_FORCED_SYNC_IF_REQ(m_vkLogicalDeviceData.vkHandle);

		updateShadowMapGenCommandBuffers();

		DBG_RENDER_FORCED_SYNC_IF_REQ(m_vkLogicalDeviceData.vkHandle);

		{
#if (ADDITIONAL_SEMAPHORES == 1)
			const uint32_t numWaitSemaphores = 1;
			VkSemaphore renderBegSemaphore[numWaitSemaphores] = { m_vkSemaphoreZPrePassFinished };
			VkPipelineStageFlags pipelineWaitStages[numWaitSemaphores] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };

			const uint32_t numSignalSemaphores = 1;
			VkSemaphore renderEndSemaphores[] = { m_vkSemaphoreShadowMapGenFinished };
#else
			const uint32_t numWaitSemaphores = 0;
			VkSemaphore * renderBegSemaphore = nullptr;
			VkPipelineStageFlags * pipelineWaitStages = nullptr;

			const uint32_t numSignalSemaphores = 0;
			VkSemaphore * renderEndSemaphores = nullptr;
#endif

			VkSubmitInfo submitInfo = {};
			submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
			submitInfo.waitSemaphoreCount = numWaitSemaphores;
			submitInfo.pWaitSemaphores = renderBegSemaphore;
			submitInfo.pWaitDstStageMask = pipelineWaitStages;
			submitInfo.commandBufferCount = 1;
			submitInfo.pCommandBuffers = &m_vkShadowMapGenCommandBuffer;
			submitInfo.signalSemaphoreCount = numSignalSemaphores;
			submitInfo.pSignalSemaphores = renderEndSemaphores;

			if (vkQueueSubmit(m_vkLogicalDeviceData.graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE) != VK_SUCCESS)
			{
				// TODO: warning
				printf("Failed to submit z-prepass command buffer!\n");
			}
		}

		DBG_RENDER_FORCED_SYNC_IF_REQ(m_vkLogicalDeviceData.vkHandle);

		updateSwapchainCommandBuffers(m_vkSwapchainData.framebuffers[m_vkSwapchainData.imageIndexInSwapchain]);

		DBG_RENDER_FORCED_SYNC_IF_REQ(m_vkLogicalDeviceData.vkHandle);

		{
#if (ADDITIONAL_SEMAPHORES == 1)
			const uint32_t numWaitSemaphores = 1;
			VkSemaphore renderBegSemaphore[numWaitSemaphores] = { m_vkSemaphoreShadowMapGenFinished };
			VkPipelineStageFlags pipelineWaitStages[numWaitSemaphores] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
#else
			const uint32_t numWaitSemaphores = 1;
			VkSemaphore renderBegSemaphore[numWaitSemaphores] = { m_vkSemaphoreImageAvailable };
			VkPipelineStageFlags pipelineWaitStages[numWaitSemaphores] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
#endif

			const uint32_t numSignalSemaphores = 1;
			VkSemaphore renderEndSemaphores[] = { m_vkSemaphoreRenderFinished };

			VkSubmitInfo submitInfo = {};
			submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
			submitInfo.waitSemaphoreCount = numWaitSemaphores;
			submitInfo.pWaitSemaphores = renderBegSemaphore;
			submitInfo.pWaitDstStageMask = pipelineWaitStages;
			submitInfo.commandBufferCount = 1;
			submitInfo.pCommandBuffers = &m_vkMainThreadSwapchainCommandBuffers[0];
			submitInfo.signalSemaphoreCount = numSignalSemaphores;
			submitInfo.pSignalSemaphores = renderEndSemaphores;

			if (vkQueueSubmit(m_vkLogicalDeviceData.graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE) != VK_SUCCESS)
			{
				// TODO: warning
				printf("Failed to submit draw command buffer!\n");
			}
		}

		DBG_RENDER_FORCED_SYNC_IF_REQ(m_vkLogicalDeviceData.vkHandle);

		VkSwapchainKHR swapChains[] = { m_vkSwapchainData.vkHandle };

		const uint32_t numSwapchainWaitSemaphores = 1;
		VkSemaphore swapchainWaitSemaphores[numSwapchainWaitSemaphores] = { m_vkSemaphoreRenderFinished };

		VkPresentInfoKHR presentInfo = {};
		presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
		presentInfo.waitSemaphoreCount = numSwapchainWaitSemaphores;
		presentInfo.pWaitSemaphores = swapchainWaitSemaphores;
		presentInfo.swapchainCount = 1;
		presentInfo.pSwapchains = swapChains;
		presentInfo.pImageIndices = &m_vkSwapchainData.imageIndexInSwapchain;
		presentInfo.pResults = nullptr;

		{
			VkResult result = vkQueuePresentKHR(m_vkLogicalDeviceData.presentingQueue, &presentInfo);

			// VK_SUBOPTIMAL_KHR can be reported here, and it is not exactly a very bad thing, so no actions on that at the moment
			if (result == VK_ERROR_OUT_OF_DATE_KHR)
			{
				reinitSwapchain();
				printf("Swapchain out of date on present!\n");
				return;
			}
			else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR)
			{
				// TODO: error
				printf("Failed to present image!\n");
				return;
			}
		}

		DBG_RENDER_FORCED_SYNC_IF_REQ(m_vkLogicalDeviceData.vkHandle);

		if (m_captureRequested)
		{
			std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
			//std::put_time(std::localtime(&now_c), "%F %T");
			std::chrono::system_clock::duration tp = now.time_since_epoch();
			tp -= std::chrono::duration_cast<std::chrono::seconds>(tp);
			time_t tt = std::chrono::system_clock::to_time_t(now);

			const size_t filenameSize = 128;
			wchar_t filename[filenameSize];

			tm t;
			//gmtime_s(&t, &tt);
			localtime_s(&t, &tt);

			swprintf_s(filename, filenameSize, L"screenshot %04u-%02u-%02u %02u_%02u_%02u_%03u.tga",
				t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec,
				static_cast<unsigned>(tp / std::chrono::milliseconds(1))
				);

			storeSwapchainImage(filename);
			m_captureRequested = false;
		}
	}

}