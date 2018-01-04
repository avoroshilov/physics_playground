#pragma once

#include <vulkan\vulkan.h>

#include "definitions.h"

namespace vulkan
{
	class Wrapper;

	class Mesh
	{
	protected:

		BufferUsage m_vertexBufUsage;
		BufferUsage m_indexBufUsage;

		struct VulkanMeshData
		{
			int verticesCount = -1;
			size_t vertexSize = 0;
			size_t verticesCapacity = 0;
			VkBuffer vertexBuffer = VK_NULL_HANDLE;
			size_t vertexBufferDeviceMemorySize = 0;
			VkDeviceMemory vertexBufferDeviceMemory = VK_NULL_HANDLE;

			int indicesCount = -1;
			size_t indexSize = sizeof(uint16_t);
			size_t indicesCapacity = 0;
			VkIndexType indexBufferType = VK_INDEX_TYPE_UINT16;
			VkBuffer indexBuffer = VK_NULL_HANDLE;
			size_t indexBufferDeviceMemorySize = 0;
			VkDeviceMemory indexBufferDeviceMemory = VK_NULL_HANDLE;
		};
		VulkanMeshData m_meshData;

		math::Mat44 m_viewMatrix;

		Wrapper * m_parentWrapper = nullptr;

		void createVertexBuffer(uint32_t numVertices, size_t vertexSize);
		void createIndexBuffer(uint32_t numIndices, size_t indexSize = sizeof(uint16_t));

		void deleteVertexBuffer();
		void deleteIndexBuffer();

		void updateBufferStaging(const VkBuffer & targetBuffer, const void * bufferData, size_t bufferSize);
		void updateBufferDirect(const VkDeviceMemory & bufferDeviceMemory, const void * bufferData, size_t bufferSize);

	public:

		void init(Wrapper * vulkanWrapper)
		{
			m_parentWrapper = vulkanWrapper;
			m_viewMatrix.identity();
		}
		void deinit()
		{
			deinitBuffers();
		}

		void initBuffers(BufferUsage vertexBufferUsage, uint32_t numVertices, size_t vertexSize, BufferUsage indexBufferUsage, uint32_t numIndices);
		void deinitBuffers();

		VkBuffer getVertexBuffer() const { return m_meshData.vertexBuffer; }
		int getVerticesCount() const { return m_meshData.verticesCount; }
		VkBuffer getIndexBuffer() const { return m_meshData.indexBuffer; }
		VkIndexType getIndexBufferType() const { return m_meshData.indexBufferType; }
		int getIndicesCount() const { return m_meshData.indicesCount; }

		void setModelMatrix(const math::Mat44 & mat44) { m_viewMatrix = mat44; }
		math::Mat44 & getModelMatrix() { return m_viewMatrix; }
		const math::Mat44 & getModelMatrix() const { return m_viewMatrix; }

		void updateVertexBuffer(const void * bufferData);
		void updateResizeVertexBuffer(const void * bufferData, uint32_t numVertices);
		void updateIndexBuffer(const void * bufferData);
		void updateResizeIndexBuffer(const void * bufferData, uint32_t numIndices);
	};

}