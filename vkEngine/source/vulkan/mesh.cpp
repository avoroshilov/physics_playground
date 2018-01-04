#include "manager.h"
#include "mesh.h"
#include "helpers.h"

namespace vulkan
{

void Mesh::createVertexBuffer(uint32_t numVertices, size_t vertexSize)
{
	m_meshData.vertexSize = vertexSize;
	m_meshData.verticesCount = numVertices;
	m_meshData.verticesCapacity = numVertices;
	m_meshData.vertexBufferDeviceMemorySize = numVertices * vertexSize;

	VkBufferUsageFlags vertexBufUsage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
	if (m_vertexBufUsage == BufferUsage::eStatic)
	{
		vertexBufUsage |= VK_BUFFER_USAGE_TRANSFER_DST_BIT;
	}

	VkMemoryPropertyFlags vertexBufMemoryProps;
	if (m_vertexBufUsage == BufferUsage::eStatic)
	{
		vertexBufMemoryProps = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
	}
	else if (m_vertexBufUsage == BufferUsage::eDynamic)
	{
		vertexBufMemoryProps = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
	}

	if (numVertices > 0)
	{
		createBuffer(
			m_parentWrapper->getPhysicalDeviceHandle(),
			m_parentWrapper->getLogicalDeviceHandle(),
			(VkDeviceSize)m_meshData.vertexBufferDeviceMemorySize,
			vertexBufUsage,
			vertexBufMemoryProps,
			&m_meshData.vertexBuffer,
			&m_meshData.vertexBufferDeviceMemory,
			m_parentWrapper->getVkAllocator()
			);
	}
}

void Mesh::createIndexBuffer(uint32_t numIndices, size_t indexSize)
{
	if (indexSize == sizeof(uint16_t))
		m_meshData.indexBufferType = VK_INDEX_TYPE_UINT16;
	else if (indexSize == sizeof(uint32_t))
		m_meshData.indexBufferType = VK_INDEX_TYPE_UINT32;
	else
	{
		// TOOD: error
		printf("Unsupported index size!\n");
	}

	m_meshData.indicesCount = numIndices;
	m_meshData.indexBufferDeviceMemorySize = indexSize * numIndices;

	VkBufferUsageFlags indexBufUsage = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
	if (m_indexBufUsage == BufferUsage::eStatic)
	{
		indexBufUsage |= VK_BUFFER_USAGE_TRANSFER_DST_BIT;
	}

	VkMemoryPropertyFlags indexBufMemoryProps;
	if (m_indexBufUsage == BufferUsage::eStatic)
	{
		indexBufMemoryProps = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
	}
	else if (m_indexBufUsage == BufferUsage::eDynamic)
	{
		indexBufMemoryProps = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
	}

	if (numIndices > 0)
	{
		createBuffer(
			m_parentWrapper->getPhysicalDeviceHandle(),
			m_parentWrapper->getLogicalDeviceHandle(),
			(VkDeviceSize)m_meshData.indexBufferDeviceMemorySize,
			indexBufUsage,
			indexBufMemoryProps,
			&m_meshData.indexBuffer,
			&m_meshData.indexBufferDeviceMemory,
			m_parentWrapper->getVkAllocator()
			);
	}
}

void Mesh::deleteVertexBuffer()
{
	const VkDevice & logicalDev = m_parentWrapper->getLogicalDeviceHandle();

	if (m_meshData.vertexBuffer != VK_NULL_HANDLE)
	{
		vkDestroyBuffer(logicalDev, m_meshData.vertexBuffer, m_parentWrapper->getVkAllocator());
		m_meshData.vertexBuffer = VK_NULL_HANDLE;
	}
	if (m_meshData.vertexBufferDeviceMemory != VK_NULL_HANDLE)
	{
		vkFreeMemory(logicalDev, m_meshData.vertexBufferDeviceMemory, m_parentWrapper->getVkAllocator());
		m_meshData.vertexBufferDeviceMemory = VK_NULL_HANDLE;
	}
	m_meshData.verticesCount = -1;
	m_meshData.verticesCapacity = 0;
}

void Mesh::deleteIndexBuffer()
{
	const VkDevice & logicalDev = m_parentWrapper->getLogicalDeviceHandle();

	if (m_meshData.indexBuffer != VK_NULL_HANDLE)
	{
		vkDestroyBuffer(logicalDev, m_meshData.indexBuffer, m_parentWrapper->getVkAllocator());
		m_meshData.indexBuffer = VK_NULL_HANDLE;
	}
	if (m_meshData.indexBufferDeviceMemory != VK_NULL_HANDLE)
	{
		vkFreeMemory(logicalDev, m_meshData.indexBufferDeviceMemory, m_parentWrapper->getVkAllocator());
		m_meshData.indexBufferDeviceMemory = VK_NULL_HANDLE;
	}
	m_meshData.indicesCount = -1;
	m_meshData.indicesCapacity = 0;
}

void Mesh::updateBufferStaging(const VkBuffer & targetBuffer, const void * bufferData, size_t bufferSize)
{
	if (bufferSize == 0)
		return;

	VkBuffer stagingBuffer;
	VkDeviceMemory stagingBufferDeviceMemory;
	createBuffer(
		m_parentWrapper->getPhysicalDeviceHandle(),
		m_parentWrapper->getLogicalDeviceHandle(),
		(VkDeviceSize)bufferSize,
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		&stagingBuffer,
		&stagingBufferDeviceMemory,
		m_parentWrapper->getVkAllocator()
		);

	const VkDevice & logicalDev = m_parentWrapper->getLogicalDeviceHandle();

	void * data = nullptr;
	vkMapMemory(logicalDev, stagingBufferDeviceMemory, 0, (VkDeviceSize)bufferSize, 0, &data);
	memcpy(data, bufferData, bufferSize);
	vkUnmapMemory(logicalDev, stagingBufferDeviceMemory);

	m_parentWrapper->copyBuffer(stagingBuffer, targetBuffer, (VkDeviceSize)bufferSize);

	vkDestroyBuffer(logicalDev, stagingBuffer, m_parentWrapper->getVkAllocator());
	vkFreeMemory(logicalDev, stagingBufferDeviceMemory, m_parentWrapper->getVkAllocator());
}

void Mesh::updateBufferDirect(const VkDeviceMemory & bufferDeviceMemory, const void * bufferData, size_t bufferSize)
{
	const VkDevice & logicalDev = m_parentWrapper->getLogicalDeviceHandle();

	void * data;
	vkMapMemory(logicalDev, bufferDeviceMemory, 0, (VkDeviceSize)bufferSize, 0, &data);
	memcpy(data, bufferData, bufferSize);
	vkUnmapMemory(logicalDev, bufferDeviceMemory);
}

void Mesh::initBuffers(BufferUsage vertexBufferUsage, uint32_t numVertices, size_t vertexSize, BufferUsage indexBufferUsage, uint32_t numIndices)
{
	m_vertexBufUsage = vertexBufferUsage;
	m_indexBufUsage = indexBufferUsage;

	createVertexBuffer(numVertices, vertexSize);
	createIndexBuffer(numIndices);
}

void Mesh::deinitBuffers()
{
	const VkDevice & logicalDev = m_parentWrapper->getLogicalDeviceHandle();

	deleteIndexBuffer();
	deleteVertexBuffer();
}

void Mesh::updateVertexBuffer(const void * bufferData)
{
	size_t vertexBufferSize = m_meshData.verticesCount * m_meshData.vertexSize;
	if (m_vertexBufUsage == BufferUsage::eStatic)
	{
		updateBufferStaging(m_meshData.vertexBuffer, bufferData, vertexBufferSize);
	}
	else if (m_vertexBufUsage == BufferUsage::eDynamic)
	{
		updateBufferDirect(m_meshData.vertexBufferDeviceMemory, bufferData, vertexBufferSize);
	}
}

void Mesh::updateResizeVertexBuffer(const void * bufferData, uint32_t numVertices)
{
	if ((int)numVertices > m_meshData.verticesCapacity)
	{
		size_t vertexSize = m_meshData.vertexSize;

		if (m_meshData.verticesCount > 0)
		{
			vkQueueWaitIdle(m_parentWrapper->getGraphicsQueue());
			deleteVertexBuffer();
		}

		createVertexBuffer(numVertices, vertexSize);
	}

	m_meshData.verticesCount = numVertices;
	updateVertexBuffer(bufferData);
}

void Mesh::updateIndexBuffer(const void * bufferData)
{
	size_t indexBufferSize = m_meshData.indicesCount * m_meshData.indexSize;
	if (m_indexBufUsage == BufferUsage::eStatic)
	{
		updateBufferStaging(m_meshData.indexBuffer, bufferData, indexBufferSize);
	}
	else if (m_indexBufUsage == BufferUsage::eDynamic)
	{
		updateBufferDirect(m_meshData.indexBufferDeviceMemory, bufferData, indexBufferSize);
	}
}

void Mesh::updateResizeIndexBuffer(const void * bufferData, uint32_t numIndices)
{
	const VkDevice & logicalDev = m_parentWrapper->getLogicalDeviceHandle();

	if ((int)numIndices > m_meshData.indicesCapacity)
	{
		size_t indexSize = m_meshData.indexSize;

		if (m_meshData.indicesCount > 0)
		{
			vkDeviceWaitIdle(logicalDev);
			deleteIndexBuffer();
		}

		createIndexBuffer(numIndices, indexSize);
	}

	m_meshData.indicesCount = numIndices;
	updateIndexBuffer(bufferData);
}

}