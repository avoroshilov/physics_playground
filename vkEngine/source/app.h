#pragma once

#include "vulkan\manager.h"

class App
{
protected:

	bool m_isExitting = false;

	int m_width = -1, m_height = -1;
	HWND m_hWnd = nullptr;
	vulkan::Wrapper * m_renderingWrapper;

	double m_elapsedTimeMS = 0.0;

public:

	void setRenderManager(vulkan::Wrapper * renderingWrapper)
	{
		m_renderingWrapper = renderingWrapper;
	}

	void init(HWND hWnd, int width, int height)
	{
		m_width = width;
		m_height = height;
		m_hWnd = hWnd;
		m_renderingWrapper->init(m_hWnd, m_width, m_height);
	}
	void deinit()
	{
		m_renderingWrapper->deinit();
	}

	double getElapsedTime() const { return m_elapsedTimeMS; }

	void update(double dtMS)
	{
		m_elapsedTimeMS += dtMS;
		m_renderingWrapper->increaseDTime(dtMS);
	}

	void onWindowResize(int width, int height)
	{
		if (width == 0 || height == 0)
			return;

		m_width = width;
		m_height = height;

		m_renderingWrapper->onWindowResize(width, height);
	}

	void setIsExitting(bool isExitting) { m_isExitting = isExitting; }
	bool getIsExitting() const { return m_isExitting; }

	void requestCapture(bool captureRequested)
	{
		m_renderingWrapper->requestCapture(captureRequested);
	}

	static const int c_titleBufSize = 256;
	wchar_t m_titleBuf[c_titleBufSize];
	void setWindowTitle(wchar_t * title)
	{
		swprintf_s(m_titleBuf, c_titleBufSize, L"%s", title);
		SetWindowText(m_hWnd, m_titleBuf);
	}

	void setDTime(double dtimeMS)
	{
		const int titleBufSizeAdditional = 16;
		const int titleBufSizeAugmented = c_titleBufSize + titleBufSizeAdditional;
		wchar_t titleBuf[titleBufSizeAugmented];
		swprintf_s(titleBuf, titleBufSizeAugmented, L"%s: %.1f (%.3f ms)", m_titleBuf, 1000.0 / dtimeMS, dtimeMS);
		SetWindowText(m_hWnd, titleBuf);
	}
};

struct CallbackData
{
	App * app;

	enum class MovementKindBits
	{
		eForward = (1 << 0),
		eBackward = (1 << 1),
		eLeft = (1 << 2),
		eRight = (1 << 3),
		eUp = (1 << 4),
		eDown = (1 << 5),
		eAccel = (1 << 6),
		eDeccel = (1 << 7),
	};
	uint32_t movementFlags;

	bool reqDropBox;

	bool isActive;
	bool isPaused;
	bool isAnimStep;

	enum class MouseMode
	{
		eCamera = 0,
		ePicking = 1,

		eNUM_ENTRIES,
		eStartingMode = eCamera
	};
	MouseMode mouseMode;

	int lmbState;

	// Mouse coordinates
	int mx, my;

	int dmx, dmy;
	int dwheel;
};
