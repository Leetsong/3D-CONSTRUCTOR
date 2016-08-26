#include "kinectreceiver.h"

KinectReceiver::KinectReceiver() :
	m_pCloudSet(nullptr),
	m_animate(false),
	m_updatePending(false),
	m_nCurRelativeTime(-1),
	m_pKinectSensor(nullptr),
	m_pMultiSourceFrameReader(nullptr),
	m_pCoordinateMapper(nullptr) {
	// Create storage for color pixel data in RGBX format
	m_pColorRGBX = new RGBQUAD[COLOR_WIDTH * COLOR_HEIGHT];
	// Create storage for the coordinate mapping from depth to camera space
	m_pDepthFrameToCameraSpace = new CameraSpacePoint[DEPTH_WIDTH * DEPTH_HEIGHT];
	// Create storage for the coordinate mapping from depth to color space
	m_pColorFrameToDepthSpace = new DepthSpacePoint[COLOR_WIDTH * COLOR_HEIGHT];
}

KinectReceiver::~KinectReceiver() {
	if (m_pColorRGBX) delete m_pColorRGBX;
	if (m_pDepthFrameToCameraSpace) delete m_pDepthFrameToCameraSpace;
	if (m_pColorFrameToDepthSpace) delete m_pColorFrameToDepthSpace;

	SafeRelease(m_pCoordinateMapper);
	SafeRelease(m_pMultiSourceFrameReader);
	if (m_pKinectSensor) {
		m_pKinectSensor->Close();
		SafeRelease(m_pKinectSensor);
	}
}

bool KinectReceiver::initialize() {
	if (FAILED(InitDefaultSensor())) {
		PRINT_INFO("initialized: ERROR.\n");

		return false;
	}

	PRINT_INFO("initialized.\n");

	return true;
}

void KinectReceiver::update() {
	IMultiSourceFrame* pMultiSourceFrame = nullptr;
	IDepthFrame* pDepthFrame = nullptr;
	IColorFrame* pColorFrame = nullptr;
	//==IBodyIndexFrame* pBodyIndexFrame = nullptr;

	// Get latest multiframe
	HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

	// Get depth frame
	if (SUCCEEDED(hr)) {
		IDepthFrameReference* pDepthFrameReference = nullptr;

		hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hr)) {
			hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);

			PRINT_INFO("got depth frame\n");
		}

		SafeRelease(pDepthFrameReference);
	}

	// Get color frame
	if (SUCCEEDED(hr)) {
		IColorFrameReference* pColorFrameReference = nullptr;

		hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		if (SUCCEEDED(hr)) {
			hr = pColorFrameReference->AcquireFrame(&pColorFrame);

			PRINT_INFO("got color frame\n");
		}

		SafeRelease(pColorFrameReference);
	}

	// Get bodyindex frame
	/*
	if (SUCCEEDED(hr)) {
	IBodyIndexFrameReference* pBodyIndexFrameReference = nullptr;

	hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
	if (SUCCEEDED(hr)) {
	hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);

	#ifdef TEST_
	std::cout << "Kinect Receiver: update: got bodyindex frame" << std::endl;
	#endif
	}

	SafeRelease(pBodyIndexFrameReference);
	}
	*/

	// Succeed in getting the frames
	if (SUCCEEDED(hr)) {
		INT64 nCurRelativeTime;
		IFrameDescription* pDepthFrameDescription = nullptr;
		int nDepthWidth = 0, nDepthHeight = 0;
		unsigned int nDepthBufferSize = 0;
		UINT16* pDepthBuffer = nullptr;

		IFrameDescription* pColorFrameDescription = nullptr;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		int nColorWidth = 0, nColorHeight = 0;
		unsigned int nColorBufferSize = 0;
		RGBQUAD* pColorBuffer = nullptr;

		/*
		IFrameDescription* pBodyIndexFrameDescription = nullptr;
		GLint nBodyIndexWidth = 0, nBodyIndexHeight = 0;
		GLuint nBodyIndexBufferSize = 0;
		BYTE* pBodyIndexBuffer = nullptr;
		*/

		// Get depth data
		{
			if (SUCCEEDED(hr)) {
				hr = pDepthFrame->get_RelativeTime(&nCurRelativeTime);
			}

			if (SUCCEEDED(hr)) {
				hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
			}

			if (SUCCEEDED(hr)) {
				hr = pDepthFrameDescription->get_Width(&nDepthWidth);
			}

			if (SUCCEEDED(hr)) {
				hr = pDepthFrameDescription->get_Height(&nDepthHeight);
			}

			if (SUCCEEDED(hr)) {
				hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);
			}
		}

		// Get color data
		{
			if (SUCCEEDED(hr)) {
				hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
			}

			if (SUCCEEDED(hr)) {
				hr = pColorFrameDescription->get_Width(&nColorWidth);
			}

			if (SUCCEEDED(hr)) {
				hr = pColorFrameDescription->get_Height(&nColorHeight);
			}

			if (SUCCEEDED(hr))
			{
				hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
			}

			if (SUCCEEDED(hr))
			{
				if (imageFormat == ColorImageFormat_Bgra)
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
				}
				else if (m_pColorRGBX)
				{
					pColorBuffer = m_pColorRGBX;
					nColorBufferSize = COLOR_WIDTH * COLOR_HEIGHT * sizeof(RGBQUAD);
					hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
				}
				else
				{
					hr = E_FAIL;
				}
			}
		}

		// Get bodyindex data
		/*
		{
		if (SUCCEEDED(hr)) {
		hr = pBodyIndexFrame->get_FrameDescription(&pBodyIndexFrameDescription);
		}

		if (SUCCEEDED(hr)) {
		hr = pBodyIndexFrameDescription->get_Width(&nBodyIndexWidth);
		}

		if (SUCCEEDED(hr)) {
		hr = pBodyIndexFrameDescription->get_Height(&nBodyIndexHeight);
		}

		if (SUCCEEDED(hr)) {
		hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);
		}
		}
		*/

		// Succeed in getting the data
		// then store them to global vertices
		if (SUCCEEDED(hr)) {
			ProcessFrame(
				nCurRelativeTime,
				(const UINT16*)pDepthBuffer, nDepthWidth, nDepthHeight,
				(const RGBQUAD*)pColorBuffer, nColorWidth, nColorHeight
				//(const BYTE*)pBodyIndexBuffer, nBodyIndexWidth, nBodyIndexHeight
			);
		}

		SafeRelease(pColorFrameDescription);
		SafeRelease(pDepthFrameDescription);
		//==SafeRelease(pBodyIndexFrameDescription);
	}

	//==SafeRelease(pBodyIndexFrame);
	SafeRelease(pColorFrame);
	SafeRelease(pDepthFrame);
	SafeRelease(pMultiSourceFrame);
}

HRESULT KinectReceiver::ProcessFrame(
	const INT64& nCurRelativeTime,
	const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
	const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight
	//==const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight
) {
	if (nCurRelativeTime == m_nCurRelativeTime) {
		return S_OK;
	}

	m_nCurRelativeTime = nCurRelativeTime;

	if (!(pDepthBuffer && (nDepthWidth == DEPTH_WIDTH) && (nDepthHeight == DEPTH_HEIGHT) &&
		pColorBuffer && (nColorWidth == COLOR_WIDTH) && (nColorHeight == COLOR_HEIGHT) //&&
																						   //==pBodyIndexBuffer && (nBodyIndexWidth == DEPTH_WIDTH) && (nBodyIndexHeight == DEPTH_HEIGHT)
		)) {
		return S_OK;
	}

	// We have received valid data, however,
	// to get the camera space point, we have to process coordinate mapping
	HRESULT hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthWidth*nDepthHeight, pDepthBuffer, nDepthWidth*nDepthHeight, m_pDepthFrameToCameraSpace);
	if (SUCCEEDED(hr)) {
		hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(nDepthWidth*nDepthHeight, pDepthBuffer, nColorWidth*nColorHeight, m_pColorFrameToDepthSpace);
	}

	//if (SUCCEEDED(hr)) {
	//	Cloud* pCurCloud = new Cloud;
	//	pCurCloud->points = new CloudPoint[IMG_HEIGHT * IMG_WIDTH];
	//	pCurCloud->size = IMG_HEIGHT * IMG_WIDTH;

	//	for (int colorIndex = 0; colorIndex < nColorHeight*nColorWidth; colorIndex++) {
	//		// Get vertex position
	//		DepthSpacePoint vDepthSpacePoint = m_pColorFrameToDepthSpace[colorIndex];

	//		if (vDepthSpacePoint.X != -std::numeric_limits<float>::infinity() &&
	//			vDepthSpacePoint.Y != -std::numeric_limits<float>::infinity()
	//			) {
	//			int vertexIndex = colorIndex;
	//			int depthX = static_cast<int>(vDepthSpacePoint.X + 0.5f);
	//			int depthY = static_cast<int>(vDepthSpacePoint.Y + 0.5f);

	//			if (depthX >= 0 && depthX < nDepthHeight && depthY >= 0 && depthY < nDepthWidth) {
	//				// Get vertex position
	//				int depthIndex = depthX + depthY * nDepthWidth;
	//				pCurCloud->points[vertexIndex].position.x = m_pDepthFrameToCameraSpace[depthIndex].X;
	//				pCurCloud->points[vertexIndex].position.y = m_pDepthFrameToCameraSpace[depthIndex].Y;
	//				pCurCloud->points[vertexIndex].position.z = -m_pDepthFrameToCameraSpace[depthIndex].Z;

	//				// Get vertex color
	//				pCurCloud->points[vertexIndex].color.r = ((float)pColorBuffer[colorIndex].rgbRed) / 256;
	//				pCurCloud->points[vertexIndex].color.g = ((float)pColorBuffer[colorIndex].rgbGreen) / 256;
	//				pCurCloud->points[vertexIndex].color.b = ((float)pColorBuffer[colorIndex].rgbBlue) / 256;
	//				pCurCloud->points[vertexIndex].color.a = 1.0f;
	//			}
	//		}
	//	}
	//	m_pCloudSet->push_back(pCurCloud);
	//	emit postUpdateEvent();

	//	PRINT_INFO("GET ONE FRAME\n");
	//}

	if (SUCCEEDED(hr)) {
		Cloud* curCloud = new Cloud();
		curCloud->points = new CloudPoint[IMG_HEIGHT * IMG_WIDTH];
		curCloud->size = IMG_HEIGHT * IMG_WIDTH;

		DepthSpacePoint vDepthSpacePoint;
		int pointIndex, depthX, depthY;

		for (int colorIndex = 0; colorIndex < nColorHeight*nColorWidth; colorIndex++) {
			// Get vertex position
			vDepthSpacePoint = m_pColorFrameToDepthSpace[colorIndex];
			pointIndex = colorIndex;

			if (vDepthSpacePoint.X != -std::numeric_limits<float>::infinity() &&
				vDepthSpacePoint.Y != -std::numeric_limits<float>::infinity()
				) {
				depthX = static_cast<int>(vDepthSpacePoint.X + 0.5f);
				depthY = static_cast<int>(vDepthSpacePoint.Y + 0.5f);

				if (depthX >= 0 && depthX < nDepthHeight && depthY >= 0 && depthY < nDepthWidth) {
					// Get vertex position
					int depthIndex = depthX + depthY * nDepthWidth;
					curCloud->points[pointIndex].position.x = m_pDepthFrameToCameraSpace[depthIndex].X;
					curCloud->points[pointIndex].position.y = m_pDepthFrameToCameraSpace[depthIndex].Y;
					curCloud->points[pointIndex].position.z = -m_pDepthFrameToCameraSpace[depthIndex].Z;

					// Get vertex color
					curCloud->points[pointIndex].color.r = ((float)pColorBuffer[colorIndex].rgbRed) / 256;
					curCloud->points[pointIndex].color.g = ((float)pColorBuffer[colorIndex].rgbGreen) / 256;
					curCloud->points[pointIndex].color.b = ((float)pColorBuffer[colorIndex].rgbBlue) / 256;
					curCloud->points[pointIndex].color.a = 1.0f;
				} else {
					curCloud->points[pointIndex].position.x = std::numeric_limits<float>::infinity();
					curCloud->points[pointIndex].position.y = std::numeric_limits<float>::infinity();
					curCloud->points[pointIndex].position.z = std::numeric_limits<float>::infinity();
					curCloud->points[pointIndex].color.r = std::numeric_limits<float>::infinity();
					curCloud->points[pointIndex].color.g = std::numeric_limits<float>::infinity();
					curCloud->points[pointIndex].color.b = std::numeric_limits<float>::infinity();
					curCloud->points[pointIndex].color.a = 1.0f;
				}
			} else {
				curCloud->points[pointIndex].position.x = std::numeric_limits<float>::infinity();
				curCloud->points[pointIndex].position.y = std::numeric_limits<float>::infinity();
				curCloud->points[pointIndex].position.z = std::numeric_limits<float>::infinity();
				curCloud->points[pointIndex].color.r = std::numeric_limits<float>::infinity();
				curCloud->points[pointIndex].color.g = std::numeric_limits<float>::infinity();
				curCloud->points[pointIndex].color.b = std::numeric_limits<float>::infinity();
				curCloud->points[pointIndex].color.a = 1.0f;
			}
		}

		m_pCloudSet->push_back(curCloud);
		emit postUpdateEvent();
	}
	
	return S_OK;
}

void KinectReceiver::updateLater() {
	if(m_updatePending == false) {
		m_updatePending = true;
		QCoreApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
	}
}

void KinectReceiver::updateNow() {
	PRINT_INFO("start\n");
	if (m_pMultiSourceFrameReader == nullptr) {
		return;
	}

	update();

	if (m_animate == true) {
		updateLater();
	}
}

void KinectReceiver::setCloudSet(std::vector<Cloud*>* cloudSet) {
	m_pCloudSet = cloudSet;
}

int KinectReceiver::setAnimate(bool animate) {
	static bool needInitialize = true;
	m_animate = animate;
	if(m_animate) {
		if(m_pCloudSet == nullptr) {
			emit postError(KRHEADER("No memory to contain point cloud\n"));
			m_animate = false;
			return -1;
		}
		if (needInitialize == true) {
			initialize();
			needInitialize = false;
		}
		updateLater();
	}
	return 0;
}

HRESULT KinectReceiver::InitDefaultSensor() {
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr)) {
		return hr;
	}

	if (m_pKinectSensor) {
		if (SUCCEEDED(hr)) {
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		if (SUCCEEDED(hr)) {
			hr = m_pKinectSensor->Open();
		}

		if (SUCCEEDED(hr)) {
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Depth |
				FrameSourceTypes::FrameSourceTypes_BodyIndex,
				&m_pMultiSourceFrameReader
			);
		}
	}

	return hr;
}

inline void KinectReceiver::SafeRelease(IUnknown* p) {
	if(p != nullptr) {
		p->Release();
	}
}

bool KinectReceiver::event(QEvent *event) {
	switch (event->type()) {
	case QEvent::UpdateRequest:
		m_updatePending = false;
		updateNow();
		return true;
	default:
		return QObject::event(event);
	}
}