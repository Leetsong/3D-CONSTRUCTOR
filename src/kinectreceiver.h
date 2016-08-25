#ifndef __KINECTRECEIVER_H__
#define __KINECTRECEIVER_H__

#include "common.h"
#include <QObject>
#include <QEvent>
#include <QCoreApplication>
#include <kinect.h>

#define KRHEADER(x) \
	 "[KINECT RECEIVER]: " + std::string(x)

class KinectReceiver : public QObject {

Q_OBJECT

signals:
	void postInfo(const std::string& info);
	void postError(const std::string& error);
	void postUpdateEvent();

public:
    KinectReceiver();
    ~KinectReceiver();
    int setAnimate(bool animate);
	void setCloudSet(std::vector<Cloud*>* cloudSet = nullptr);

protected:
	bool event(QEvent * event) Q_DECL_OVERRIDE;

private:
    bool initialize();
	void updateLater();
	void updateNow();
	void update();
	// Initializes the default Kinect sensor
	// S_OK on success, otherwise failure code
	HRESULT InitDefaultSensor();
	HRESULT ProcessFrame(
		const INT64& nCurRelativeTime,
		const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
		const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight
		//const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight
	);
	// Safely release a kinect pointer
	inline void SafeRelease(IUnknown* p);

private:
    std::vector<Cloud*>* m_pCloudSet;
    bool m_animate;
	bool m_updatePending;

	// Kinect Configuration
	static const int DEPTH_WIDTH = 512;
	static const int DEPTH_HEIGHT = 424;
	static const int COLOR_WIDTH = 1920;
	static const int COLOR_HEIGHT = 1080;

	// Current Kinect
	IKinectSensor* m_pKinectSensor;

	// Coodinate mapping
	ICoordinateMapper* m_pCoordinateMapper;
	CameraSpacePoint* m_pDepthFrameToCameraSpace;
	DepthSpacePoint* m_pColorFrameToDepthSpace;

	// Frame Reader
	IMultiSourceFrameReader *m_pMultiSourceFrameReader;
	INT64 m_nCurRelativeTime;

	// Color buffer
	RGBQUAD* m_pColorRGBX;

	// Update thread
	HANDLE m_updateThreadHandler;

};

#endif // __KINECTRECEIVER_H__
