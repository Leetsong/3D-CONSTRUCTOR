#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "common.h"
#include "openglviewer.h"
#include "kinectreceiver.h"
#include <QObject>
#include <QEvent>
#include <QCoreApplication>

class Controller : public QObject {
	
Q_OBJECT

public slots:
	void openglUpdatedOnce();
	void kinectUpdatedOnce();

public:
	Controller();
	~Controller();
	void setAnimate(bool animate);
	void setCloudSet(std::vector<Cloud*>* cloudSet);
	void setKinectReceiver(KinectReceiver* kinect);
	void setOpenGLViewer(OpenGLViewer* opengl);

protected:
	bool event(QEvent* event) Q_DECL_OVERRIDE;

private:
	inline void runLater();
	void run();

private:
	bool m_animate;
	size_t m_size;
	size_t m_pointer;

	std::vector<Cloud*>* m_cloudSet;
	KinectReceiver* m_kinect;
	OpenGLViewer* m_opengl;

};

#endif // __CONTROLLER_H__
