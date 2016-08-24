#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "common.h"
#include "openglviewer.h"
#include "kinectreceiver.h"
#include <QObject>
#include <QEvent>
#include <QCoreApplication>
#include <QVTKWidget.h>

class Controller : public QObject {
	
Q_OBJECT

public slots:
	void openglUpdatedOnce();
	void kinectUpdatedOnce();
	void pclUpdatedOnce();

public:
	Controller();
	~Controller();
	void setAnimate(bool animate);
	void setCloudSet(std::vector<Cloud*>* cloudSet);
	void setKinectReceiver(KinectReceiver* kinect);
	void setOpenGLViewer(OpenGLViewer* opengl);
	void setPCLViewer(QVTKWidget* pcl);

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
	QVTKWidget* m_pcl;

};

#endif // __CONTROLLER_H__
