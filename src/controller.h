#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "common.h"
#include "openglviewer.h"
#include "kinectreceiver.h"
#include "pclregister.h"
#include <QObject>
#include <QEvent>
#include <QCoreApplication>
#include <QVTKWidget.h>

#define CTHEADER(x) \
	"[CONTROLLER]: " + std::string(x)

class Controller : public QObject {
	
Q_OBJECT

signals:
	void postInfo(const std::string& info);
	void postError(const std::string& error);

public slots:
	void openglUpdatedOnce();
	void kinectUpdatedOnce();
	void pclUpdatedOnce();

public:
	Controller();
	~Controller();
	int setAnimate(bool animate);
	void setCloudSet(std::vector<Cloud*>* cloudSet);
	void setKinectReceiver(KinectReceiver* kinect);
	void setOpenGLViewer(OpenGLViewer* opengl);
	void setPCL(PCLRegister* pclregister, QVTKWidget* pclviewer);
	int runRegistration();

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
	struct {
		PCLRegister* _register;
		QVTKWidget* _viewer;
	} m_pcl;
	
};

#endif // __CONTROLLER_H__
