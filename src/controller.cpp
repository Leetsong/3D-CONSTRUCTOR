#include "controller.h"

Controller::Controller() :
	m_animate(false),
	m_cloudSet(nullptr),
	m_kinect(nullptr),
	m_opengl(nullptr),
	m_size(0),
	m_pointer(0) {
	m_pcl._register = nullptr;
	m_pcl._viewer = nullptr;
}

Controller::~Controller() {
}

int Controller::setAnimate(bool animate) {
	m_animate = animate;
	if(m_animate == true) {
		if(m_kinect->setAnimate(true) == 0) {
			runLater();
		} else {
			m_animate = false;
			m_kinect->setAnimate(false);
			m_opengl->setAnimate(false);
			return -1;
		}
	} else {
		m_kinect->setAnimate(false);
		m_opengl->setAnimate(false);
	}
	return 0;
}

void Controller::setCloudSet(std::vector<Cloud*>* cloudSet) {
	m_cloudSet = cloudSet;
	m_kinect->setCloudSet(m_cloudSet);
	m_opengl->setCloud(nullptr);
	m_pcl._register->setCloudSet(m_cloudSet);
}

void Controller::setKinectReceiver(KinectReceiver * kinect) {
	m_kinect = kinect;
}

void Controller::setOpenGLViewer(OpenGLViewer * opengl) {
	m_opengl = opengl;
}

void Controller::setPCL(PCLRegister* pclregister, QVTKWidget* pclviewer) {
	m_pcl._register = pclregister;
	m_pcl._viewer = pclviewer;

	// set qvtkwindow(pclviewer)
	m_pcl._viewer->SetRenderWindow(m_pcl._register->getRenderWindow());
	m_pcl._register->setupInteractor(m_pcl._viewer->GetInteractor(), m_pcl._viewer->GetRenderWindow());
	m_pcl._viewer->update();
	
	emit postInfo(CTHEADER("qvtkwindow successfully setup"));
}

inline void Controller::runLater() {
	QCoreApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
}

void Controller::run() {
	PRINT_INFO("pointer: %zu, size: %zu\n", m_pointer, m_size);

	size_t pointer = m_pointer, size = m_size;
	if(pointer < size) {
		m_opengl->setAnimate(true, (*m_cloudSet)[pointer]);
	}

	if(m_animate == true) {
		runLater();
	}
}

int Controller::runRegistration() {
	return m_pcl._register->runRegistration();
}

bool Controller::event(QEvent* event) {
	switch(event->type()) {
	case QEvent::UpdateRequest:
		run();
		return true;
	default:
		return QObject::event(event);
	}
}

void Controller::kinectUpdatedOnce() {
	m_size ++;

	PRINT_INFO("updated size: to %zu\n", m_size);
}

void Controller::openglUpdatedOnce() {
	m_pointer ++;

	PRINT_INFO("updated pointer: to %zu\n", m_pointer);
}

void Controller::pclUpdatedOnce() {
	m_pcl._viewer->update();
}