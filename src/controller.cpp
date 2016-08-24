#include "controller.h"

Controller::Controller() :
	m_animate(false),
	m_size(0),
	m_pointer(0) {

}

Controller::~Controller() {
}

void Controller::setAnimate(bool animate) {
	m_animate = animate;
	if(m_animate == true) {
		m_kinect->setAnimate(true);
		runLater();
	} else {
		m_kinect->setAnimate(false);
		m_opengl->setAnimate(false);
	}
}

void Controller::setCloudSet(std::vector<Cloud*>* cloudSet) {
	m_cloudSet = cloudSet;
}

void Controller::setKinectReceiver(KinectReceiver * kinect) {
	m_kinect = kinect;
}

void Controller::setOpenGLViewer(OpenGLViewer * opengl) {
	m_opengl = opengl;
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