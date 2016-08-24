#ifndef __WRAPPER_H__
#define __WRAPPER_H__

// Qt
#include <QMainWindow>

// controller
#include "controller.h"

// openglwidget
#include "openglviewer.h"

// kinectreceiver
#include "kinectreceiver.h"

// pclregister
#include "pclregister.h"

namespace Ui {
	class Wrapper;
}

class Wrapper : public QMainWindow {

Q_OBJECT

public:
    explicit Wrapper(QWidget* parent = nullptr);
	~Wrapper();
    // We will delete it when kinect is added in
    void loadCloud();

public slots:
    void startButtonPressed();
	void stopButtonPressed();
	void resetButtonPressed();

private:
	bool m_start;
	Controller* m_controller;

	Ui::Wrapper* m_ui;
	std::vector<Cloud*> m_cloudSet;
    KinectReceiver* m_kinectReceiver;
	PCLRegister* m_pclRegister;

};

#endif // __WRAPPER_H__
