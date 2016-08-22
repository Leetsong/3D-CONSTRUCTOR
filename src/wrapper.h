#ifndef __WRAPPER_H__
#define __WRAPPER_H__

// Qt
#include <QMainWindow>

// Point Cloud Library
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
// #include <vtkRenderWindow.h>

// controller
#include "controller.h"

// openglwidget
#include "openglviewer.h"

// kinectreceiver
#include "kinectreceiver.h"

// typedef pcl::PointXYZRGBA PointType;
// typedef pcl::PointCloud<PointType> PointCloud;

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

private:
	Controller* m_controller;

	Ui::Wrapper* m_ui;
	std::vector<Cloud*> m_cloudSet;
    KinectReceiver* m_kinectReceiver;

};

#endif // __WRAPPER_H__
