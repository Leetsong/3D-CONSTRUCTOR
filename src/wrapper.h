#ifndef __WRAPPER_H__
#define __WRAPPER_H__

// Qt
#include <QMainWindow>
#include <QFileDialog>
#include <QStringList>

// controller
#include "controller.h"

// openglwidget
#include "openglviewer.h"

// kinectreceiver
#include "kinectreceiver.h"

// pclregister
#include "pclregister.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#define CONSOLE_START(x) \
	">>>>> " + std::string(x) + " START"
#define CONSOLE_END(x) \
	"<<<<< " + std::string(x) + " DONE"

namespace Ui {
	class Wrapper;
}

class Wrapper : public QMainWindow {

Q_OBJECT

public:
    explicit Wrapper(QWidget* parent = nullptr);
	~Wrapper();
    // We will delete it when kinect is added in

private:
	inline void consoleAppendInfo(const std::string& info);
	inline void consoleAppendError(const std::string& error);
	void loadCloud(const QStringList& filenames);
	void addToCloudSet(PointCloud<PointXYZRGB>::Ptr cloud);

public slots:
	void loadButtonPressed();
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
