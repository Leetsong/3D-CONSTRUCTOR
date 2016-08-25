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
	">>>>> " + std::string(x) + " STARTED"
#define CONSOLE_END_SUCCEEDED(x) \
	"<<<<< " + std::string(x) + " SUCCEEDED"
#define CONSOLE_END_FAILED(x) \
	"<<<<< " + std::string(x) + " FAILED"

#define WPHEADER(x) \
	"[WRAPPER]: " + std::string(x)

namespace Ui {
	class Wrapper;
}

class Wrapper : public QMainWindow {

Q_OBJECT

public slots:
	void consoleAppendInfo(const std::string& info = "");
	void consoleAppendError(const std::string& error = "");
	void loadButtonPressed();
	void startButtonPressed();
	void stopButtonPressed();
	void resetButtonPressed();
	void registeButtonPressed();

public:
    explicit Wrapper(QWidget* parent = nullptr);
	~Wrapper();
    // We will delete it when kinect is added in

private:
	int loadCloud(const QStringList& filenames);
	void addToCloudSet(PointCloud<PointXYZRGB>::Ptr cloud);
	inline void clearCloudSet();

private:
	bool m_start;
	Controller* m_controller;

	Ui::Wrapper* m_ui;
	std::vector<Cloud*> m_cloudSet;
    KinectReceiver* m_kinectReceiver;
	PCLRegister* m_pclRegister;

};

#endif // __WRAPPER_H__
