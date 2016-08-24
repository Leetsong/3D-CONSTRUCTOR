#include "wrapper.h"
#include "../build/ui_wrapper.h"

Wrapper::Wrapper(QWidget* parent) :
    QMainWindow(parent),
	m_start(false),
	m_controller(new Controller),
    m_ui(new Ui::Wrapper),
    m_kinectReceiver(new KinectReceiver(&m_cloudSet)),
	m_pclRegister(new PCLRegister()) {

    m_ui->setupUi(this);
	this->setWindowTitle("3D-CONSTRUCTOR");
	consoleAppend("============3D-CONSTRUCTOR SETUP STARTS============");
	consoleAppend("UI successfully setup");

	// set controller
	m_controller->setCloudSet(&m_cloudSet);
	m_controller->setOpenGLViewer(m_ui->openGLViewer);
	m_controller->setKinectReceiver(m_kinectReceiver);
	consoleAppend("controller successfully setup");

    // set openglviewer cloud
    m_ui->openGLViewer->setCloud(nullptr);
	m_pclRegister->setCloudSet(&m_cloudSet);
	consoleAppend("cloud set for opengl and pcl successfully setup");

	// set qvtkwindow(pclviewer)
	m_ui->pclViewer->SetRenderWindow(m_pclRegister->getRenderWindow());
	m_pclRegister->setupInteractor(m_ui->pclViewer->GetInteractor(), m_ui->pclViewer->GetRenderWindow());
	m_ui->pclViewer->update();
	consoleAppend("qvtkwindow successfully setup");

	// Connect
	connect(m_ui->loadButton, SIGNAL(clicked()), this, SLOT(loadButtonPressed()));
    connect(m_ui->startButton, SIGNAL(clicked()), this, SLOT(startButtonPressed()));
	connect(m_ui->stopButton, SIGNAL(clicked()), this, SLOT(stopButtonPressed()));
	connect(m_ui->resetButton, SIGNAL(clicked()), this, SLOT(resetButtonPressed()));
	connect(m_ui->registerNowButton, SIGNAL(clicked()), m_pclRegister, SLOT(runRegistration()));
	connect(m_ui->maximumIterationsSpinBox, SIGNAL(valueChanged(int)), m_pclRegister, SLOT(setMaximumIterations(int)));
	connect(m_ui->maxCorrespondenceDistanceDoubleSpinBox, SIGNAL(valueChanged(const QString&)), m_pclRegister, SLOT(setMaxCorrespondenceDistance(const QString&)));
	connect(m_ui->transformationEpsilonDoubleSpinBox, SIGNAL(valueChanged(const QString&)), m_pclRegister, SLOT(setTransformationEpsilon(const QString&)));
	connect(m_kinectReceiver, SIGNAL(postUpdateEvent()), m_controller, SLOT(kinectUpdatedOnce()));
	connect(m_ui->openGLViewer, SIGNAL(postUpdateEvent()), m_controller, SLOT(openglUpdatedOnce()));
	consoleAppend("connections successfully setup");

	m_ui->maximumIterationsSpinBox->setValue(60);
	m_ui->transformationEpsilonDoubleSpinBox->setValue(1e-6);
	m_ui->maxCorrespondenceDistanceDoubleSpinBox->setValue(0.1);

	consoleAppend("============3D-CONSTRUCTOR SETUP DONE============\n");
}

Wrapper::~Wrapper() {
	for(size_t i = 0; i < m_cloudSet.size(); i++) {
		if(m_cloudSet[i] != nullptr) {
			delete m_cloudSet[i];
			m_cloudSet[i] = nullptr;
		}
	}
	if (m_kinectReceiver != nullptr) {
		delete m_kinectReceiver;
	}
}

void Wrapper::loadCloud(const QStringList& filenames) {
	std::string ex_pcd = ".pcd", ex_ply = ".ply";
	for (size_t i = 0; i < filenames.size(); i++) {
		std::string filename(filenames.at(i).toLocal8Bit().constData());
		PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());

		if (filename.substr(filename.find('.')) == ex_pcd) {
			if (io::loadPCDFile(filename, *cloud) < 0) {
				PCL_ERROR("Error loading file[%d]: %s\n", i, filename.c_str());
				return;
			}
			addToCloudSet(cloud);
			PCL_INFO("Loading over file[%d]: %s....\n", i, filename.c_str());
			std::cout << "    Height: " << cloud->height << " Width: " << cloud->width << " Dense: " << cloud->is_dense << std::endl;
		} else if (filename.substr(filename.find('.')) == ex_ply) {
			if (io::loadPLYFile(filename, *cloud) < 0) {
				PCL_ERROR("Error loading file[%d]: %s\n", i, filename.c_str());
				return;
			}
			addToCloudSet(cloud);
			PCL_INFO("Loading over file[%d]: %s....\n", i, filename.c_str());
			std::cout << "    Height: " << cloud->height << " Width: " << cloud->width << " Dense: " << cloud->is_dense << std::endl;
		} else {
			PCL_ERROR("Error file[%d] %s is not valid\n", i, filename.c_str());
			return;
		}
	}
}

void Wrapper::addToCloudSet(PointCloud<PointXYZRGB>::Ptr pointcloud) {
	Cloud* cloud = new Cloud();
	cloud->size = pointcloud->points.size();
	cloud->points = new CloudPoint[cloud->size];

	for (size_t j = 0; j < cloud->size; j++) {
		cloud->points[j].position.x = pointcloud->points[j].x;
		cloud->points[j].position.y = pointcloud->points[j].y;
		cloud->points[j].position.z = pointcloud->points[j].z;
		cloud->points[j].color.r = pointcloud->points[j].r;
		cloud->points[j].color.g = pointcloud->points[j].g;
		cloud->points[j].color.b = pointcloud->points[j].b;
	}

	m_cloudSet.push_back(cloud);

	PCL_INFO("Copying over file: cloud size: %lu\n", cloud->size);
}

inline void Wrapper::consoleAppend(const std::string& info) {
	m_ui->consoleTextBrowser->append(QString(info.c_str()));
	m_ui->consoleTextBrowser->moveCursor(QTextCursor::End);
}

void Wrapper::loadButtonPressed() {
	QStringList filenames = QFileDialog::getOpenFileNames(this,
		"Select one or more files to open",
		"/home",
		"pointclouds(*.pcd *.ply)"
	);
	consoleAppend(CONSOLE_START("Load files"));
	loadCloud(filenames);
	consoleAppend(CONSOLE_END("Load files"));
}

void Wrapper::startButtonPressed() {
	PRINT_INFO("start button pressed\n");
	if(m_start == true) return;
	m_controller->setAnimate(true);
}

void Wrapper::stopButtonPressed() {
	PRINT_INFO("stop button pressed\n");
	if (m_start == false) return;
	m_controller->setAnimate(false);
}

void Wrapper::resetButtonPressed() {
	PRINT_INFO("start button pressed\n");
	stopButtonPressed();
	m_cloudSet.clear();
}