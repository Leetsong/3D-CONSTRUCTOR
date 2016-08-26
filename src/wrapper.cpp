#include "wrapper.h"
#include "../build/ui_wrapper.h"

Wrapper::Wrapper(QWidget* parent) :
    QMainWindow(parent),
	m_start(false),
	m_controller(new Controller),
    m_ui(new Ui::Wrapper),
    m_kinectReceiver(new KinectReceiver()),
	m_pclRegister(new PCLRegister()) {

    m_ui->setupUi(this);
	this->setWindowTitle("3D-CONSTRUCTOR");
	consoleAppendInfo("============ 3D-CONSTRUCTOR SETUP STARTS ============");
	consoleAppendInfo(WPHEADER("UI successfully setup"));

	// set controller
	m_controller->setOpenGLViewer(m_ui->openGLViewer);
	m_controller->setKinectReceiver(m_kinectReceiver);
	m_controller->setPCL(m_pclRegister, m_ui->pclViewer);
	m_controller->setCloudSet(&m_cloudSet);
	consoleAppendInfo(WPHEADER("controller successfully setup"));

	// Connect
	connect(m_ui->openGLViewer, SIGNAL(postInfo(const std::string&)), this, SLOT(consoleAppendInfo(const std::string&)));
	connect(m_ui->openGLViewer, SIGNAL(postError(const std::string&)), this, SLOT(consoleAppendError(const std::string&)));
	connect(m_kinectReceiver, SIGNAL(postInfo(const std::string&)), this, SLOT(consoleAppendInfo(const std::string&)));
	connect(m_kinectReceiver, SIGNAL(postError(const std::string&)), this, SLOT(consoleAppendError(const std::string&)));
	connect(m_pclRegister, SIGNAL(postInfo(const std::string&)), this, SLOT(consoleAppendInfo(const std::string&)));
	connect(m_pclRegister, SIGNAL(postError(const std::string&)), this, SLOT(consoleAppendError(const std::string&)));
    connect(m_ui->startButton, SIGNAL(clicked()), this, SLOT(startButtonPressed()));
	connect(m_ui->stopButton, SIGNAL(clicked()), this, SLOT(stopButtonPressed()));
	connect(m_ui->registeButton, SIGNAL(clicked()), this, SLOT(registeButtonPressed()));
	connect(m_kinectReceiver, SIGNAL(postUpdateEvent()), m_controller, SLOT(kinectUpdatedOnce()));
	connect(m_ui->openGLViewer, SIGNAL(postUpdateEvent()), m_controller, SLOT(openglUpdatedOnce()));
	connect(m_pclRegister, SIGNAL(postUpdateEvent()), m_controller, SLOT(pclUpdatedOnce()));
	connect(m_ui->loadAction, SIGNAL(triggered()), this, SLOT(loadActionTriggered()));
	connect(m_ui->resetAction, SIGNAL(triggered()), this, SLOT(resetActionTriggered()));
	connect(m_ui->iterationsAction, SIGNAL(triggered()), this, SLOT(iterationsActionTriggered()));
	connect(m_ui->distanceAction, SIGNAL(triggered()), this, SLOT(distanceActionTriggered()));
	connect(m_ui->epsilonAction, SIGNAL(triggered()), this, SLOT(epsilonActionTriggered()));
	consoleAppendInfo(WPHEADER("connections successfully setup"));

	consoleAppendInfo("============ 3D-CONSTRUCTOR SETUP DONE ============\n");
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

int Wrapper::loadCloud(const QStringList& filenames) {
	std::string ex_pcd = ".pcd", ex_ply = ".ply";
	for (size_t i = 0; i < filenames.size(); i++) {
		std::stringstream info, error_info;
		std::string filename(filenames.at(static_cast<int>(i)).toLocal8Bit().constData());
		PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());

		if (filename.substr(filename.find('.')) == ex_pcd) {
			if (io::loadPCDFile(filename, *cloud) < 0) {
				error_info << "Cannot load file[" << i << "]: "<< filename;
				consoleAppendError(error_info.str());
				return -1;
			}
			addToCloudSet(cloud);
			info << "  Loading file[" << i << "]: " << filename << " done:" << std::endl;
			info << "    [Height: " << cloud->height << " Width: " << cloud->width << " Dense: " << cloud->is_dense << "]" << std::endl;
			consoleAppendInfo(info.str());
		} else if (filename.substr(filename.find('.')) == ex_ply) {
			if (io::loadPLYFile(filename, *cloud) < 0) {
				error_info << "Cannot load file[" << i << "]: " << filename;
				consoleAppendError(error_info.str());
				return -1;
			}
			addToCloudSet(cloud);
			info << "  Loading file[" << i << "]: " << filename << " done:" << std::endl;
			info << "    [Height: " << cloud->height << " Width: " << cloud->width << " Dense: " << cloud->is_dense << "]" << std::endl;
			consoleAppendInfo(info.str());
		} else {
			error_info << "Cannot open file[" << i << "]: " << filename << ": file is invalid";
			consoleAppendError(error_info.str());
			return -2;
		}
	}

	return 0;
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

inline void Wrapper::clearCloudSet() {
	for(size_t i = 0; i < m_cloudSet.size(); i++) {
		if(m_cloudSet[i] != nullptr) {
			delete m_cloudSet[i];
		}
	}
	m_cloudSet.clear();
	m_cloudSet.swap(std::vector<Cloud*>());
}

void Wrapper::consoleAppendInfo(const std::string& info) {
	if(m_ui->consoleTabs->currentIndex() != m_ui->consoleTabs->indexOf(m_ui->outputTab)) {
		m_ui->consoleTabs->setCurrentIndex(m_ui->consoleTabs->indexOf(m_ui->outputTab));
	}
	if(info != "") {
		m_ui->outputTextBrowser->append(QString(info.c_str()));
		m_ui->outputTextBrowser->moveCursor(QTextCursor::End);
	}
}

void Wrapper::consoleAppendError(const std::string& error) {
	if(m_ui->consoleTabs->currentIndex() != m_ui->consoleTabs->indexOf(m_ui->errorTab)) {
		m_ui->consoleTabs->setCurrentIndex(m_ui->consoleTabs->indexOf(m_ui->errorTab));
	}
	if(error != "") {
		m_ui->errorTextBrowser->append(QString(error.c_str()));
		m_ui->errorTextBrowser->moveCursor(QTextCursor::End);
		m_controller->setAnimate(false);
		clearCloudSet();
		m_controller->setCloudSet(&m_cloudSet);
	}
}

void Wrapper::startButtonPressed() {
	PRINT_INFO("start button pressed\n");
	consoleAppendInfo(CONSOLE_START("Start"));
	if(m_start == true) return;
	m_start = true;
	if(m_controller->setAnimate(m_start) != 0) {
		consoleAppendInfo(CONSOLE_END_FAILED("Start"));
		consoleAppendError("");
	}
}

void Wrapper::stopButtonPressed() {
	PRINT_INFO("stop button pressed\n");
	consoleAppendInfo(CONSOLE_START("Stop"));
	if (m_start == false) return;
	m_start = false;
	m_controller->setAnimate(m_start);
}

void Wrapper::registeButtonPressed() {
	PRINT_INFO("registe button pressed\n");
	consoleAppendInfo(CONSOLE_START("Regist"));
	if(m_controller->runRegistration() == 0) {
		consoleAppendInfo(CONSOLE_END_SUCCEEDED("Registe"));
	} else {
		consoleAppendInfo(CONSOLE_END_FAILED("Registe"));
		consoleAppendError("");
	}
}

void Wrapper::loadActionTriggered() {
	QStringList filenames = QFileDialog::getOpenFileNames(this,
		"Select one or more files to open",
		"/home",
		"pointclouds(*.pcd *.ply)"
	);
	consoleAppendInfo(CONSOLE_START("Load files"));
	if(loadCloud(filenames) == 0) {
		consoleAppendInfo(CONSOLE_END_SUCCEEDED("Load files"));
	} else {
		consoleAppendInfo(CONSOLE_END_FAILED("Load files"));
		consoleAppendError("");
	}
}

void Wrapper::resetActionTriggered() {
	stopButtonPressed();
	consoleAppendInfo(CONSOLE_START("Reset"));
	clearCloudSet();
	m_controller->setCloudSet(&m_cloudSet);
}

void Wrapper::iterationsActionTriggered() {
	bool ok;  
	int iterations = QInputDialog::getInt(this, "Setting", "iterations: ", 60, 1, 2147483647, 1, &ok);
	consoleAppendInfo(CONSOLE_START("Set ICP Max Iterations"));
	if(ok)  {
		m_pclRegister->setMaximumIterations(iterations);

		std::stringstream info;
		info << "Set max iterations to " << iterations;
		consoleAppendInfo(CONSOLE_END_SUCCEEDED(info.str()));
	} else {
		consoleAppendInfo(CONSOLE_END_CANCELED("Set ICP Max Iterations"));
	}
}

void Wrapper::distanceActionTriggered() {
	bool ok;  
	int distance = QInputDialog::getDouble(this,"Setting", "distance: ", 0.1f, 0, std::numeric_limits<double>::infinity(), 3, &ok);
	consoleAppendInfo(CONSOLE_START("Set ICP Max Correspondence Distance"));
	if(ok)  {
		m_pclRegister->setMaxCorrespondenceDistance(distance);

		std::stringstream info;
		info << "Set max correspondence distance to " << distance;
		consoleAppendInfo(CONSOLE_END_SUCCEEDED(info.str()));
	} else {
		consoleAppendInfo(CONSOLE_END_CANCELED("Set ICP Max Correspondence Distance"));
	}
}

void Wrapper::epsilonActionTriggered() {
	bool ok;  
	int epsilon = QInputDialog::getDouble(this,"Setting", "epsilon: ", 0.000001f, 0, 1, 15, &ok);
	consoleAppendInfo(CONSOLE_START("Set ICP Transformation Epsilon"));
	if(ok)  {
		m_pclRegister->setTransformationEpsilon(epsilon);

		std::stringstream info;
		info << "Set transformation epsilon to " << epsilon;
		consoleAppendInfo(CONSOLE_END_SUCCEEDED(info.str()));
	} else {
		consoleAppendInfo(CONSOLE_END_CANCELED("Set ICP Transformation Epsilon"));
	}
}