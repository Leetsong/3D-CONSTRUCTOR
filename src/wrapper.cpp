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

	PRINT_INFO("UI successfully setup\n");

	// set controller
	m_controller->setCloudSet(&m_cloudSet);
	m_controller->setOpenGLViewer(m_ui->openGLViewer);
	m_controller->setKinectReceiver(m_kinectReceiver);

	PRINT_INFO("controller successfully setup\n");

    // set openglviewer cloud
    m_ui->openGLViewer->setCloud(nullptr);
	m_pclRegister->setCloudSet(&m_cloudSet);

	PRINT_INFO("cloud set for opengl and pcl successfully setup\n");

	// set qvtkwindow(pclviewer)
	m_ui->pclViewer->SetRenderWindow(m_pclRegister->getRenderWindow());
	m_pclRegister->setupInteractor(m_ui->pclViewer->GetInteractor(), m_ui->pclViewer->GetRenderWindow());
	m_ui->pclViewer->update();

	PRINT_INFO("qvtkwindow successfully setup\n");

	// Connect
    connect(m_ui->startButton, SIGNAL(clicked()), this, SLOT(startButtonPressed()));
	connect(m_ui->stopButton, SIGNAL(clicked()), this, SLOT(stopButtonPressed()));
	connect(m_ui->resetButton, SIGNAL(clicked()), this, SLOT(resetButtonPressed()));
	connect(m_ui->registerNowButton, SIGNAL(clicked()), m_pclRegister, SLOT(registerNow()));
	connect(m_ui->maximumIterationsSpinBox, SIGNAL(valueChanged(int)), m_pclRegister, SLOT(setMaximumIterations(int)));
	connect(m_ui->maxCorrespondenceDistanceDoubleSpinBox, SIGNAL(valueChanged(int)), m_pclRegister, SLOT(setMaxCorrespondenceDistance(int)));
	connect(m_ui->transformationEpsilonDoubleSpinBox, SIGNAL(valueChanged(int)), m_pclRegister, SLOT(setTransformationEpsilon(int)));
	connect(m_kinectReceiver, SIGNAL(postUpdateEvent()), m_controller, SLOT(kinectUpdatedOnce()));
	connect(m_ui->openGLViewer, SIGNAL(postUpdateEvent()), m_controller, SLOT(openglUpdatedOnce()));

	PRINT_INFO("connections successfully setup\n");
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

void Wrapper::loadCloud() {
    std::vector<float> data;

    std::ifstream fStream;
    fStream.open("../../src/test.data", std::ifstream::in);

    while(!fStream.eof()) {
        float d;
        fStream >> d;
        data.push_back(d);
    }

    if(data.empty() == true) {
        std::cout << "Error loading" << std::endl;
        exit(-1);
    } else {
        std::cout << "Points: " << data.size() / 3 << std::endl;
    }

    Cloud* cloud = new Cloud;
    cloud->size = data.size() / 3;
    cloud->points = new CloudPoint[cloud->size];

    size_t i = 0;
    for(size_t j = 0; j < cloud->size; j ++) {
        cloud->points[j].position.x = data[i];
        cloud->points[j].position.y = data[i+1];
        cloud->points[j].position.z = data[i+2];
        cloud->points[j].color.r = 255;
        cloud->points[j].color.g = 255;
        cloud->points[j].color.b = 255;
        cloud->points[j].color.a = 1;
        i += 3;
    }

    m_cloudSet.push_back(cloud);

    m_ui->openGLViewer->setCloud(m_cloudSet.back());
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