#include "wrapper.h"
#include "../build/ui_wrapper.h"

Wrapper::Wrapper(QWidget* parent) :
    QMainWindow(parent),
	m_controller(new Controller),
    m_ui(new Ui::Wrapper),
    m_kinectReceiver(new KinectReceiver(&m_cloudSet)) {
    m_ui->setupUi(this);

	// set controller
	m_controller->setCloudSet(&m_cloudSet);
	m_controller->setOpenGLViewer(m_ui->openGLViewer);
	m_controller->setKinectReceiver(m_kinectReceiver);

    // set openglviewer cloud
    m_ui->openGLViewer->setCloud(nullptr);

    // Connect
    connect(m_ui->startButton, SIGNAL(clicked()), this, SLOT(startButtonPressed()));
	connect(m_kinectReceiver, SIGNAL(postUpdateEvent()), m_controller, SLOT(kinectUpdatedOnce()));
	connect(m_ui->openGLViewer, SIGNAL(postUpdateEvent()), m_controller, SLOT(openglUpdatedOnce()));
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

	m_controller->setAnimate(true);
}