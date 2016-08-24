#include "pclregister.h"

PCLRegister::PCLRegister(
	double transformationEpsilon,
	double maxCorrespondenceDistance,
	int maximumIterations
) : m_cloudSet(nullptr),
	m_isFirst(true),
	m_transformationEpsilon(transformationEpsilon),
	m_maxCorrespondenceDistance(maxCorrespondenceDistance),
	m_maximumIterations(maximumIterations) {
	m_viewer.reset(new visualization::PCLVisualizer("PCL Viewer", false));
	m_viewer->setBackgroundColor(0, 0, 0, 0);
}

PCLRegister::~PCLRegister() {

}

void PCLRegister::setCloudSet(std::vector<Cloud*>* cloudSet) {
	m_cloudSet = cloudSet;
}

void PCLRegister::run() {
	initialize();

	int ptrToProcessingCloud = 0;
	int ptrToNewestCloud = -1;

	while (true) {
		ptrToNewestCloud = static_cast<int>(m_cloudSet->size()) - 1;
		if (ptrToProcessingCloud < ptrToNewestCloud) {
			update(ptrToProcessingCloud);
			ptrToProcessingCloud++;
		}
	}
}

vtkRenderWindow* PCLRegister::PCLRegister::getRenderWindow() {
	return m_viewer->getRenderWindow();
}

void PCLRegister::setupInteractor(vtkRenderWindowInteractor *iren, vtkRenderWindow *win) {
	m_viewer->setupInteractor(iren, win);
}

void PCLRegister::setTransformationEpsilon(double transformationEpsilon) {
	m_transformationEpsilon = transformationEpsilon;
}

void PCLRegister::setMaxCorrespondenceDistance(double maxCorrespondenceDistance) {
	m_maxCorrespondenceDistance = maxCorrespondenceDistance;
}

void PCLRegister::setMaximumIterations(int maximumIterations) {
	m_maximumIterations = maximumIterations;
}

void PCLRegister::copyFromCloud(PointCloud<PointXYZRGB>& tgt_cloud, const Cloud* src_cloud) {
	size_t size = src_cloud->size;

	tgt_cloud.clear();
	tgt_cloud.is_dense = false;
	tgt_cloud.width = static_cast<uint32_t>(size);
	tgt_cloud.height = 1;
	tgt_cloud.resize(tgt_cloud.width * tgt_cloud.height);

	for (size_t i = 0; i < size; i++) {
		tgt_cloud.points[i].x = src_cloud->points[i].position.x;
		tgt_cloud.points[i].y = src_cloud->points[i].position.y;
		tgt_cloud.points[i].z = src_cloud->points[i].position.z;
		tgt_cloud.points[i].r = src_cloud->points[i].color.r;
		tgt_cloud.points[i].g = src_cloud->points[i].color.g;
		tgt_cloud.points[i].b = src_cloud->points[i].color.b;
	}

}

inline void PCLRegister::copyToCurCloud(int ptrToProcessingCloud) {
	// Save current cloud
	copyPointCloud(m_curCloud, m_lastCloud);
	copyFromCloud(m_curCloud, (*m_cloudSet)[ptrToProcessingCloud]);
}

inline void PCLRegister::filter() {
	removeNaN();
	downSample(m_curCloud);
}

void PCLRegister::removeNaN() {
	static int i = -1;
	std::vector<int> indices;

	i++;
	std::cout << "[" << i << "]: Before Remove NaN: " << m_curCloud.size() << std::endl;
	removeNaNFromPointCloud(m_curCloud, m_curCloud, indices);
	std::cout << "[" << i << "]: After Remove NaN: " << m_curCloud.size() << std::endl;
}

void PCLRegister::downSample(PointCloud<PointXYZRGB>& srcCloud) {
	static int i = -1;
	static VoxelGrid<PointXYZ> grid;
	PointCloud<PointXYZ>::Ptr tgtCloud(new PointCloud<PointXYZ>);

	i++;
	std::cout << "[" << i << "]: Before Down Sample: " << srcCloud.size() << std::endl;
	copyPointCloud(srcCloud, *tgtCloud);
	grid.setLeafSize(0.05f, 0.05f, 0.05f);
	grid.setInputCloud(tgtCloud);
	grid.filter(*tgtCloud);
	copyPointCloud(*tgtCloud, srcCloud);
	std::cout << "[" << i << "]: After Down Sample: " << srcCloud.size() << std::endl;
}

inline void PCLRegister::firstUpdate() {
	copyFromCloud(m_curCloud, (*m_cloudSet)[0]);
	removeNaN();
	downSample(m_curCloud);
	copyPointCloud(m_curCloud, m_globalCloud);
}

void PCLRegister::initialize() {
	m_globalTransform = Eigen::Matrix4f::Identity();
	m_icpNl.setTransformationEpsilon(m_transformationEpsilon);
	m_icpNl.setMaxCorrespondenceDistance(m_maxCorrespondenceDistance);
	m_icpNl.setMaximumIterations(m_maximumIterations);

}

void PCLRegister::registerNow() {
	m_reCloud.clear();

	// Align this to last, and save it in resultant
	m_icpNl.setInputSource(boost::make_shared<const PointCloud<PointXYZRGB>>(m_curCloud));
	m_icpNl.setInputTarget(boost::make_shared<const PointCloud<PointXYZRGB>>(m_lastCloud));
	m_icpNl.align(m_reCloud);

	// Transform to first cloud
	transformPointCloud(m_reCloud, m_reCloud, m_globalTransform);
	m_globalTransform = m_globalTransform * m_icpNl.getLastIncrementalTransformation();
}

void PCLRegister::fuse() {
	m_globalCloud += m_reCloud;
	// TODO: Further Processing for fusion
	// downSample to make it as small as possible
	downSample(m_globalCloud);
}

void PCLRegister::update(int ptrToProcessingCloud) {
	// Save and update last and current cloud
	if (m_isFirst) {
		firstUpdate();
		m_isFirst = false;
		visualize();
		return;
	}

	// Process next cloud, registe it to the first cloud
	copyToCurCloud(ptrToProcessingCloud);

	// filtering
	filter();
	// Registration
	registerNow();
	// Visualization
	visualize();
	// Fusion
	fuse();
}

void PCLRegister::visualize() {
	m_viewer->removePointCloud("RESULT");
	m_viewer->removePointCloud("GLOBAL");

	visualization::PointCloudColorHandlerCustom<PointXYZRGB> globalCloudColorHandler(boost::make_shared<const PointCloud<PointXYZRGB>>(m_globalCloud), 255, 0, 0);
	m_viewer->addPointCloud(boost::make_shared<const PointCloud<PointXYZRGB>>(m_globalCloud), globalCloudColorHandler, "GLOBAL");
	std::cout << "Global Cloud Points: " << m_globalCloud.size() << std::endl;

	if (m_reCloud.size() > 0) {
		visualization::PointCloudColorHandlerCustom<PointXYZRGB> reCloudColorHandler(boost::make_shared<const PointCloud<PointXYZRGB>>(m_reCloud), 0, 255, 0);
		m_viewer->addPointCloud(boost::make_shared<const PointCloud<PointXYZRGB>>(m_reCloud), reCloudColorHandler, "RESULT");
	}

	m_viewer->spin();
}