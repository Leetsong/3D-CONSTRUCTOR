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
	m_ptrToProcessingCloud = 0;

	// clear cloud
	m_currentCloud.clear();
	m_currentCloud.points.swap(std::vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB>>());
	m_currentCloud.resize(0);
	m_resultantCloud.clear();
	m_resultantCloud.points.swap(std::vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB>>());
	m_resultantCloud.resize(0);
	m_globalCloud.clear();
	m_globalCloud.points.swap(std::vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB>>());
	m_globalCloud.resize(0);

	visualize();
}

int PCLRegister::runRegistration() {
	if((m_cloudSet == nullptr) || (m_cloudSet->size() == 0)) {
		emit postError(PRHEADER("No input point clouds."));
		return -1;
	}

	initialize();
	if(m_ptrToProcessingCloud < static_cast<int>(m_cloudSet->size())) {
		update(m_ptrToProcessingCloud);
		m_ptrToProcessingCloud++;
	}

	return 0;
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
	copyPointCloud(m_currentCloud, m_previousCloud);
	copyFromCloud(m_currentCloud, (*m_cloudSet)[ptrToProcessingCloud]);
}

inline void PCLRegister::filter() {
	removeNaN();
	downSample(m_currentCloud);
}

void PCLRegister::removeNaN() {
	static int i = -1;
	std::vector<int> indices;

	i++;
	std::cout << "[" << i << "]: Before Remove NaN: " << m_currentCloud.size() << std::endl;
	removeNaNFromPointCloud(m_currentCloud, m_currentCloud, indices);
	std::cout << "[" << i << "]: After Remove NaN: " << m_currentCloud.size() << std::endl;
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
	copyFromCloud(m_currentCloud, (*m_cloudSet)[0]);
	removeNaN();
	downSample(m_currentCloud);
	copyPointCloud(m_currentCloud, m_globalCloud);
}

void PCLRegister::initialize() {
	m_globalTransform = Eigen::Matrix4f::Identity();
	m_icpNl.setTransformationEpsilon(m_transformationEpsilon);
	m_icpNl.setMaxCorrespondenceDistance(m_maxCorrespondenceDistance);
	m_icpNl.setMaximumIterations(m_maximumIterations);

}

void PCLRegister::registe() {
	m_resultantCloud.clear();

	// Align this to last, and save it in resultant
	m_icpNl.setInputSource(boost::make_shared<const PointCloud<PointXYZRGB>>(m_currentCloud));
	m_icpNl.setInputTarget(boost::make_shared<const PointCloud<PointXYZRGB>>(m_previousCloud));
	m_icpNl.align(m_resultantCloud);

	// Transform to first cloud
	transformPointCloud(m_resultantCloud, m_resultantCloud, m_globalTransform);
	m_globalTransform = m_globalTransform * m_icpNl.getLastIncrementalTransformation();
}

void PCLRegister::fuse() {
	m_globalCloud += m_resultantCloud;
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
	registe();
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

	if (m_resultantCloud.size() > 0) {
		visualization::PointCloudColorHandlerCustom<PointXYZRGB> reCloudColorHandler(boost::make_shared<const PointCloud<PointXYZRGB>>(m_resultantCloud), 0, 255, 0);
		m_viewer->addPointCloud(boost::make_shared<const PointCloud<PointXYZRGB>>(m_resultantCloud), reCloudColorHandler, "RESULT");
	}

	emit postUpdateEvent();
}