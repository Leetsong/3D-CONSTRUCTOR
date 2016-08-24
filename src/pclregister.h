#ifndef __PCLREGISTER_H__
#define __PCLREGISTER_H__

#include "common.h"

#include <QObject>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/pcl_visualizer.h>

 // Visualization Toolkit (VTK)
 #include <vtkRenderWindow.h>

using namespace pcl;

class PCLRegister : public QObject {

	Q_OBJECT

signals:
	void postUpdateEvent();

public slots:
	void runRegistration();
	void setTransformationEpsilon(const QString &text);
	void setMaxCorrespondenceDistance(const QString &text);
	void setTransformationEpsilon(double transformationEpsilon = 1e-6);
	void setMaxCorrespondenceDistance(double maxCorrespondenceDistance = 0.1);
	void setMaximumIterations(int maximumIterations = 60);

public:
	PCLRegister(double transformationEpsilon = 1e-6, double maxCorrespondenceDistance = 0.1, int maximumIterations = 60);
	~PCLRegister();
	vtkRenderWindow* getRenderWindow();
	void setCloudSet(std::vector<Cloud*>* cloudSet);
	void setupInteractor(vtkRenderWindowInteractor *iren, vtkRenderWindow *win);

private:
	void initialize();
	void update(int ptrToProcessingCloud);
	void copyFromCloud(PointCloud<PointXYZRGB>& tgt_cloud, const Cloud* src_cloud);
	inline void copyToCurCloud(int ptrToProcessingCloud);
	inline void firstUpdate();
	// Filtering
	inline void filter();
	void removeNaN();
	void downSample(PointCloud<PointXYZRGB>& cloud);
	// Registration
	void registe();
	// Fusion
	void fuse();
	// Visualization
	void visualize();

private:
	std::vector<Cloud*>* m_cloudSet;
	bool m_isFirst;
	PointCloud<PointXYZRGB> m_globalCloud;
	PointCloud<PointXYZRGB> m_reCloud;
	PointCloud<PointXYZRGB> m_curCloud;
	PointCloud<PointXYZRGB> m_lastCloud;
	IterativeClosestPointNonLinear<PointXYZRGB, PointXYZRGB> m_icpNl;
	double m_transformationEpsilon;
	double m_maxCorrespondenceDistance;
	int m_maximumIterations;
	Eigen::Matrix4f m_globalTransform;
	visualization::PCLVisualizer::Ptr m_viewer;

};

#endif // __PCLREGISTER_H__
