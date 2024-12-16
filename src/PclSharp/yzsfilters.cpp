#include "pch.h"
#include "yzsfilters.h"
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "Helper.h"


float* radius_down_sampling_run(const float* arrayF1, int rows, size_t* resultRow)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	Helper::Create3DPointCloudFromArray<pcl::PointXYZ>(arrayF1, rows, cloud);
	YzsFilters<pcl::PointXYZ> rd;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = rd.radiusSearch_filter(cloud);
	float* floatArray = Helper::PointCloudToArray(out_cloud);
	*resultRow = out_cloud->points.size();
	return floatArray;
}

float* statistical_outlier_filter(const float* array, const int rows, const int meanK, const float stddevMulThresh, size_t* resultRow)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	Helper::Create3DPointCloudFromArray<pcl::PointXYZ>(array, rows, cloud);
	YzsFilters<pcl::PointXYZ> rd;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = rd.statistical_filter(cloud, meanK, stddevMulThresh);
	float* floatArray = Helper::PointCloudToArray(out_cloud);
	*resultRow =out_cloud->points.size();
	return floatArray;
}              

float* smooth_filter(const float* array, const int rows,const int polynomialOrder, const float searchRadius, size_t* resultRow)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	Helper::Create3DPointCloudFromArray<pcl::PointXYZ>(array, rows, cloud);
	YzsFilters<pcl::PointXYZ> rd;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = rd.smooth_filter(cloud, polynomialOrder, searchRadius);
	float* floatArray = Helper::PointCloudToArray(out_cloud);
	*resultRow = out_cloud->points.size();
	return floatArray;
}
