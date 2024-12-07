#include "pch.h"
#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include "pclFun.h"
#include <pcl/common/distances.h>
#include <pcl/registration/icp.h>


float* RadiusDownSamplingRun(const float* arrayF1, int rows,int* resultRow)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(rows);
	for (int i = 0; i < rows; ++i)
	{
		cloud->points[i].x = arrayF1[i*3+0];
		cloud->points[i].y = arrayF1[i*3+1];
		cloud->points[i].z = arrayF1[i*3+2];
	}
    YzsFilters<pcl::PointXYZ> rd;
    const pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = rd.filter(cloud);
	float* floatArray = new float[out_cloud->points.size() * 3];
	for (size_t i = 0; i < out_cloud->points.size(); ++i) 
	{
		floatArray[i * 3 + 0] = out_cloud->points[i].x;
		floatArray[i * 3 + 1] = out_cloud->points[i].y;
		floatArray[i * 3 + 2] = out_cloud->points[i].z;
	}
	*resultRow = out_cloud->points.size();
	return floatArray;
}

float* StatisticalOutlierFilter(const float* array, int rows,int meanK, float stddevMulThresh,int* resultRow)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(rows);
	for (int i = 0; i < rows; ++i)
	{
		cloud->points[i].x = array[i*3+0];
		cloud->points[i].y = array[i*3+1];
		cloud->points[i].z = array[i*3+2];
	}
	YzsFilters<pcl::PointXYZ> rd;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = rd.statistical_filter(cloud,meanK,stddevMulThresh);
	float* floatArray = new float[out_cloud->points.size() * 3];
	for (size_t i = 0; i < out_cloud->points.size(); ++i) 
	{
		floatArray[i * 3 + 0] = out_cloud->points[i].x;
		floatArray[i * 3 + 1] = out_cloud->points[i].y;
		floatArray[i * 3 + 2] = out_cloud->points[i].z;
	}
	*resultRow = out_cloud->points.size();
	return floatArray;
}

