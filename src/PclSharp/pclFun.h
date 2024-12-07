#pragma once

#include "pch.h"
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

template<typename PointT>
class YzsFilters {
public:
    YzsFilters() = default;

    typename pcl::PointCloud<PointT>::Ptr filter(
        const typename pcl::PointCloud<PointT>::Ptr& input_cloud) 
    {
        // 创建输出点云
        typename pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);
        
        // 如果输入点云为空则返回
        if (input_cloud->empty()) {
            return output_cloud;
        }
    	pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
		ror.setInputCloud(input_cloud);
	    ror.setRadiusSearch(0.1);   // 设置搜索半径
	    ror.setMinNeighborsInRadius(8);   // 设置最小近邻点数
	    ror.filter(*output_cloud);
        return output_cloud;
    }

    
    typename pcl::PointCloud<PointT>::Ptr statistical_filter(
        const typename pcl::PointCloud<PointT>::Ptr& input_cloud,int meanK, float stddevMulThresh) 
    {
        typename pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);
        if (input_cloud->empty()) {
            return output_cloud;
        }
    	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> ror;
		ror.setInputCloud(input_cloud);
	    ror.setMeanK(meanK);   // 设置搜索半径
	    ror.setStddevMulThresh(stddevMulThresh);   // 设置最小近邻点数
	    ror.filter(*output_cloud);
        return output_cloud;
    }
    
    
};

extern "C" EXPORT(float*)  RadiusDownSamplingRun(const float* array, int rows,int* resultRow);

extern "C" EXPORT(float*)  StatisticalOutlierFilter(const float* array, int rows,int meanK, float stddevMulThresh,int* resultRow);

extern "C" inline __declspec(dllexport) void FreeArray(const float* array) {
	delete[] array;
}
