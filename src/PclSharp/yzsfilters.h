#pragma once

#ifndef YZSFILTER_H
#define YZSFILTER_H

#include "pch.h"
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>


extern "C" EXPORT(float*) radius_down_sampling_run(const float* arrayF1, int rows, size_t* resultRow);

extern "C" EXPORT(float*)  statistical_outlier_filter(const float* array, int rows,int meanK, float stddevMulThresh,size_t* resultRow);

extern "C" EXPORT(float*)  smooth_filter(const float* array, int rows,int polynomialOrder,float searchRadius,size_t* resultRow);

extern "C" inline __declspec(dllexport) void FreeArray(const float* array) {
	delete[] array;
}


template<typename PointT>
class YzsFilters {
public:
    YzsFilters() = default;

    typename pcl::PointCloud<PointT>::Ptr radiusSearch_filter(
        const typename pcl::PointCloud<PointT>::Ptr& input_cloud) 
    {
        typename pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);
        if (input_cloud->empty()) {
            return output_cloud;
        }
    	pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
		ror.setInputCloud(input_cloud);
	    ror.setRadiusSearch(0.1); 
	    ror.setMinNeighborsInRadius(8); 
	    ror.filter(*output_cloud);
        return output_cloud;
    }

    typename pcl::PointCloud<PointT>::Ptr statistical_filter(
        const typename pcl::PointCloud<PointT>::Ptr& input_cloud, const int meanK, const float stddevMulThresh)
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

    typename pcl::PointCloud<PointT>::Ptr smooth_filter(
    const typename pcl::PointCloud<PointT>::Ptr& input_cloud, const int polynomialOrder, const float searchRadius)
    {
        typename pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);
        if (input_cloud->empty()) {
            return output_cloud;
        }
        const pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        // 设置MLS参数
        mls.setComputeNormals(false); 
        mls.setPolynomialOrder(polynomialOrder); 
        mls.setSearchRadius(searchRadius); 
        mls.setInputCloud(input_cloud);
        mls.setSearchMethod(tree);
        mls.process(*output_cloud);
        return output_cloud;
    }

};

#endif
