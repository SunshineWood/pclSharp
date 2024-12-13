#pragma once


//
// Created by HoldenSun on 2024/12/13.
//

#ifndef HELPER_H
#define HELPER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Helper {
    template <typename PointT>
    static void Create3DPointCloudFromArray(const float* array, int rows,typename pcl::PointCloud<PointT>::Ptr& cloud) {
        cloud.reset(new pcl::PointCloud<PointT>());
        cloud->resize(rows);
        for (int i = 0; i < rows; ++i) {
            cloud->points[i].x = array[i * 3 + 0];
            cloud->points[i].y = array[i * 3 + 1];
            cloud->points[i].z = array[i * 3 + 2];
        }
    }

	template <typename PointT>
    static void Create2DPointCloudFromArray(const float* array, int rows,typename pcl::PointCloud<PointT>::Ptr& cloud) {
        cloud.reset(new pcl::PointCloud<PointT>());
        cloud->resize(rows);
        for (int i = 0; i < rows; ++i) {
            cloud->points[i].x = array[i * 2 + 0];
            cloud->points[i].y = array[i * 2 + 1];
            cloud->points[i].z = 0;
        }
    }

    static float* PointCloudToArray(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
        float* array = new float[cloud->points.size() * 3];
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            array[i * 3 + 0] = cloud->points[i].x;
            array[i * 3 + 1] = cloud->points[i].y;
            array[i * 3 + 2] = cloud->points[i].z;
        }
        return array;
    }
}

#endif //HELPER_H
