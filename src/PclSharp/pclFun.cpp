#include "pch.h"
#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include "pclFun.h"
#include <pcl/common/distances.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>

float* RadiusDownSamplingRun(const float* arrayF1, int rows, int* resultRow)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(rows);
	for (int i = 0; i < rows; ++i)
	{
		cloud->points[i].x = arrayF1[i * 3 + 0];
		cloud->points[i].y = arrayF1[i * 3 + 1];
		cloud->points[i].z = arrayF1[i * 3 + 2];
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

float* StatisticalOutlierFilter(const float* array, int rows, int meanK, float stddevMulThresh, int* resultRow)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(rows);
	for (int i = 0; i < rows; ++i)
	{
		cloud->points[i].x = array[i * 3 + 0];
		cloud->points[i].y = array[i * 3 + 1];
		cloud->points[i].z = array[i * 3 + 2];
	}
	YzsFilters<pcl::PointXYZ> rd;
	const pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = rd.statistical_filter(cloud, meanK, stddevMulThresh);
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

struct CircleFitResult
{
	double diameter;
	double center_x;
	double center_y;
	double average_error;
	int inlier_count;
};

void FitCircle(const float* array, int rows, float* outputDiameter)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(rows);
	for (int i = 0; i < rows; ++i)
	{
		cloud->points[i].x = array[i * 2 + 0];
		cloud->points[i].y = array[i * 2 + 1];
		cloud->points[i].z = 0;
	}

	CircleFitResult result;
	result.diameter = 9.480;

	// 初始化质心作为初始圆心估计
	Eigen::Vector2d centroid(0, 0);
	for (const auto& point : cloud->points)
	{
		centroid.x() += point.x;
		centroid.y() += point.y;
	}
	centroid /= cloud->points.size();

	// 迭代优化圆心位置
	const int max_iterations = 1000;
	const double convergence_threshold = 1e-4;

	Eigen::Vector2d center = centroid;
	Eigen::Vector2d prev_center;

	for (int iter = 0; iter < max_iterations; iter++)
	{
		prev_center = center;
		Eigen::Vector2d sum_direction(0, 0);
		double total_error = 0;

		for (const auto& point : cloud->points)
		{
			Eigen::Vector2d pt(point.x, point.y);
			Eigen::Vector2d direction = pt - center;
			double current_radius = direction.norm();
			direction.normalize();

			// 计算点到理想圆的径向偏差
			double radial_error = current_radius - result.diameter / 2;
			sum_direction += direction * radial_error;
			total_error += std::abs(radial_error);
		}

		// 更新圆心
		center += sum_direction / cloud->points.size();

		// 检查收敛
		if ((center - prev_center).norm() < convergence_threshold)
		{
			break;
		}
	}

	// 计算最终结果
	result.center_x = center.x();
	result.center_y = center.y();

	// 计算拟合误差
	double total_error = 0;
	int inlier_count = 0;
	const double inlier_threshold = 1e-4; // 1mm

	for (const auto& point : cloud->points)
	{
		double dx = point.x - center.x();
		double dy = point.y - center.y();
		double current_radius = std::sqrt(dx * dx + dy * dy);
		double error = std::abs(current_radius - result.diameter / 2);
		total_error += error;

		if (error < inlier_threshold)
		{
			inlier_count++;
		}
	}

	result.average_error = total_error / cloud->points.size();
	result.inlier_count = inlier_count;
	pcl::PointXYZ min_x_point = cloud->points[0];
	for (const auto& point : cloud->points)
	{
		if (point.x < min_x_point.x)
		{
			min_x_point = point;
		}
	}

	// Calculate the distance between center and the point with minimum X
	double distance_to_min_x = std::sqrt(
		std::pow(center.x() - min_x_point.x, 2) +
		std::pow(center.y() - min_x_point.y, 2)
	);
	*outputDiameter = distance_to_min_x*2;
}
