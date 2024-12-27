#include "pch.h"
#include "yzsfit.h"
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/common/distances.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "Helper.h"

void fit_circle(const float* array, const int rows, float* diameter)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	Helper::Create2DPointCloudFromArray<pcl::PointXYZ>(array, rows, cloud);

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
	const int max_iterations = 1500;
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
	
	// 找到X最小的点的索引
	int min_x_index = 0;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].x < cloud->points[min_x_index].x)
		{
			min_x_index = i;
		}
	}

	// 计算X最小点及其附近两个点到center的距离平均值
	double total_distance = 0.0;
	int count = 0;

	// 考虑边界情况，确保不会访问越界
	for (int i = std::max(0, min_x_index - 1); 
		 i <= std::min(static_cast<int>(cloud->points.size()) - 1, min_x_index + 1); 
		 i++)
	{
		double current_distance = std::sqrt(
			std::pow(center.x() - cloud->points[i].x, 2) +
			std::pow(center.y() - cloud->points[i].y, 2)
		);
		total_distance += current_distance;
		count++;
	}
	// 计算平均距离
	double distance = total_distance / count;
	*diameter = distance*2;
}