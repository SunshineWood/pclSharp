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
	*diameter = distance_to_min_x*2;
}