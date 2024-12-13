#pragma once

#ifndef YZSFIT_H
#define YZSFIT_H

extern "C" EXPORT(void)  fit_circle(const float* array, int rows,float* diameter);


struct CircleFitResult
{
	double diameter;
	double center_x;
	double center_y;
	double average_error;
	int inlier_count;
};

#endif