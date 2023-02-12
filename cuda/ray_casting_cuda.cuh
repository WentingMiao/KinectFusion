#pragma once

#include <cuda_runtime.h>
#include <vector_types.h>
#include "../Voxels.h"
#include "../Frame.h"
#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>


namespace kinectfusion
{
	using Matrix4fda = Eigen::Matrix<float, 4, 4, Eigen::DontAlign>;
	using Matrix3fda = Eigen::Matrix<float, 3, 3, Eigen::DontAlign>;
	using Vec3fda = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;
	
	std::vector<Vertex>
	ray_casting (
		const VoxelArray &volume,
		const float step_size,  // turnc / 2
		const Matrix3f *Intrinsics,
		const Matrix4f *Extrinsics,
		int width, int height);
};