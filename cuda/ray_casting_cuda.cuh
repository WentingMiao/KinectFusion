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

	// ray casting for 1 pixel
	__global__ void ray_casting_kernel(
		const VoxelElement *raw_tsdf,
		const float4 *intrinsics, // x, fx, y, fy
		const Matrix4fda *extrinsics,
		const int3 *volume_size,
		const float *grid_len,
		const float *step_size,
		const Vec3fda *origin,
		const int2 *img_size,
		Vector3f *ret_positions,
		Vector3f *ret_normals);
	
	// std::tuple<std::vector<Vector3f>, std::vector<Vector3f>> 
	void ray_casting (
		const VoxelArray &volume,
		const float step_size,  // turnc / 2
		const Matrix3f *Intrinsics,
		const Matrix4f *Extrinsics,
		int width, int height);
};