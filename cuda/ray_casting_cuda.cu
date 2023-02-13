#include "ray_casting_cuda.cuh"

#define HANDLE_ERROR(err) (HandleError(err, __FILE__, __LINE__))

static void HandleError(cudaError_t err, const char *file, int line)
{
	if (err != cudaSuccess)
	{
		fprintf(stderr, "Error %d: \"%s\" in %s at line %d\n", int(err), cudaGetErrorString(err), file, line);
		exit(int(err));
	}
}

namespace kinectfusion
{
	__device__
	Vector3f Vec4to3(Vector4f vertex)
	{
		Vector3f output;
		output.x() = vertex.x();
		output.y() = vertex.y();
		output.z() = vertex.z();
		return output;
	}

	__device__
	Vector4f Vec3to4(Vector3f vertex)
	{
		Vector4f output;
		output.x() = vertex.x();
		output.y() = vertex.y();
		output.z() = vertex.z();
		output.w() = 1.0;
		return output;
	}

	__device__ __forceinline__ int grid2idx(const int3 &vol_size, int x, int y, int z)
	{
		return x + y * vol_size.x + z * vol_size.x * vol_size.y;
	}

	__device__ __forceinline__ float interpolate_trilinearly(const Vec3fda &point, const VoxelElement *volume,
															 const int3 &volume_size, const float voxel_scale)
	{
		int3 point_in_grid = make_int3(__float2int_rd(point.x()), __float2int_rd(point.y()), __float2int_rd(point.z()));

		const float vx = (static_cast<float>(point_in_grid.x) + 0.5f);
		const float vy = (static_cast<float>(point_in_grid.y) + 0.5f);
		const float vz = (static_cast<float>(point_in_grid.z) + 0.5f);

		point_in_grid.x = (point.x() < vx) ? (point_in_grid.x - 1) : point_in_grid.x;
		point_in_grid.y = (point.y() < vy) ? (point_in_grid.y - 1) : point_in_grid.y;
		point_in_grid.z = (point.z() < vz) ? (point_in_grid.z - 1) : point_in_grid.z;

		const float a = (point.x() - (static_cast<float>(point_in_grid.x) + 0.5f));
		const float b = (point.y() - (static_cast<float>(point_in_grid.y) + 0.5f));
		const float c = (point.z() - (static_cast<float>(point_in_grid.z) + 0.5f));
		return static_cast<float>(volume[grid2idx(volume_size, point_in_grid.x, point_in_grid.y, point_in_grid.z)].sdf) * (1 - a) * (1 - b) * (1 - c) +
			   static_cast<float>(volume[grid2idx(volume_size, point_in_grid.x, point_in_grid.y, point_in_grid.z + 1)].sdf) * (1 - a) * (1 - b) * c +
			   static_cast<float>(volume[grid2idx(volume_size, point_in_grid.x, point_in_grid.y + 1, point_in_grid.z)].sdf) * (1 - a) * b * (1 - c) +
			   static_cast<float>(volume[grid2idx(volume_size, point_in_grid.x, point_in_grid.y + 1, point_in_grid.z + 1)].sdf) * (1 - a) * b * c +
			   static_cast<float>(volume[grid2idx(volume_size, point_in_grid.x + 1, point_in_grid.y, point_in_grid.z)].sdf) * a * (1 - b) * (1 - c) +
			   static_cast<float>(volume[grid2idx(volume_size, point_in_grid.x + 1, point_in_grid.y, point_in_grid.z + 1)].sdf) * a * (1 - b) * c +
			   static_cast<float>(volume[grid2idx(volume_size, point_in_grid.x + 1, point_in_grid.y + 1, point_in_grid.z)].sdf) * a * b * (1 - c) +
			   static_cast<float>(volume[grid2idx(volume_size, point_in_grid.x + 1, point_in_grid.y + 1, point_in_grid.z + 1)].sdf) * a * b * c;
	}

	__device__ __forceinline__ float get_min_time(const float3 &volume_max, const Vec3fda &origin, const Vec3fda &direction)
	{
		float txmin = ((direction.x() > 0 ? 0.f : volume_max.x) - origin.x()) / direction.x();
		float tymin = ((direction.y() > 0 ? 0.f : volume_max.y) - origin.y()) / direction.y();
		float tzmin = ((direction.z() > 0 ? 0.f : volume_max.z) - origin.z()) / direction.z();
		return fmax(fmax(txmin, tymin), tzmin);
	}

	__device__ __forceinline__ float get_max_time(const float3 &volume_max, const Vec3fda &origin, const Vec3fda &direction)
	{
		float txmax = ((direction.x() > 0 ? volume_max.x : 0.f) - origin.x()) / direction.x();
		float tymax = ((direction.y() > 0 ? volume_max.y : 0.f) - origin.y()) / direction.y();
		float tzmax = ((direction.z() > 0 ? volume_max.z : 0.f) - origin.z()) / direction.z();
		return fmin(fmin(txmax, tymax), tzmax);
	}


	__device__ __forceinline__ float loc2tsdf(const VoxelElement *raw_tsdf, const Vec3fda &location, const float grid_len, const int3 vol_size)
	{
		int3 grid = make_int3(__float2int_rd(location.x() / grid_len), __float2int_rd(location.y() / grid_len), __float2int_rd(location.z() / grid_len));
		return static_cast<float>(raw_tsdf[grid2idx(vol_size, grid.x, grid.y, grid.z)].sdf);
	}

	__device__ __forceinline__
	int3 loc2grid(const Vec3fda &location, const float grid_len)
	{
		return make_int3(__float2int_rd(location.x() / grid_len), __float2int_rd(location.y() / grid_len), __float2int_rd(location.z() / grid_len));
	}

	__device__ __forceinline__ 
	bool isValidGrid(int3 grid, const int3 &volume_size)
	{
		if (grid.x < 1 || grid.x >= volume_size.x - 1 ||
			grid.y < 1 || grid.y >= volume_size.y - 1 ||
			grid.z < 1 || grid.z >= volume_size.z - 1)
			return false;
		return true;
	}
	__device__ __forceinline__
	bool isValidGrid(Vec3fda grid, const int3 &volume_size)
	{
		if (grid.x() < 1 || grid.x() >= volume_size.x - 1 ||
			grid.y() < 1 || grid.y() >= volume_size.y - 1 ||
			grid.z() < 1 || grid.z() >= volume_size.z - 1)
			return false;
		return true;
	}

	__global__ void ray_casting_kernel(
		const VoxelElement *raw_tsdf,
		const float4 *intrinsics, // x, fx, y, fy
		const Matrix4fda *extrinsics,
		const int3 *volume_size,
		const float *grid_len,
		const float *step_size, // turncation distance / 2
		const Vec3fda *origin,
		const int2 *img_size,
		Vertex* ret_vertices)
	{
		// processing pixel at x column, y row
		int x = blockIdx.x * blockDim.x + threadIdx.x;
		int y = blockIdx.y * blockDim.y + threadIdx.y;
		const int &width = img_size->x;
		if (x >= img_size->x || y >= img_size->y)
			return;

		Vector3f Pixel_dir{
			(x - intrinsics->x) / intrinsics->y,
			(y - intrinsics->z) / intrinsics->w,
			1};
		Vec3fda ray_direction = extrinsics->block(0, 0, 3, 3) * Pixel_dir;
		ray_direction.normalize();
		// camera2world: camera + translation
		// world2voxel: world - origin
		// trans: translation - origin
		Vec3fda trans = extrinsics->block(0, 3, 3, 1) - *origin;

		// range in voxel coordinate
		const float3 volume_range = make_float3(volume_size->x * (*grid_len),
												volume_size->y * (*grid_len),
												volume_size->z * (*grid_len));
		float ray_length = fmax(get_min_time(volume_range, trans, ray_direction), 0.f);
		if (ray_length >= get_max_time(volume_range, trans, ray_direction)) {
			#ifdef DEBUG
			printf("ERROR: exit pixel: %d row, %d col, ray len: %f , max time: %f\n", y, x, ray_length, get_max_time(volume_range, trans, ray_direction));
			#endif
			return;
		}

		ray_length += *grid_len;

		Vec3fda ray_loc = trans + (ray_direction * ray_length);
		float tsdf = loc2tsdf(raw_tsdf, ray_loc, *grid_len, *volume_size);
		const float max_search_length = ray_length + volume_range.x * sqrt(3.f);
		// 为啥单独计算max_search_len？

		for (; ray_length < max_search_length; ray_length += *step_size)
		{
			float prev_tsdf = tsdf;
			int3 grid = loc2grid(ray_loc, *grid_len);
			if (!isValidGrid(grid, *volume_size))
				continue;
			ray_loc = trans + (ray_direction * ray_length);
			tsdf = loc2tsdf(raw_tsdf, ray_loc, *grid_len, *volume_size);
			#ifdef DEBUG
			if (x == 500 && y == 240)
				printf("IN-LOOP Processing pixel: %d row, %d col at location: (%f, %f, %f), with sdf = %f\n", y, x, ray_loc.x(), ray_loc.y(), ray_loc.z(), tsdf);
			#endif
			if (prev_tsdf > 0 && tsdf < 0)
			{
				// note that surface_location_world = surface_location_voxel + *origin
				Vec3fda surface_location_voxel = ray_loc - (*step_size) * prev_tsdf / (tsdf - prev_tsdf) * ray_direction;
				Vec3fda surface_normal;

				Vec3fda surface_grid_float = (surface_location_voxel) / (*grid_len);
				Vec3fda shifted = surface_grid_float;
				if (!isValidGrid(surface_grid_float, *volume_size))
					break;
				shifted.x() += 1;
				if (!isValidGrid(shifted, *volume_size))
					break;
				const float Fx1 = interpolate_trilinearly(shifted, raw_tsdf, *volume_size, *grid_len);

				shifted = surface_grid_float;
				shifted.x() -= 1;
				if (shifted.x() < 1)
					break;
				const float Fx2 = interpolate_trilinearly(shifted, raw_tsdf, *volume_size, *grid_len);

				surface_normal.x() = (Fx1 - Fx2);

				shifted = surface_grid_float;
				shifted.y() += 1;
				if (!isValidGrid(shifted, *volume_size))
					break;
				const float Fy1 = interpolate_trilinearly(shifted, raw_tsdf, *volume_size, *grid_len);

				shifted = surface_grid_float;
				shifted.y() -= 1;
				if (!isValidGrid(shifted, *volume_size))
					break;
				const float Fy2 = interpolate_trilinearly(shifted, raw_tsdf, *volume_size, *grid_len);

				surface_normal.y() = (Fy1 - Fy2);

				shifted = surface_grid_float;
				shifted.z() += 1;
				if (!isValidGrid(shifted, *volume_size))
					break;
				const float Fz1 = interpolate_trilinearly(shifted, raw_tsdf, *volume_size, *grid_len);

				shifted = surface_grid_float;
				shifted.z() -= 1;
				if (shifted.z() < 1)
					break;
				const float Fz2 = interpolate_trilinearly(shifted, raw_tsdf, *volume_size, *grid_len);

				surface_normal.z() = (Fz1 - Fz2);

				if (surface_normal.norm() == 0)
					break;

				surface_normal.normalize();
				ret_vertices[y * width + x].position = Vec3to4(surface_location_voxel + *origin);
				ret_vertices[y * width + x].normal = -1 * surface_normal;
				break;
			}
		}
		// return value for invalid point?

	}

	std::vector<Vertex>
	ray_casting(
		const VoxelArray &volume,
		const float step_size,
		const Matrix3f *Intrinsics,
		const Matrix4f *Extrinsics,
		int width, int height)
	{
		// allocate memory for tsdf
		VoxelElement *d_tsdf_data;
		cudaMalloc(&d_tsdf_data, sizeof(VoxelElement) * volume.voxel.size());
		cudaMemcpy(d_tsdf_data, volume.voxel.data(), sizeof(VoxelElement) * volume.voxel.size(), cudaMemcpyHostToDevice);

		int3 voxel_size = make_int3(volume.GetSize()[0], volume.GetSize()[1], volume.GetSize()[2]);
		int3 *d_voxel_size;
		cudaMalloc(&d_voxel_size, sizeof(int3));
		cudaMemcpy(d_voxel_size, &voxel_size, sizeof(int3), cudaMemcpyHostToDevice);

		float grid_len = volume.getGridlen();
		float *d_grid_len;
		cudaMalloc(&d_grid_len, sizeof(float));
		cudaMemcpy(d_grid_len, &grid_len, sizeof(float), cudaMemcpyHostToDevice);

		Vector3f origin = volume.GetOrigin();
		Vec3fda *d_origin;
		cudaMalloc((void **)&d_origin, sizeof(Vec3fda));
		cudaMemcpy(d_origin, &origin, sizeof(Vec3fda), cudaMemcpyHostToDevice);

		float *d_step_size;
		cudaMalloc(&d_step_size, sizeof(float));
		cudaMemcpy(d_step_size, &step_size, sizeof(float), cudaMemcpyHostToDevice);

		// allocate memory for transformation matrix
		float4 intrinsics_val = make_float4(Intrinsics->operator()(0, 2), Intrinsics->operator()(0, 0), Intrinsics->operator()(1, 2), Intrinsics->operator()(1, 1));
		float4 *d_intrinsics;
		cudaMalloc(&d_intrinsics, sizeof(float4));
		cudaMemcpy(d_intrinsics, &intrinsics_val, sizeof(float4), cudaMemcpyHostToDevice);

		Matrix4fda *d_extrinsics;
		cudaMalloc((void **)&d_extrinsics, sizeof(Matrix4fda));
		cudaMemcpy(d_extrinsics, Extrinsics, sizeof(Matrix4fda), cudaMemcpyHostToDevice);

		// allocate memory for return values
		int2 img_size = make_int2(width, height);
		int2 *d_img_size;
		cudaMalloc(&d_img_size, sizeof(int2));
		cudaMemcpy(d_img_size, &img_size, sizeof(int2), cudaMemcpyHostToDevice);

		Vertex *d_ret_vertices;
		// unifier memory
		cudaMallocManaged((void **)&d_ret_vertices, sizeof(Vertex) * width * height);

		dim3 block(32, 32);
		dim3 grid((width + block.x - 1) / block.x,
				  (height + block.y - 1) / block.y);
		kinectfusion::ray_casting_kernel<<<grid, block>>>(d_tsdf_data, d_intrinsics, d_extrinsics, d_voxel_size, d_grid_len, d_step_size, d_origin, d_img_size, d_ret_vertices);
		HANDLE_ERROR(cudaDeviceSynchronize());
		std::vector<Vertex> vertices;
		vertices.insert(vertices.end(), &d_ret_vertices[0], &d_ret_vertices[width * height]);
		cudaFree(d_extrinsics);
		cudaFree(d_grid_len);
		cudaFree(d_img_size);				
		cudaFree(d_intrinsics);
		cudaFree(d_origin);
		cudaFree(d_ret_vertices);
		cudaFree(d_step_size);
		cudaFree(d_tsdf_data);
		cudaFree(d_voxel_size);
		return vertices;
	}
};