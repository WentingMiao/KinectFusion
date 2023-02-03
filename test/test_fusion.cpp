#include <iostream>
#include <fstream>
#include <array>
#include <vector>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <math.h>
#include <string>
#include "VirtualSensor.h"
#include "SurfaceReconstruction.hpp"
#include "Eigen.h"
#include "Frame.h"
#include "Voxels.h"


void SaveVoxelGrid2SurfacePointCloud(const string &file_name, std::array<unsigned, 3> volume_size,
	float voxel_size, float origin_x, float origin_y, float origin_z,
	float *voxel_grid_TSDF, float *voxel_grid_weight, float tsdf_thresh, float weight_thres)
{
	// count
	int num_pts = 0;
	for (int i = 0; i < volume_size[0] * volume_size[1] * volume_size[2]; i++)
		if (abs(voxel_grid_TSDF[i]) <= tsdf_thresh && voxel_grid_weight[i] >= weight_thres)
			num_pts++;
	cout << num_pts << " Points in tsdf.ply" << endl;
	FILE *fp = fopen(file_name.c_str(), "w");
	fprintf(fp, "ply\n");
	fprintf(fp, "format binary_little_endian 1.0\n");
	fprintf(fp, "element vertex %d\n", num_pts);
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uint8 red\n");
	fprintf(fp, "property uint8 green\n");
	fprintf(fp, "property uint8 blue\n");
	fprintf(fp, "end_header\n");

	for (int i = 0; i < volume_size[0] * volume_size[1] * volume_size[2]; i++)
	{
		if (abs(voxel_grid_TSDF[i]) <= tsdf_thresh && voxel_grid_weight[i] >= weight_thres)
		{
			int z = floor(i / (volume_size[0] * volume_size[1]));
			int y = floor((i - (z * volume_size[0] * volume_size[1])) / volume_size[0]);
			int x = i - (z * volume_size[0] * volume_size[1]) - (y * volume_size[0]);

			// voxel indices to float
			float pt_base_x = origin_x + (float)x * voxel_size;
			float pt_base_y = origin_y + (float)y * voxel_size;
			float pt_base_z = origin_z + (float)z * voxel_size;

			uint8_t color = 96;
			fwrite(&pt_base_x, sizeof(float), 1, fp);
			fwrite(&pt_base_y, sizeof(float), 1, fp);
			fwrite(&pt_base_z, sizeof(float), 1, fp);
			fwrite(&color, sizeof(uint8_t), 1, fp);
			fwrite(&color, sizeof(uint8_t), 1, fp);
			fwrite(&color, sizeof(uint8_t), 1, fp);
		}
	}
	fclose(fp);
}



int main() {

	//int res;
	//path to the data 
	std::string filenameIn = "E:/KinectFusion-volume/KinectFusion-volume/Data/rgbd_dataset_freiburg1_xyz/";

	//path to the output 
	std::string filenameBaseOut = "E:/KinectFusion-volume/KinectFusion-volume/Data/rgbd_dataset_freiburg1_xyz/";
	std::cout << "Initialize virtual sensor..." << std::endl;

	VirtualSensor sensor;


	if (!sensor.Init(filenameIn)) {
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}


	float origin_x = -1.5f; // location of voxel grid origin in base frame camera coor
	float origin_y = -1.5f;
	float origin_z = 0.5f;

	std::array<unsigned, 3> volumesize{ 500, 500, 500 };
	float voxel_size = 0.006f;
	Matrix4f depthExtrinsics = sensor.GetDepthExtrinsics();
	Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics(); //内参
	Matrix4f trajectory = sensor.GetTrajectory(); //外参

	BYTE* colorMap = &sensor.GetColorRGBX()[0];
	float* depthMap = &sensor.GetDepth()[0];

	unsigned int width = sensor.GetDepthImageWidth();
	unsigned int height = sensor.GetDepthImageHeight();
	float edgeThreshold = 10;
	double truncationDistance = 0;
	bool filtered = true;

	//cout << <<std::endl;

	Eigen::Matrix4f cur_pose = Matrix4f::Identity();
	std::shared_ptr<Frame> currentFrame = std::make_shared<Frame>(Frame(depthMap, colorMap, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold, filtered));
	//Frame currentFrame(depthMap, colorMap, depthIntrinsics, depthExtrinsics, trajectory, width, height, edgeThreshold, filtered);

	auto volume = std::make_shared<VoxelArray>(
		std::array<unsigned, 3>{600, 600, 600}, 
		0.05, Vector3f{ -3, -3, 0 }, Matrix4f{});

	Fusion fusion;
	fusion.SurfaceReconstruction(currentFrame, volume, cur_pose, truncationDistance);

	//float voxel_TSDF = volume->GetSDFVal();
	//float voxel_weight = volume->GetWeightVal();

	//build ply file for visualizing
	//SaveVoxelGrid2SurfacePointCloud("tsdf.ply", volumesize,
	//	voxel_size, origin_x,origin_y, origin_z,
	//	voxel_TSDF, voxel_weight, 0.25, 2.0f); // 后两超参： 到隐势面的距离阈值（-1,1), 看到voxel的次数; 即(-0.4,0.4),3次

	//res = execute();
	//return res;
}