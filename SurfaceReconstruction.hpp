#pragma once
#include "Frame.h"
#include <memory>
#include "Voxels.h"


namespace Fusion {
    /*
        * @description : Loop over every world point and calculate a truncated signed distance value (TSDF), along with a weight
        * @param cur_Frame: current frame
        * @param volume: The container which save the data
        * @param cur_pose: The camera pose of shape(4x4)
        * @param truncationDistance:
        * 
    */
	bool SurfaceReconstruction(Frame& cur_Frame, VoxelArray& volume, float truncationDistance);
};


