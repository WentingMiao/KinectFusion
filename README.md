# Introduction

Kinect Fusion is a technology that allows for the reconstruction of 3D models of real-world objects and scenes using depth data from a Kinect sensor. The original version of Kinect Fusion has been widely used in various applications, including 3D printing and virtual and augmented reality. However, with advancements in computational techniques and algorithms, it is possible to improve upon the original implementation to create a more accurate and efficient version of the technology. This project aims to achieve this goal through the use of updated algorithms and more efficient computation methods, with the ultimate goal of creating a robust and versatile tool for 3D modeling and other applications. As a re-implementation work, we will still implement the various modules according to the original version of Kinect Fusion.

# Workflow

![](https://s2.loli.net/2022/12/28/LdajOpqt6Mm2NxF.png)

# Requirement

**Packages requirement:**

FreeImage, Eigen

**Structure requirement:**

1. Data folder
    
    Create a data folder on the top level and add the data file inside.
    
    e.g Data/rgbd_dataset_freiburg1_xyz/..
    
2. Build directory 
    
    Create the build folder and in this case, we can create an out-of-source build.
    

# Run the project:

After all the packages are prepared and structure requirements are met, we run the project:

```bash
cd build/
cmake ..
make 
./KinectFusion
```