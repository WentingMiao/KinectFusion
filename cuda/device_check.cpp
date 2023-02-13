#include <iostream>
#include <cuda_runtime.h>

int main() {
  int dev = 0;
    cudaDeviceProp devProp;
    cudaGetDeviceProperties(&devProp, dev);
    std::cout << "使用GPU device " << dev << ": " << devProp.name << std::endl;
    std::cout << "SM的数量：" << devProp.multiProcessorCount << std::endl;
    std::cout << "每个线程块的共享内存大小：" << devProp.sharedMemPerBlock / 1024.0 << " KB" << std::endl;
    std::cout << "每个线程块的最大线程数：" << devProp.maxThreadsPerBlock << std::endl;
    std::cout << "每个EM的最大线程数：" << devProp.maxThreadsPerMultiProcessor << std::endl;
    std::cout << "每个SM的最大线程束数：" << devProp.maxThreadsPerMultiProcessor / 32 << std::endl;

}

// 使用GPU device 0: NVIDIA GeForce RTX 3060 Laptop GPU
// SM的数量：30
// 每个线程块的共享内存大小：48 KB
// 每个线程块的最大线程数：1024
// 每个EM的最大线程数：2359296
// 每个SM的最大线程束数：73728
