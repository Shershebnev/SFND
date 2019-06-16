# SFND
Repository for the projects from Sensor Fusion NanoDegree

project1 - Lidar obstacle detection
The goal of the project is to detect obstacles on the point cloud data. This is done in two steps:
1. Finding the road plane using RANSAC (Random Sample Consensus) algorithm;
2. Creating clusters on the segmented cloud using KdTree and Euclidean Clustering;

RANSAC, KdTree and Euclidean Clustering were first used from the C++ PCL library and later reimplemented by myself.
To build and run:
```cd project1
mkdir build && cd build
cmake ..
make
./environment
```
