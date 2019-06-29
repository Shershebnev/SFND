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

project2 - Camera based 2D feature tracking  
The goal of the project is to try different combinations of detectors and descriptors on the image data.
I should add some argument parsing, but for now to change the detector:
`project2/src/MidTermProject_Camera_Student.cpp:78`

To change the descriptor:
`project2/src/MidTermProject_Camera_Student.cpp:136`

To change matching algorithm:
`project2/src/MidTermProject_Camera_Student.cpp:151-153`

To build and run:
```cd project2
mkdir build && cd build
cmake ..
make
./2D_feature_tracking
```

project3 - Time-to-collision (TTC) calculation based on lidar and camera data  
The goal of the project is to calculate the TTC using either lidar or camera data.
Camera part is based on project2 for feature detection. Additionally cars are detected using YOLO.
Lidar point cloud is first projected onto 2D image space. Bounding boxes from YOLO are used to decide which points are parts of the cars.

To build and run:
```
cd project3
mkdir build && cd build
cmake ..
make
./3D_object_tracking
```

project4 - Radar target generation and detection  
The goal of this project is to simulate the FMCW radar data with a "target" - a moving car, for instance. This data is then processed using FFT to get Range Doppler Map (RDM). In order to filter out the noise I’ve applied 2D CA-CFAR - 2D Cell Averaging Constant False Alarm Rate.
