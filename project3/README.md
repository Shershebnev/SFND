# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

FP.1  
I’ve decided to store the counts in a map where keys are indexes of the bounding boxes in the previous frame and values are maps where keys are indexes of bounding boxes on the current frame and values are counts. Then for each cv::DMatch object I try to find which bounding boxes each of the ends of the match belong to and update respective count if both ends were matched with the boxes. After that for each bounding box from the previous frame the box from the current frame with the highest count is selected.

FP.2  
I’ve decided to use median value of the distances to mitigate the problem with potential outliers instead of using the closest point. See also FP.5

FP.3  
I’ve first calculated the mean and standard deviation of the distribution of distances between matched points. After that I’ve filtered out all points that are more than one sigma away from the mean.

FP.4  
I’ve used median value for distance ratios here as well to mitigate the outliers problem.

FP.5  
The general trend for TTC seems to be aligned with the distance to the car with both of them getting smaller with time. For frames 3-4 and 4-5, however, there exists a reverse trend where the distance to the car keeps decreasing but the TTC gets bigger. After visual examination I’ve noticed that the enclosed lidar points have slightly different shape for those frames - they appear to be placed in a wider formation compared to frames before and after where the clusters seem to be more tight. This could affected the median value of distance to the point cloud. Given that for TTC we are mostly interested in the closest point but need to avoid outliers one possible solution could be to first filter point cloud using mead and standard deviation for example (as in FP.3) and then calculate the distance to the closest point and use that for TTC calculations.

FP.6  
Please see the table and the plot in the spreadsheet. It looks like there is significant error for SIFT detector with any possible descriptor. One thing I’ve noticed is that if I filter key points and keep only those that are in the region of interest for the cat in front, just like in the midterm project, than the TTC prediction using SIFT gets back to normal. I guess the error could be caused by false matches from other vehicles which are further on the road. After noticing this I’ve decided to add another table and plot, this time with key points from region of interest only. This gave more reasonable plot with overall trend on decreasing TTC towards the last frames.  
Also ORB detector occasionally	produced `-inf` values for some reason. And Harris detector produced an error in OpenCV code.
