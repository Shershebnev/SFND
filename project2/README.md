# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.


Description of my approaches to student tasks:
MP.1
Data buffer was implemented using std::deque which allows insertion or removal of elements at the end of beginning in O(1), whereas for vector removal of the first element is O(N). At each iteration the size of deque is checked and if it exceeds the threshold, the first element is removed. I should note that even though data buffer here is basically a FIFO structure, I’ve refrained from using std::queue since it doesn’t provide iteration and this functionality was used in other places of the code. Well, std::queue uses std::deque anyway, so… :)

MP.2
Implemented using basic if..else if..else functionality where condition is the result of using std::string::compare method.

MP.3
std::vector is not the best container here as well for the same reason as in MP.1 - erasing random element is O(N), whereas std::list, for instance, provide this functionality in O(1). However I’ve already done the pretty time consuming parts on getting different statistics into the spreadsheet (MP.7-MP.9), so I decided not to optimise this. Even though this shouldn’t affect any of the values.
Also since I was erasing elements in the process of iterating over the container, I’ve started iterating from the end of the vector to ensure that “upcoming” elements won’t change their indexes after erasing.

MP.4
Basically the same as MP.2

MP.5
Same as MP.2 and MP.4

MP.6
I’m iterating over the found neighbours and checking the ration of best vs second-best match.

MP.7 - MP.9
The numbers are in xlsx file. 

Based on the performance, the best combination is FAST as detector and either BRISK, BRIEF or ORB as descriptor extractors.
