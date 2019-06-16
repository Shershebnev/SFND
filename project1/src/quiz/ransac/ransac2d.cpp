/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <algorithm>
#include <random>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	auto startTime = std::chrono::steady_clock::now();
	
	// TODO: Fill in this function

	// For max iterations 
	float A, B, C, dist, x1, y1, x2, y2;
	int numPoints = cloud->points.size();
	std::unordered_set<int> inliers;
	for (int i = 0; i < maxIterations; i++) {
	// Randomly sample subset and fit line
		while (inliers.size() < 2) {
			inliers.insert(rand() % numPoints);
		}
		auto ptr = std::begin(inliers);
		x1 = cloud->points[*ptr].x;
		y1 = cloud->points[*ptr].y;
		ptr++;
		x2 = cloud->points[*ptr].x;
		y2 = cloud->points[*ptr].y;
		A = y1 - y2;
		B = x2 - x1;
		C = x1 * y2 - x2 * y1;
	// Measure distance between every point and fitted line
		for (int j = 0; j < numPoints; j++) {
			if (inliers.count(j) > 0) {
				continue;
			}
			dist = std::abs(A * cloud->points[j].x + B * cloud->points[j].y + C) / std::sqrt(std::pow(A, 2) + std::pow(B, 2));
			if (dist < distanceTol) {
				inliers.insert(j);
			}
		}
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
		inliers.clear();

	// If distance is smaller than threshold count it as inlier

	}
	// Return indicies of inliers from fitted line with most inliers
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac2D took " << elapsedTime.count() << " milliseconds" << std::endl;
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	auto startTime = std::chrono::steady_clock::now();

	float A, B, C, D, dist, x1, y1, z1, x2, y2, z2, x3, y3, z3;
	int numPoints = cloud->points.size();
	std::unordered_set<int> inliers;
	std::vector<float> v1, v2;
	for (int i = 0; i < maxIterations; i++) {
	// Randomly sample subset and fit line
		while (inliers.size() < 3) {
			inliers.insert(rand() % numPoints);
		}
		auto ptr = std::begin(inliers);
		x1 = cloud->points[*ptr].x;
		y1 = cloud->points[*ptr].y;
		z1 = cloud->points[*ptr].z;
		ptr++;
		x2 = cloud->points[*ptr].x;
		y2 = cloud->points[*ptr].y;
		z2 = cloud->points[*ptr].z;
		ptr++;
		x3 = cloud->points[*ptr].x;
		y3 = cloud->points[*ptr].y;
		z3 = cloud->points[*ptr].z;
		v1 = {x2 - x1, y2 - y1, z2 - z1};
		v2 = {x3 - x1, y3 - y1, z3 - z1};
		A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		D = -(A * x1 + B * y1 + C * z1);

		for (int j = 0; j < numPoints; j++) {
			if (inliers.count(j) > 0) {
				continue;
			}
			dist = std::abs(A * cloud->points[j].x + B * cloud->points[j].y + C * cloud->points[j].z + D)\
					/ std::sqrt(std::pow(A, 2) + std::pow(B, 2) + std::pow(C, 2));
			if (dist < distanceTol) {
				inliers.insert(j);
			}
		}
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
		inliers.clear();
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac3D took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
