// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int pointIdx, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol) {
	processed[pointIdx] = true;
	cluster.push_back(pointIdx);
	for (int idx : tree->search(points[pointIdx], distanceTol)) {
		if (!processed[idx]) {
			proximity(idx, points, cluster, processed, tree, distanceTol);
		}
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed (points.size(), false);
	for (int i = 0; i < points.size(); i++) {
		if (!processed[i]) {
			std::vector<int> cluster;
			proximity(i, points, cluster, processed, tree, distanceTol);
			clusters.push_back(cluster);
		}
	}
	return clusters;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr gridCloud (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> grid;
    grid.setInputCloud(cloud);
    grid.setLeafSize(filterRes, filterRes, filterRes);
    grid.filter(*gridCloud);

    typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> crop(true);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.setInputCloud(gridCloud);
    crop.filter(*filteredCloud);

    // remove points from roof
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(filteredCloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int point : indices) {
    	inliers->indices.push_back(point);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filteredCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT> ());
	typename pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT> ());
	for (int index : inliers->indices) {
		plane->push_back(cloud->points[index]);
	}

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
//	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.
//	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//	pcl::SACSegmentation<PointT> seg;
//
//	seg.setOptimizeCoefficients(true);
//	seg.setModelType(pcl::SACMODEL_PLANE);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setMaxIterations(maxIterations);
//	seg.setDistanceThreshold(distanceThreshold);
//
//	seg.setInputCloud(cloud);
//	seg.segment(*inliers, *coefficients);
//	if (inliers->indices.size() == 0) {
//		std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
//	}

	// using my own implementation of Ransac
    std::unordered_set<int> inliersResult;
	srand(time(NULL));

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
			if (dist < distanceThreshold) {
				inliers.insert(j);
			}
		}
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
		inliers.clear();
	}

	if (inliersResult.size() == 0) {
			std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
	}

	typename pcl::PointIndices::Ptr cloudInliers (new pcl::PointIndices());
	for (int point : inliersResult) {
		cloudInliers->indices.push_back(point);
	}
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(cloudInliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

//    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
//    std::vector<pcl::PointIndices> clusterIndices;
//    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
//    tree->setInputCloud(cloud);
//
//    pcl::EuclideanClusterExtraction<PointT> ec;
//    ec.setClusterTolerance(clusterTolerance);
//    ec.setMinClusterSize(minSize);
//    ec.setMaxClusterSize(maxSize);
//    ec.setSearchMethod(tree);
//    ec.setInputCloud(cloud);
//    ec.extract(clusterIndices);
//
//    for (pcl::PointIndices ind : clusterIndices) {
//    	typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
//    	for (int clustInd : ind.indices) {
//    		cluster->points.push_back(cloud->points[clustInd]);
//    	}
//    	clusters.push_back(cluster);
//    }

    // using my own implementation of clustering and KdTree
	std::vector<std::vector<float>> points;
	KdTree* tree = new KdTree;
	for (int i = 0; i < cloud->points.size(); i++) {
		auto point = cloud->points[i];
		tree->insert({point.x, point.y, point.z}, i);
		points.push_back({point.x, point.y, point.z});
	}
	std::vector<std::vector<int>> indices = euclideanCluster(points, tree, clusterTolerance);

	for (std::vector<int> v : indices) {
		typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
		if (v.size() >= minSize && v.size() <= maxSize) {
			for (int clustInd : v) {
				cluster->points.push_back(cloud->points[clustInd]);
			}
			clusters.push_back(cluster);
			std::cout << "cluster size: " << v.size() << std::endl;
		}
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
