/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

// Project: Switch from PCL algorithm to KD-Tree Clustering
//#include "../../render/render.h"
//#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtreeProject.h"

// Create ClusterKDT class
template<typename PointT>
class ClusterKDT {
private:
  int numPoints;
  float distanceTol;
  int minClusterSize;
  int maxClusterSize;
  std::vector<bool> processed;
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

public:
  ClusterKDT(int nPts, float distTol, int minSize, int maxSize) :
      numPoints(nPts), distanceTol(distTol), minClusterSize(minSize), maxClusterSize(maxSize) {
        processed.assign(numPoints, false);
      }
  ~ClusterKDT();

  void clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, KdTree* tree);
  std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud);
};

template<typename PointT>
ClusterKDT<PointT>::~ClusterKDT() {}

// Lesson 3, Chapter 8
template<typename PointT>
void ClusterKDT<PointT>::clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, KdTree* tree) 
{
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(cloud->points[indice], distanceTol);
	for (int id: nearest){
		if (!processed[id]){
			clusterHelper(id, cloud, cluster, tree);
		}
	}
}

//std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusterKDT<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud)
{
	// Create tree object
	KdTree* tree = new KdTree;

	// Insert cloud points to the tree
	for (int i=0; i<numPoints; i++)
	{
		tree->insert(cloud->points[i],i);
	}

	// Calculate by using clusterHelper
	int i = 0;
	while (i < numPoints){
		if (processed[i]){
			i++;
			continue;
		}
		std::vector<int> cluster;
		typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
    	clusterHelper(i, cloud, cluster, tree);

		int clusterSize = cluster.size();

		if (clusterSize >= minClusterSize && clusterSize <= maxClusterSize){
			for (int j=0; j < clusterSize; j++){
				cloudCluster->points.push_back(cloud->points[cluster[j]]);
			}
			cloudCluster->width = cloudCluster->points.size();
			cloudCluster->height = 1;
			clusters.push_back(cloudCluster);
		}
	
	}
	return clusters;
}