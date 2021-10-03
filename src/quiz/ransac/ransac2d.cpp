/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cmath>

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
	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// Lesson 2, Chapter 7 (2D)
	// Lesson 2, Chapter 8 (3D)

	while (maxIterations--){
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
		//while (inliers.size() < 2)
			inliers.insert(rand()%(cloud->points.size())); //unordered_set will reject the same index adding.

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		//float x1, y1, x2, y2;

		auto itr = inliers.begin(); //pointer of the begining of inliers.

		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float A = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 -y1);
		float B = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 -z1);
		float C = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 -x1);
		float D = -(A*x1 + B*y1 + C*z1);
		//float A = (y1 - y2);
		//float B = (x2 - x1);
		//float C = (x1*y2 - x2*y1);

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		
		for (int index = 0; index < cloud->points.size(); index++){
			if (inliers.count(index) > 0)
				continue;
			
			pcl::PointXYZ point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			float d = fabs(A*x4 + B*y4 + C*z4 + D)/sqrt(A*A + B*B +C*C);
			//float x3 = point.x;
			//float y3 = point.y;
			//float d = fabs(A*x3 + B*y3 + C)/sqrt(A*A + B*B);
			if (d <= distanceTol){
				inliers.insert(index);
			}
		}

		if (inliers.size() > inliersResult.size()){
			inliersResult = inliers; // Update the result if there are more inliers
		}			
	}
	// Return indicies of inliers from fitted line with most inliers
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// Lesson 2, Chapter 8
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// Lesson 2, Chapter 7
	//std::unordered_set<int> inliers = Ransac(cloud, 0, 0);
	std::unordered_set<int> inliers = Ransac(cloud, 30, 0.2);

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
