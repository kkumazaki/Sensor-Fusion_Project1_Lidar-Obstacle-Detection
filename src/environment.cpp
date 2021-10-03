/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    // Lesson 1, Chapter 19
    //bool renderScene = true;
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    // Lesson 1, Chapter 15
    Lidar* lidar = new Lidar(cars, 0);

    // Lesson 1, Chapter 16
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // Lesson 1, Chapter 19
    renderRays(viewer, lidar->position, inputCloud);
    renderPointCloud(viewer, inputCloud, "inputCloud", Color(1,1,1));

    /* // Lesson 4, Chapter 6: Avoid unnecessary error
    // TODO:: Create point processor
    // Lesson 2, Chapter 3
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    //ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new  ProcessPointClouds<pcl::PointXYZ>();//Either one is fine.

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    //renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    // Lesson 3, Chapter 3 (Euclidean Clustering with PCL)
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {  
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        // Lesson 3, Chapter 9 (Bounding Boxes)
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
    // Lesson 3, Chapter 9 (Bounding Boxes)
    //renderPointCloud(viewer, segmentCloud.second, "placeCloud");
    */
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

// Lesson 4, Chapter 2 (Load PCD)
// Lesson 4, Chapter 7 (Stream PCD)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  // Parameters
  // Filter Parameters
  float filterRes = 0.3;
  Eigen::Vector4f minPoint(-15, -6.5, -2, 1);
  Eigen::Vector4f maxPoint( 30,  6.5,  1, 1);
  // Segmentation Parameters
  int maxIterations = 25;
  float distanceThreshold = 0.2;
  //int maxIterations = 100;
  //float distanceThreshold = 0.2;
  // Clustering Parameters
  float clusterTolerance = 0.5;
  int minSize = 10;
  int maxSize = 500;
  //float clusterTolerance = 1.0;
  //int minSize = 3;
  //int maxSize = 30;

  // Input Cloud Data
  // Lesson 4, Chapter 7 (Stream PCD)
  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  //renderPointCloud(viewer,inputCloud,"inputCloud");

  // Lesson 4, Chapter 5 (Filtering with PCL)
  // Experiment with the values and find what works best
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filterRes , minPoint, maxPoint);
  //renderPointCloud(viewer,filterCloud,"filterCloud");

  // Lesson 4, Chapter 6 (step1: Segment the filtered cloud into two parts, road and obstacles.)
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, maxIterations, distanceThreshold);
  //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

  // Lesson 4, Chapter 6 (step2: Cluster the obstacle cloud.)
  //  (Euclidean Clustering with PCL)
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, clusterTolerance, minSize, maxSize);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {  
      std::cout << "cluster size ";
      pointProcessorI->numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
      //  (step3: Find bounding boxes for the clusters.)
      Box box = pointProcessorI->BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
      ++clusterId;
  }
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // Lesson 4, Chapter 7 (Stream PCD)
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    // Lesson 4, Chapter 2 (Load PCD)
    //simpleHighway(viewer);
    //cityBlock(viewer);

    // Lesson 4, Chapter 7 (Stream PCD)
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}