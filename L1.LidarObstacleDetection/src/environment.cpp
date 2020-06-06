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
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // Done:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = lidar->scan();

    // Done:: Create point processor: heap/stack
    //ProcessPointClouds<pcl::PointXYZ> point_processor;
    ProcessPointClouds<pcl::PointXYZ>* point_processor = new ProcessPointClouds<pcl::PointXYZ>();

    // segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> seg = point_processor->SegmentPlane(input_cloud, 100, 0.2);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_cloud = point_processor->Clustering(seg.first, 1.0, 3, 30);

    // renderRays(viewer, lidar->position, input_cloud);
    // renderPointCloud(viewer, input_cloud, "input_cloud");

    // render obstacle and plane
    //renderPointCloud(viewer, seg.first, "obstacle", Color(1,0,0));
    renderPointCloud(viewer, seg.second, "plane", Color(1,1,0));

    // render cluster
    int cluster_id = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: clusters_cloud)
    {
        // render cluster point cloud
        renderPointCloud(viewer, cluster, "obstacle_cloud "+std::to_string(cluster_id), colors[cluster_id]);

        // render box
        Box box = point_processor->BoundingBox(cluster);
        renderBox(viewer, box, cluster_id, colors[cluster_id], 1);

        cluster_id++;
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* point_processor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = point_processor->FilterCloud(input_cloud, 0.2, Eigen::Vector4f(-30, -6.5, -3, 1), Eigen::Vector4f(30, 6.5, 10, 1));
    std::cout << "org size: " << input_cloud->width * input_cloud->height << endl;
    std::cout << "filtered size: " << filterCloud->width * filterCloud->height << endl;

    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> seg = point_processor->SegmentPlane (filterCloud, 100, 0.3);
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters_cloud = point_processor->Clustering(seg.first, 0.6, 10, 5000);

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> seg = point_processor->RansacPlane(filterCloud, 100, 0.3);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters_cloud = point_processor->ClusteringRubric(seg.first, 0.6, 10, 5000);

    renderPointCloud(viewer, seg.second, "plane", Color(1,1,0));

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: clusters_cloud)
    {
        renderPointCloud(viewer, cluster, "obstacle_cloud " + std::to_string(clusterId), colors[clusterId % 3]);

        Box box = point_processor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(1,0,0), 1);

        clusterId++;
    }

}

void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // Set position and angle of the camera
    viewer->initCameraParameters();
    // Init distance (meters)
    int distance = 20;

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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // simpleHighway(viewer);

    // Real PCD data
    ProcessPointClouds<pcl::PointXYZI>* point_processor = new ProcessPointClouds<pcl::PointXYZI>();

    // Reading from stream
    std::vector<boost::filesystem::path> stream = point_processor->streamPcd("../src/sensors/data/pcd/data_1");
    auto stream_iter = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;

    // stream process
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load PCD
        input_cloud = point_processor->loadPcd((*stream_iter).string());

        // Run obstacle detection
        cityBlock(viewer, point_processor, input_cloud);

        stream_iter++;
        if (stream_iter == stream.end())
            stream_iter = stream.begin();

        viewer->spinOnce ();
    }
}