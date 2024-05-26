#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/filters/voxel_grid.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr accumulatedPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr accumulatedGroundCloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
ros::Subscriber subscriber;
ros::Time lastSegmentationTime;


void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr receivedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloudMsg, *receivedCloud);
    *pointCloud += *receivedCloud;
}

void appendUniquePointsKDTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& targetCloud) {
    // Building a KD-tree from the target cloud
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(targetCloud);

    // Define search parameters
    float radius = 1e-4; // Small radius for "equality" (consider float precision)
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // Check each point in the source cloud to see if it exists in the target cloud
    for (const auto& point : sourceCloud->points) {
        if (!kdtree.radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)) {
            // If no points are within the small radius, the point is considered unique
            targetCloud->push_back(point);
        }
    }
}

#include <cmath> // For cos and sin calculations

void segmentPointCloud(ros::Publisher blocked_nodes_pub)
{
    appendUniquePointsKDTree(pointCloud, accumulatedPointCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr concatenatedPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    *concatenatedPointCloud = *pointCloud;
    *concatenatedPointCloud += *accumulatedGroundCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonGroundCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.5);

    // Set the expected axis of the plane
    double maxInclinationAngle = 10.0 * M_PI / 180.0;  // Convert degrees to radians
    Eigen::Vector3f axis(cos(maxInclinationAngle), sin(maxInclinationAngle), 0); // Assuming tilt in the XZ plane
    seg.setAxis(axis);
    seg.setEpsAngle(maxInclinationAngle); // Allow 10 degrees deviation

    seg.setInputCloud(concatenatedPointCloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(concatenatedPointCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*groundCloud);


	pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
    voxelFilter.setInputCloud(pointCloud);
    voxelFilter.setLeafSize(0.1f, 0.1f, 0.1f);  // Set the voxel size
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized(new pcl::PointCloud<pcl::PointXYZ>());
    voxelFilter.filter(*voxelized);

    // Build KD-tree for accumulated ground points
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(accumulatedGroundCloud);

    // Process voxelized points
    sensor_msgs::PointCloud blockedNodes;
    blockedNodes.header.stamp = ros::Time::now();
    blockedNodes.header.frame_id = "map";
    
    int K = 1; // Number of nearest neighbors
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    for (const auto& point : voxelized->points) {
        if (kdtree.nearestKSearch(point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            if (sqrt(pointNKNSquaredDistance[0]) <= 0.5) { // Check if the closest ground point is within 0.5 meters
                geometry_msgs::Point32 roundedPoint;
                roundedPoint.x = std::round(point.x * 2) / 2.0;
                roundedPoint.y = std::round(point.y * 2) / 2.0;
                roundedPoint.z = 0;
                blockedNodes.points.push_back(roundedPoint);
            }
        }
    }

    // Publish the blocked nodes
    blocked_nodes_pub.publish(blockedNodes);


   // *accumulatedGroundCloud = *groundCloud;
    appendUniquePointsKDTree(groundCloud, accumulatedPointCloud);


    extract.setNegative(true);
    extract.filter(*nonGroundCloud);

    pointCloud->clear();

    viewer->removeAllPointClouds();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> groundColorHandler(accumulatedGroundCloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(accumulatedPointCloud, groundColorHandler, "Accumulated Ground Cloud");

    //viewer->addPointCloud<pcl::PointXYZ>(nonGroundCloud, "Non-Ground Cloud");
    viewer->spinOnce();

    ROS_INFO("Number of points in the accumulated ground cloud: %lu", accumulatedGroundCloud->size());
    ROS_INFO("Number of points in the non-ground cloud: %lu", nonGroundCloud->size());
    ROS_INFO("Number of points in the accumulated-point cloud: %lu", accumulatedPointCloud->size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "orb_slam3_map_processor");
    ros::NodeHandle nh;

    pointCloud->header.frame_id = "map";
    accumulatedGroundCloud->header.frame_id = "map";

    subscriber = nh.subscribe("/orb_slam3/map_points", 1000, pointCloudCallback);

    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();

    ros::Rate loop_rate(10);

    //Initialization takes longer, accumulate more ground points
     
    lastSegmentationTime = ros::Time::now();
    ros::Time currentTime = ros::Time::now();

    pcl::PointXYZ newPoint(0, 0, 0);    
    accumulatedPointCloud->push_back(newPoint);
    pcl::PointXYZ newPoint2(0, 0, 0);    
    accumulatedGroundCloud->push_back(newPoint2);

    
bool isInitialized = false;
ros::Time initializationStartTime = ros::Time::now();

while (ros::ok() && !viewer->wasStopped())
{
    ros::spinOnce();
    ros::Time currentTime = ros::Time::now();

    // Wait for 10 seconds on initialization (only once)
    if (!isInitialized)
    {
        if ((currentTime - initializationStartTime).toSec() <= 10)
        {
            ROS_INFO("Waiting for initialization... %f seconds remaining", 10 - (currentTime - initializationStartTime).toSec());
            ros::Duration(0.1).sleep(); // Sleep for 100ms to avoid excessive CPU usage
        }
        else
        {
            isInitialized = true;
            lastSegmentationTime = currentTime; // Set the last segmentation time to the current time
        }
    }
    ros::init(argc, argv, "orb_slam3_map_processor");
    ros::NodeHandle nh;
    ros::Publisher blocked_nodes_pub = nh.advertise<sensor_msgs::PointCloud>("blocked_nodes", 10);


    if (isInitialized && (currentTime - lastSegmentationTime).toSec() >= 0.2)
    {
        segmentPointCloud(blocked_nodes_pub);
        lastSegmentationTime = currentTime;
    }

    loop_rate.sleep();
}

return 0;
}
