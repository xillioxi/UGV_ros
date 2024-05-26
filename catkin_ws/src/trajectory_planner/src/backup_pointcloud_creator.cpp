#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr accumulatedGroundCloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
ros::Subscriber subscriber;
ros::Time lastSegmentationTime;

pcl::PointXYZ createPoint(const geometry_msgs::Point::ConstPtr& coordinates)
{
    pcl::PointXYZ point;
    point.x = coordinates->x;
    point.y = coordinates->y;
    point.z = coordinates->z;
    return point;
}

void coordinatesCallback(const geometry_msgs::Point::ConstPtr& coordinates)
{
    ROS_INFO("Received coordinates: x=%f, y=%f, z=%f", coordinates->x, coordinates->y, coordinates->z);
    pcl::PointXYZ point = createPoint(coordinates);
    pointCloud->points.push_back(point);
}

void segmentPointCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonGroundCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(pointCloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(pointCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*groundCloud);

    *accumulatedGroundCloud += *groundCloud;

    extract.setNegative(true);
    extract.filter(*nonGroundCloud);

    pointCloud->clear();

    viewer->removeAllPointClouds();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> groundColorHandler(accumulatedGroundCloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(accumulatedGroundCloud, groundColorHandler, "Accumulated Ground Cloud");

    viewer->addPointCloud<pcl::PointXYZ>(nonGroundCloud, "Non-Ground Cloud");
    viewer->spinOnce();

    ROS_INFO("Number of points in the accumulated ground cloud: %lu", accumulatedGroundCloud->size());
    ROS_INFO("Number of points in the non-ground cloud: %lu", nonGroundCloud->size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_coordinates_receiver");
    ros::NodeHandle nh;

    pointCloud->header.frame_id = "map";
    accumulatedGroundCloud->header.frame_id = "map";
    lastSegmentationTime = ros::Time::now();

    subscriber = nh.subscribe("random_coordinates", 1000, coordinatesCallback);

    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();

    ros::Rate loop_rate(10);
    while (ros::ok() && !viewer->wasStopped())
    {
        ros::spinOnce();

        ros::Time currentTime = ros::Time::now();
        if ((currentTime - lastSegmentationTime).toSec() >= 0.2)
        {
            segmentPointCloud();
            lastSegmentationTime = currentTime;
        }

        loop_rate.sleep();
    }

    return 0;
}
