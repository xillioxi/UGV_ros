#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <random>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_pointcloud_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/orb_slam3/map_points", 10);
    ros::Rate rate(1000); // Publish once per second

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1000);

    while (ros::ok())
    {
        sensor_msgs::PointCloud2 cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = "map";
        cloud.height = 1;  // Unordered point cloud
        cloud.width = 12;  // Total 12 points per packet

        // Define the point cloud structure
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                         "y", 1, sensor_msgs::PointField::FLOAT32,
                                         "z", 1, sensor_msgs::PointField::FLOAT32);
        modifier.resize(12);

        // Fill the PointCloud2 message
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (size_t i = 0; i < cloud.width; ++i, ++iter_x, ++iter_y, ++iter_z)
        {
            *iter_x = dis(gen);
            *iter_y = dis(gen);
            *iter_z = dis(gen);
        }

        cloud.is_bigendian = false;  // Set to little endian
        cloud.is_dense = true;       // No invalid points

        ROS_INFO_STREAM("Publishing cloud with " << cloud.width << " points");
        pub.publish(cloud);

        rate.sleep();
    }

    return 0;
}
