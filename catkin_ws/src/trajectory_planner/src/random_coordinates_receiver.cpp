#include <ros/ros.h>
#include <geometry_msgs/Point.h>

void coordinatesCallback(const geometry_msgs::Point::ConstPtr& coordinates)
{
    ROS_INFO("Received coordinates: x=%f, y=%f, z=%f", coordinates->x, coordinates->y,coordinates->z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_coordinates_receiver");
    ros::NodeHandle nh;

    ros::Subscriber subscriber = nh.subscribe("random_coordinates", 10, coordinatesCallback);

    ros::spin();

    return 0;
}
