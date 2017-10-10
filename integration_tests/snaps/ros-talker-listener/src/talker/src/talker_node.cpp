#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle nodeHandle;

    ros::Publisher publisher = nodeHandle.advertise<std_msgs::String>("chatter", 1);

    ros::Rate loopRate(10);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String message;

        std::stringstream stream;
        stream << "Hello world " << count++;
        message.data = stream.str();

        ROS_INFO("%s", message.data.c_str());

        publisher.publish(message);

        ros::spinOnce();

        loopRate.sleep();
    }

    return 0;
}
