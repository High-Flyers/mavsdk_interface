#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry& odom)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(
        tf::Vector3(
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z
        )
    );
    tf::Quaternion q;
    q.setValue(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    );
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavsdk_base_link_tf");
    ros::NodeHandle node("~");
    std::string odom_sub = "";
    node.getParam("odom", odom_sub);
    if(odom_sub == "")
    {
        ROS_ERROR("SPECIFY ODOMETRY FOR TF PUB NODE!");
        return 1;
    }
    ros::Subscriber sub = node.subscribe(odom_sub, 60, &odomCallback);
    ros::spin();
    return 0;
}