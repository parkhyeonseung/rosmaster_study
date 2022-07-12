#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"turtle1_turtle2_listener");
    ros::NodeHandle node;
    ros::service::waitForService("/spawn");
    ros::ServiceClient add_turtle = node.serviceClient <turtlesim::Spawn>("/spawn");
    turtlesim::Spawn srv;
    add_turtle.call(srv);

    ros::Publisher vel = node.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",10);
    tf::TransformListener listener;
    ros::Rate rate(10.0);

    while (node.ok()){
        tf::StampedTransform transform;
        try {
            listener.waitForTransform("/turtle2","/turtle1",ros::Time(0) ,ros::Duration(3.0));
            listener.lookupTransform("/turtle2","/turtle1",ros::Time(0),transform);
        }
        catch (tf::TransformException & ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Twist turtle2_vel_msg;

        turtle2_vel_msg.angular.z = 6.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
        turtle2_vel_msg.linear.x = 0.8 * sqrt(pow(transform.getOrigin().x(),2) + pow(transform.getOrigin().y(),2));
        vel.publish(turtle2_vel_msg);
        rate.sleep();
        }

        return 0;
    
};