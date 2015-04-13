/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:    Dario Limongi                                              *
 *   Email:     dario.limongi@gmail.com                                    *
 *   Date:      12/08/2014                                                 *
 *                                                                         *
 ***************************************************************************/

#include <stdio.h>
#include <curl/curl.h>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>


using namespace std;

int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "osm_map_test_node");
    ros::NodeHandle nh;

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "/osm_map_test_node/marker", 0 );
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>( "/osm_map_test_node/pose", 0 );
    ros::Publisher array_pub = nh.advertise<geometry_msgs::PoseArray>( "/osm_map_test_node/array", 0 );

    while(ros::ok()){

        // MARKER-----------------------------------------------
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 516788;
        marker.pose.position.y = 5041109;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        vis_pub.publish( marker );

        // POSE ------------------------------------------------
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = 516788;
        pose.pose.position.y = 5041109;
        pose.pose.position.z = 1;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose_pub.publish(pose);

        // POSE ARRAY -----------------------------------------
        geometry_msgs::Pose p1;
        p1.position.x = 516782.249439;
        p1.position.y = 5041088.72838;
        p1.position.z = 0.0;
        p1.orientation.x = 0.0;
        p1.orientation.y = 0.0;
        p1.orientation.z = 0.0;
        p1.orientation.w = 1.0;

        geometry_msgs::Pose p2;
        p2.position.x = 516757.213693;
        p2.position.y = 5041084.33928;
        p2.position.z = 0.0;
        p2.orientation.x = 0.0;
        p2.orientation.y = 0.0;
        p2.orientation.z = 0.0;
        p2.orientation.w = 1.0;

        geometry_msgs::Pose p3;
        p3.position.x = 516680.334629;
        p3.position.y = 5041075.52764;
        p3.position.z = 0.0;
        p3.orientation.x = 0.0;
        p3.orientation.y = 0.0;
        p3.orientation.z = 0.0;
        p3.orientation.w = 1.0;

        geometry_msgs::PoseArray p_array;
        p_array.poses.push_back(p1);
        p_array.poses.push_back(p2);
        p_array.poses.push_back(p3);
        p_array.header.frame_id = "map";
        p_array.header.stamp = ros::Time::now();

        array_pub.publish(p_array);

        //--------------------------------------------------------------------------
        ros::spinOnce();
    }
    return 0;
}

