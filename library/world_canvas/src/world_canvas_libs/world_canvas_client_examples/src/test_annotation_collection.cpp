/*
 * test_annotation_collection.cpp
 *
 *  Created on: Jul 8, 2014
 *      Author: jorge
 */

#include <ros/ros.h>

#include <yocs_msgs/Wall.h>
#include <yocs_msgs/Column.h>
#include <nav_msgs/OccupancyGrid.h>

#include <world_canvas_client_cpp/annotation_collection.hpp>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_annotation_collection");

  // Parameters
  ros::NodeHandle nh("~");

  std::string world_name;
  std::string topic_name;
  std::string topic_type;
  std::string default_tn("annotations");
  std::string default_tt;
  std::string default_wn("INVALID_WORLD");
  bool        pub_as_list;
  std::vector<std::string> uuids;
  std::vector<std::string> names;
  std::vector<std::string> types;
  std::vector<std::string> keywords;
  std::vector<std::string> relationships;

  nh.param("world",         world_name, default_wn);
  nh.param("topic_name",    topic_name, default_tn);
  nh.param("topic_type",    topic_type, default_tt);
  nh.param("pub_as_list",   pub_as_list, false);
  nh.param("uuids",         uuids, uuids);
  nh.param("names",         names, names);
  nh.param("types",         types, types);
  nh.param("keywords",      keywords, keywords);
  nh.param("relationships", relationships, relationships);

  // Prepare the annotation collection
  wcf::FilterCriteria filter(world_name, uuids, names, types, keywords, relationships);
  wcf::AnnotationCollection ac(filter);
  ac.loadData();
  ROS_INFO("Annotation collection ready!");

  // Publish annotations' visual markers on client side
  ac.publishMarkers("annotation_markers");

  // Request server to publish the annotations
  ac.publish(topic_name, topic_type, true, pub_as_list);

  std::vector<yocs_msgs::Wall> walls;
  std::vector<yocs_msgs::Column> columns;
  std::vector<nav_msgs::OccupancyGrid> maps;

  ROS_INFO("Done!  got %u walls, %u columns and %u maps",
           ac.getData(walls), ac.getData(columns), ac.getData(maps));
  ROS_INFO("(for confirmation    %lu   %lu   %lu)", walls.size(), columns.size(), maps.size());
  ros::Publisher wp = nh.advertise<yocs_msgs::Wall> ("walls_on_client", 1, true);
  ros::Publisher cp = nh.advertise<yocs_msgs::Column> ("columns_on_client", 1, true);
  ros::Publisher mp = nh.advertise<nav_msgs::OccupancyGrid> ("maps_on_client", 1, true);

  for (unsigned int i = 0; i < walls.size(); i++)
    wp.publish(walls[i]);
  for (unsigned int i = 0; i < columns.size(); i++)
    cp.publish(columns[i]);
  for (unsigned int i = 0; i < maps.size(); i++)
    mp.publish(maps[i]);

  ros::spin();

  return 0;
}
