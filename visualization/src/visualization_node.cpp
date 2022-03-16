#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

void mean_path(visualization_msgs::Marker &points, visualization_msgs::Marker &line_strip)
{
  std::vector<std::vector<std::string>> content;
  std::vector<std::string> row;
  std::string line, word;
  std::fstream file(("/home/akshit/ros/drdo22_ws/src/inter_iit_uav_guided_ugv/prius_controller/testing/mean_path/world2/world2.csv"), std::ios::in);
  float f = 0.0;
  geometry_msgs::Point p;
  if (file.is_open())
  {
    while (getline(file, line))
    {
      row.clear();

      std::stringstream str(line);

      while (getline(str, word, ','))
        row.push_back(word);
      p.x = std::stod(row[0]);
      p.y = std::stod(row[1]);
      p.z = std::stod(row[2]);
      points.points.push_back(p);
      line_strip.points.push_back(p);
      // content.push_back(row);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_visualization");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/trajectory", 10);

  ros::Rate r(30);

  visualization_msgs::Marker points, line_strip, line_list;

  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/base_link";
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = "trajectory";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  points.scale.x = 0.2;
  points.scale.y = 0.2;

  line_strip.scale.x = 0.1;

  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  mean_path(points, line_strip);

  while (ros::ok())
  {

    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    r.sleep();
  }
}