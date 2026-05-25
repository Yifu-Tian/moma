#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace
{
geometry_msgs::Point makePoint(double x, double y, double z)
{
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

visualization_msgs::Marker makeMarkerBase(
    const std::string &ns, int id, int type, const std::string &frame_id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = ros::Duration(0.0);
  return marker;
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "charging_demo_helper");
  ros::NodeHandle nh("~");

  std::vector<double> port_surface_position{ -0.60, 0.25, 0.45 };
  std::vector<double> goal_position{ -0.60, 0.05, 0.45 };
  std::vector<double> port_normal{ 0.0, -1.0, 0.0 };
  nh.param<std::vector<double>>("charging_demo/port_surface_position", port_surface_position, port_surface_position);
  nh.param<std::vector<double>>("charging_demo/goal_position", goal_position, goal_position);
  nh.param<std::vector<double>>("charging_demo/port_normal", port_normal, port_normal);

  bool auto_trigger = true;
  double trigger_delay = 2.0;
  nh.param("charging_demo/auto_trigger", auto_trigger, true);
  nh.param("charging_demo/trigger_delay", trigger_delay, 2.0);

  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>("/charging_demo/markers", 1, true);
  ros::Publisher rviz_default_marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>("/model_vis/vis_mm", 1, true);
  ros::Publisher goal_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);

  const std::string frame_id = "world";
  const double px = port_surface_position.size() > 0 ? port_surface_position[0] : -0.60;
  const double py = port_surface_position.size() > 1 ? port_surface_position[1] : 0.25;
  const double pz = port_surface_position.size() > 2 ? port_surface_position[2] : 0.45;
  const double gx = goal_position.size() > 0 ? goal_position[0] : -0.60;
  const double gy = goal_position.size() > 1 ? goal_position[1] : 0.05;
  const double gz = goal_position.size() > 2 ? goal_position[2] : 0.45;
  const double nx = port_normal.size() > 0 ? port_normal[0] : 0.0;
  const double ny = port_normal.size() > 1 ? port_normal[1] : -1.0;
  const double nz = port_normal.size() > 2 ? port_normal[2] : 0.0;
  const double normal_norm = std::max(1e-6, std::sqrt(nx * nx + ny * ny + nz * nz));

  visualization_msgs::MarkerArray markers;
  {
    auto port = makeMarkerBase("charging_port", 0, visualization_msgs::Marker::CUBE, frame_id);
    const double port_length = std::max(0.05, std::abs(py - gy));
    port.pose.position = makePoint(0.5 * (px + gx), 0.5 * (py + gy), 0.5 * (pz + gz));
    port.scale.x = 0.08;
    port.scale.y = port_length;
    port.scale.z = 0.08;
    port.color.r = 0.05;
    port.color.g = 0.75;
    port.color.b = 1.0;
    port.color.a = 1.0;
    markers.markers.push_back(port);
  }
  {
    auto target = makeMarkerBase("charging_port", 1, visualization_msgs::Marker::SPHERE, frame_id);
    target.pose.position = makePoint(gx, gy, gz);
    target.scale.x = 0.10;
    target.scale.y = 0.10;
    target.scale.z = 0.10;
    target.color.r = 0.0;
    target.color.g = 1.0;
    target.color.b = 0.45;
    target.color.a = 0.9;
    markers.markers.push_back(target);
  }
  {
    auto normal = makeMarkerBase("charging_port", 2, visualization_msgs::Marker::ARROW, frame_id);
    normal.points.push_back(makePoint(px, py, pz));
    normal.points.push_back(makePoint(px + 0.25 * nx / normal_norm,
                                      py + 0.25 * ny / normal_norm,
                                      pz + 0.25 * nz / normal_norm));
    normal.scale.x = 0.025;
    normal.scale.y = 0.055;
    normal.scale.z = 0.055;
    normal.color.r = 0.0;
    normal.color.g = 0.45;
    normal.color.b = 1.0;
    normal.color.a = 1.0;
    markers.markers.push_back(normal);
  }

  ros::Rate rate(10.0);
  const ros::Time start_time = ros::Time::now();
  bool triggered = false;
  while (ros::ok())
  {
    for (auto &marker : markers.markers)
    {
      marker.header.stamp = ros::Time::now();
    }
    marker_pub.publish(markers);
    rviz_default_marker_pub.publish(markers);

    if (auto_trigger && !triggered &&
        (ros::Time::now() - start_time).toSec() >= trigger_delay)
    {
      geometry_msgs::PoseStamped goal;
      goal.header.frame_id = frame_id;
      goal.header.stamp = ros::Time::now();
      goal.pose.position.x = gx;
      goal.pose.position.y = gy;
      goal.pose.position.z = 0.0;
      goal.pose.orientation.w = 1.0;
      goal_pub.publish(goal);
      ROS_INFO("[charging_demo] Triggered preset REMANI waypoint for charging port demo.");
      triggered = true;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
