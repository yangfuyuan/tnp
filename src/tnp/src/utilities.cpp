#include "utilities.h"

geometry_msgs::Point cgal_triangulation_point_to_ros_geometry_point(triangulation_Point cgal_point, float elevation)
{
  geometry_msgs::Point ros_point;
  ros_point.x = cgal_point.x();
  ros_point.y = cgal_point.y();
  ros_point.z = elevation;

  return ros_point;
}

geometry_msgs::Point cgal_point_to_ros_geometry_point(kernel_Point_2 cgal_point) {

  geometry_msgs::Point ros_point;
  ros_point.x = cgal_point.x();
  ros_point.y = cgal_point.y();
  ros_point.z = 0.0;

  return ros_point;
}

geometry_msgs::PoseStamped build_pose_stamped(geometry_msgs::Point point_position){

  geometry_msgs::Pose pose;
  pose.position = point_position;
  pose.orientation.w = 0.0;

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose = pose;
  pose_stamped.header.frame_id = "/my_frame";
  pose_stamped.header.stamp = ros::Time::now();

  return pose_stamped;

}

geometry_msgs::Point32 point_to_point_32(geometry_msgs::Point point) {

  geometry_msgs::Point32 point32;
  point32.x = point.x;
  point32.y = point.y;
  point32.z = point.z;

  return point32;
}


geometry_msgs::Point face_points_to_center(triangulation_Point point1, triangulation_Point point2, triangulation_Point point3){

  geometry_msgs::Point center;
  center.x =  ( (point1.x() + point2.x() + point3.x()) / 3);
  center.y =  ( (point1.y() + point2.y() + point3.y()) / 3);
  center.z = 0;

  return center;
}


geometry_msgs::Point face_to_center(CDT& cdt, CDT::Face_handle face){

  // create a point for each of the edges of the face.
  triangulation_Point point1 = cdt.triangle(face)[0];
  triangulation_Point point2 = cdt.triangle(face)[1];
  triangulation_Point point3 = cdt.triangle(face)[2];

  return face_points_to_center(point1, point2, point3);

}
