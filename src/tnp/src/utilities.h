#ifndef UTILITIES_H
#define UTILITIES_H

#define COVERAGE_DEPTH 999

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>


#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

const double PI = 3.1415926;
const static double r_earth = 6378.137; // in kilometers

// each face has some info..
struct FaceInfo2
{
  FaceInfo2(){}

  bool visited, numbered, path_visited, cover_depth;
  int id, depth, agent_id, jumps_agent_id, coverage_depth;

  void initialize(int face_id){
    visited = numbered = path_visited = cover_depth = false;
    depth = 0;
    coverage_depth = COVERAGE_DEPTH;
    id = face_id;
  }

  bool is_visited(){
    return visited;
  }
  bool is_path_visited(){
    return path_visited;
  }
  bool has_number(){
    return numbered;
  }
  bool has_coverage_depth(){
    return cover_depth;
  }
};

// the constructions Kernel
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 kernel_Point_2;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Constrained_triangulation_face_base_2<K,Fb>        CTFb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K,CTFb>    InfoFbb;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_data_structure_2<Vb, InfoFbb> Tds;
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds, Itag> CDT;
typedef CDT::Point triangulation_Point;

geometry_msgs::Point cgal_triangulation_point_to_ros_geometry_point(triangulation_Point cgal_point, float elevation);
geometry_msgs::Point cgal_point_to_ros_geometry_point(kernel_Point_2 cgal_point);
geometry_msgs::PoseStamped build_pose_stamped(geometry_msgs::Point point_position);
geometry_msgs::Point32 point_to_point_32(geometry_msgs::Point point);

geometry_msgs::Point face_points_to_center(triangulation_Point point1, triangulation_Point point2, triangulation_Point point3);
geometry_msgs::Point face_to_center(CDT& cdt, CDT::Face_handle face);

#endif // UTILITIES_H
