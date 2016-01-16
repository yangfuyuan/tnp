#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

#include <CGAL/Simple_cartesian.h>

#include <CGAL/partition_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/polygon_function_objects.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/point_generators_2.h>

#include <cmath>
#include <vector>
#include <cassert>
#include <list>

// the cartesian Kernel
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 kernel_Point_2;
typedef Kernel::Segment_2 Segment_2;

// the constructions Kernel
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K> Traits;
typedef Traits::Polygon_2 Polygon_2;

// defining a standard list of polygons
typedef std::list<Polygon_2> Polygon_list;
typedef Polygon_2::Edge_const_iterator EdgeIterator;
// different kind of points for the construction kernel
typedef Traits::Point_2 construction_Point_2;

geometry_msgs::Point cgal_construction_point_to_ros_geometry_point(construction_Point_2 cgal_point) {

  geometry_msgs::Point ros_point;
  ros_point.x = cgal_point.x();
  ros_point.y = cgal_point.y();
  ros_point.z = 0.0;    

  return ros_point;
}

geometry_msgs::Point cgal_point_to_ros_geometry_point(kernel_Point_2 cgal_point) {

  geometry_msgs::Point ros_point;
  ros_point.x = cgal_point.x();
  ros_point.y = cgal_point.y();
  ros_point.z = 0.0;    

  return ros_point;
}

// create the initial CGAL polygon
void make_polygon(Polygon_2& polygon, std::vector<kernel_Point_2>& polygon_edges){

   // ------- A fairly simple polygon, segmented to 2 -----
   // polygon.push_back(construction_Point_2(50, 50));
   // polygon_edges.push_back(kernel_Point_2(50,50));

   // polygon.push_back(construction_Point_2(150, 50));
   // polygon_edges.push_back(kernel_Point_2(150,50));

   // polygon.push_back(construction_Point_2(150, 350));
   // polygon_edges.push_back(kernel_Point_2(150,350));

   // polygon.push_back(construction_Point_2(20, 150));
   // polygon_edges.push_back(kernel_Point_2(20,150));

   // polygon.push_back(construction_Point_2(50, 150));
   // polygon_edges.push_back(kernel_Point_2(50,150));
   // -----------------------------------------------------

   // -- A more complex one, segmented to 7 sub-areas ----
   polygon.push_back(construction_Point_2(391, 374));
   polygon_edges.push_back(kernel_Point_2(391,374));

   polygon.push_back(construction_Point_2(240, 431));
   polygon_edges.push_back(kernel_Point_2(240,431));

   polygon.push_back(construction_Point_2(252, 340));
   polygon_edges.push_back(kernel_Point_2(252,340));

   polygon.push_back(construction_Point_2(374, 320));
   polygon_edges.push_back(kernel_Point_2(374,320));

   polygon.push_back(construction_Point_2(289, 214));
   polygon_edges.push_back(kernel_Point_2(289,214));

   polygon.push_back(construction_Point_2(134, 390));
   polygon_edges.push_back(kernel_Point_2(134,390));

   polygon.push_back(construction_Point_2( 68, 186));
   polygon_edges.push_back(kernel_Point_2(68,186));

   polygon.push_back(construction_Point_2(154, 259));
   polygon_edges.push_back(kernel_Point_2(154,259));

   polygon.push_back(construction_Point_2(161, 107));
   polygon_edges.push_back(kernel_Point_2(161,107));

   polygon.push_back(construction_Point_2(435, 108));
   polygon_edges.push_back(kernel_Point_2(435,108));

   polygon.push_back(construction_Point_2(208, 148));
   polygon_edges.push_back(kernel_Point_2(208,148));

   polygon.push_back(construction_Point_2(295, 160));
   polygon_edges.push_back(kernel_Point_2(295,160));

   polygon.push_back(construction_Point_2(421, 212));
   polygon_edges.push_back(kernel_Point_2(421,212));

   polygon.push_back(construction_Point_2(441, 303));
   polygon_edges.push_back(kernel_Point_2(441,303));
   // -------------------------------------------------
}

geometry_msgs::Point32 point_to_point_32(geometry_msgs::Point point) {

  geometry_msgs::Point32 point32;
  point32.x = point.x;
  point32.y = point.y;
  point32.z = point.z;

  return point32;
}

int main( int argc, char** argv )
{
  // --------- CGAL CODE ----------------------
  // create a vector of kernel_points that are the edges of the polygon
  std::vector<kernel_Point_2> polygon_edges;
  // create a constructions kernel polygon
  Polygon_2 cgal_polygon; 
  // make the polygon and populate the points vector also
  make_polygon(cgal_polygon, polygon_edges);

  // where the resulting partitioning will be stored
  Polygon_list partition_polys;
  // have no idea what traits are.. TODO
  Traits partition_traits;
  
  // greene_approx_convex_partition_2
  // optimal_convex_partition_2
  CGAL::optimal_convex_partition_2(cgal_polygon.vertices_begin(),
                                    cgal_polygon.vertices_end(),
                                    std::back_inserter(partition_polys),
                                    partition_traits);
  // endOf: --------- CGAL CODE ----------------------

  std::cout << "Main polygon is partitioned in " << partition_polys.size() << " partitions" << "\n";

  ros::init(argc, argv, "polygon_partition");
  ros::NodeHandle n;
  
  // advertise: topic, queue size
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Publisher polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("visualization_polygon", 10);

  ros::Publisher segmented_polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("segmented_polygon", 10);
  // refresh/publishing rate in Hz (times per second)
  ros::Rate r(30);

  while (ros::ok())
  {

    // create points collection of type visualization_msgs::Marker.
    visualization_msgs::Marker points;
    // create a Polygon (in order to publish it, you must also create a polygonStamped)
    // PolygonStamped has a polygon and a header as members
    geometry_msgs::Polygon polygon, segmented_polygon;
    geometry_msgs::PolygonStamped polygonStamped, segmented_polygon_stamped;

    // set the variable "frame_id" (which is the frame reference of the object) of the header 
    // of the points/polygonStamped to "my_frame"
    points.header.frame_id = polygonStamped.header.frame_id = segmented_polygon_stamped.header.frame_id = "/my_frame";
    // set the variable "stamp" (which refers to the time stamp) of the header of each
    // of the points/polygonStamped to the time that ROS has now
    points.header.stamp = polygonStamped.header.stamp = segmented_polygon_stamped.header.stamp = ros::Time::now();
    // set the namespace of each of the points/line_strips to "cgal_rviz_namespace" so to create a unique
    // ID (along with the Marker.id) of these markers. (any marker sent with the same namespace and id will overwrite the old one)
    points.ns = "cgal_rviz_namespace";
    // set these markers action. The options are ADD, DELETE, DELETEALL. Here we add them
    points.action = visualization_msgs::Marker::ADD;
    // set the initial orientation. x,y,z defaults to 0, set the w quaternion variable
    points.pose.orientation.w = 1.0;

    // set the id of the points collection
    points.id = 0;
 
    // set their type
    points.type = visualization_msgs::Marker::POINTS;
 
    // POINTS markers use x and y scale for width/height respectively (how big the point is)
    // 1 means 1 meter so 0.5 is 50cm
    points.scale.x = 5;
    points.scale.y = 5;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0f;

    // create an iterator to go through all edges of polygon and add them as points to rviz
    std::vector<kernel_Point_2>::iterator iterator;
    
    for(iterator=polygon_edges.begin(); iterator < polygon_edges.end(); iterator++ ){  
      geometry_msgs::Point new_point = cgal_point_to_ros_geometry_point(*iterator);    
      points.points.push_back(new_point);
    }
    // iterate again through the points in order to create the initial polygon in rviz
    for(iterator=polygon_edges.begin(); iterator < polygon_edges.end(); iterator++ ){
      polygon.points.push_back(point_to_point_32(cgal_point_to_ros_geometry_point(*iterator)));
    }

    polygonStamped.polygon = polygon;
    marker_pub.publish(points);
    polygon_pub.publish(polygonStamped);

    // iterate through the cgal produced polygons
    for (std::list<Polygon_2>::const_iterator cons_iterator = partition_polys.begin(), 
      end = partition_polys.end(); cons_iterator != end; ++cons_iterator) {      
      
      // this is just a tweak in order to show the produced polygons on the same topic 
      ros::Duration d = ros::Duration(1, 0);

      // the current polygon
      Polygon_2 current_polygon =  *cons_iterator; 

      // iterate through every edge of the polygon. The EdgeIterator has two traits: the starting edge point
      // and the ending edge point for every edge.
      for (EdgeIterator ei = current_polygon.edges_begin(); ei != current_polygon.edges_end(); ++ei){
        
        // we take the starting edge point (source), convert it from it's type to a ROS geometry Point,
        // then to a 32bit point(...) and then we add it to the points collection of the current segmented polygon.
        segmented_polygon.points.push_back(point_to_point_32(cgal_construction_point_to_ros_geometry_point(ei->source())));

       }

        segmented_polygon_stamped.polygon = segmented_polygon;
        // we publish each of the polygons
        segmented_polygon_pub.publish(segmented_polygon_stamped);
        // and sleep for 3 seconds in order to show it in rviz before clearing it.
        // it's not the best way I guess, but it's better than publishing in a different topic each new polygon.
        // either way, it's for demonstration purposes only.
        d.sleep();
        // we clear the polygon points
        segmented_polygon.points.clear();
    }


    r.sleep();
  }
}