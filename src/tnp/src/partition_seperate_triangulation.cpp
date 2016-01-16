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

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

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

// Triangulation specific data types
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;

typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
typedef CGAL::Delaunay_mesher_2<CDT, Criteria> Mesher;

// The triangulation points
typedef CDT::Vertex_handle Vertex_handle;
typedef CDT::Point triangulation_Point;


void triangulate (CDT& cdt, std::vector<construction_Point_2>& current_polygon_edges) {

  Vertex_handle va;
  Vertex_handle vb;

  for (std::vector<construction_Point_2>::iterator it = current_polygon_edges.begin(); it != current_polygon_edges.end(); ++it){ 
    
    va = cdt.insert(triangulation_Point(it->x(),it->y()));
    if ((it+1) != current_polygon_edges.end()){
      vb = cdt.insert(triangulation_Point((it+1)->x(),(it+1)->y()));
    } else {
      vb = cdt.insert(triangulation_Point(current_polygon_edges.begin()->x(), current_polygon_edges.begin()->y()));
    }
    cdt.insert_constraint(va,vb);

    // insert a point inside the domain
    cdt.insert(triangulation_Point(it->x(),it->y()));
  }

    //uncomment to set a seed later on
    va = cdt.insert(triangulation_Point(200,200));
    vb = cdt.insert(triangulation_Point(240,200));
    cdt.insert_constraint(va,vb);
    va = vb;
    vb = cdt.insert(triangulation_Point(240,240));
    cdt.insert_constraint(va,vb);
    va = vb;
    vb = cdt.insert(triangulation_Point(200,240));
    cdt.insert_constraint(va,vb);
    va = vb;
    vb = cdt.insert(triangulation_Point(200,200));
    cdt.insert_constraint(va,vb);

}

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

geometry_msgs::Point cgal_triangulation_point_to_ros_geometry_point(triangulation_Point cgal_point) {

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
  
  ros::init(argc, argv, "polygon_partition");
  ros::NodeHandle n;
  
  // advertise: topic, queue size
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Publisher polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("visualization_polygon", 10);

  ros::Publisher segmented_polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("segmented_polygon", 10);
  
  ros::Publisher triangulation_mesh_pub = n.advertise<visualization_msgs::Marker>("triangulation_mesh", 100);

  ros::Publisher center_pub = n.advertise<visualization_msgs::Marker>("center_points", 100);

  // refresh/publishing rate in Hz (times per second)
  ros::Rate r(1);

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

  float area_sum = 0.0f;


  // while (ros::ok())
  // {
    area_sum = 0.0f;

    // create points collection of type visualization_msgs::Marker.
    visualization_msgs::Marker points, triangulation_mesh, center_points;

    // create a Polygon (in order to publish it, you must also create a polygonStamped)
    // PolygonStamped has a polygon and a header as members
    geometry_msgs::Polygon polygon, segmented_polygon;
    geometry_msgs::PolygonStamped polygonStamped, segmented_polygon_stamped;


    // set the variable "frame_id" (which is the frame reference of the object) of the header 
    // of the points/polygonStamped to "my_frame"
    points.header.frame_id = polygonStamped.header.frame_id = center_points.header.frame_id =
    segmented_polygon_stamped.header.frame_id = triangulation_mesh.header.frame_id = "/my_frame";

    // set the variable "stamp" (which refers to the time stamp) of the header of each
    // of the points/polygonStamped to the time that ROS has now
    points.header.stamp = polygonStamped.header.stamp = segmented_polygon_stamped.header.stamp = 
    triangulation_mesh.header.stamp =  center_points.header.stamp = ros::Time::now();

    // set the namespace of each of the points/line_strips to "cgal_rviz_namespace" so to create a unique
    // ID (along with the Marker.id) of these markers. (any marker sent with the same namespace and id will overwrite the old one)
    points.ns = triangulation_mesh.ns = center_points.ns = "cgal_rviz_namespace";

    // set these markers action. The options are ADD, DELETE, DELETEALL. Here we add them
    points.action = triangulation_mesh.action = center_points.action = visualization_msgs::Marker::ADD;

    // set the initial orientation. x,y,z defaults to 0, set the w quaternion variable
    points.pose.orientation.w = triangulation_mesh.pose.orientation.w = center_points.pose.orientation.w = 1.0;

    // set the id of the points collection
    points.id = 0;
    triangulation_mesh.id = 1;
    center_points.id = 2;

    // set their type
    points.type = center_points.type = visualization_msgs::Marker::POINTS;
    triangulation_mesh.type = visualization_msgs::Marker::TRIANGLE_LIST;
 
    // POINTS markers use x and y scale for width/height respectively (how big the point is)
    // 1 means 1 meter so 0.5 is 50cm
    points.scale.x = 5;
    points.scale.y = 5;

    center_points.scale.x = 2; 
    center_points.scale.y = 2;

    triangulation_mesh.scale.x = 1;
    triangulation_mesh.scale.y = 1;
    triangulation_mesh.scale.z = 1;
 
    // Points are green
    points.color.g = center_points.color.r = 1.0f;
    points.color.a = center_points.color.a = 1.0f;

    triangulation_mesh.color.b = 0.5f;
    triangulation_mesh.color.a = 1.0f;

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
    // marker_pub.publish(points);
    // polygon_pub.publish(polygonStamped);

    int iterations = 0;

    // iterate through the cgal produced polygons
    for (std::list<Polygon_2>::const_iterator cons_iterator = partition_polys.begin(), 
      end = partition_polys.end(); cons_iterator != end; ++cons_iterator) {      
      
      iterations++;
      // this is just a tweak in order to show the produced polygons on the same topic 
      ros::Duration d = ros::Duration(1, 0);

      // the current polygon
      Polygon_2 current_polygon =  *cons_iterator; 
      // the area
      std::cout << "Polygon " << iterations << " area: " << current_polygon.area() << std::endl;
      
      
      std::vector<construction_Point_2> current_polygon_edges;

      // iterate through every edge of the polygon. The EdgeIterator has two traits: the starting edge point
      // and the ending edge point for every edge.
      for (EdgeIterator ei = current_polygon.edges_begin(); ei != current_polygon.edges_end(); ++ei){
        
        // we take the starting edge point (source), convert it from it's type to a ROS geometry Point,
        // then to a 32bit point(...) and then we add it to the points collection of the current segmented polygon.
        segmented_polygon.points.push_back(point_to_point_32(cgal_construction_point_to_ros_geometry_point(ei->source())));
        
        // NEW: here create a vector that has all the edges of the current polygon.
        current_polygon_edges.push_back(ei->source());

       }

       // NEW : and here, call the make_triangulation function, by adding the edges created vector

        // create a triangulation polygon -> destroy later
        CDT cdt;
        // make the triangulation and populate the points vector also
        triangulate(cdt, current_polygon_edges);

        Mesher mesher(cdt);
        mesher.refine_mesh();
        std::cout << "Partitioned polygon no: " << iterations << ". Number of vertices: " << cdt.number_of_vertices() << std::endl;
        std::cout << "Meshing with new criterias..." << std::endl;

        // 0.125 is the default shape bound. It corresponds to abound 20.6 degree.
        // 0.5 is the upper bound on the length of the longuest edge.
        // See reference manual for Delaunay_mesh_size_traits_2<K>.
        mesher.set_criteria(Criteria(0.125, 20)); // was 0.5
        mesher.refine_mesh();
        std::cout << "Partitioned polygon no: " << iterations << ". NEW number of vertices: " << cdt.number_of_vertices() << std::endl;

        // Adding a seed, inside the hole that was defined in the creation of the triangulation constrains.
        std::list<triangulation_Point> list_of_seeds;
        list_of_seeds.push_back(triangulation_Point(210, 210));
        //list_of_seeds.push_back(triangulation_Point(190, 190));
        std::cout << "Meshing the domain with a seed defining the hole..." << std::endl;
        CGAL::refine_Delaunay_mesh_2(cdt, list_of_seeds.begin(), list_of_seeds.end(),
                                     Criteria());
  
        segmented_polygon_stamped.polygon = segmented_polygon;
        // we publish each of the polygons
        //segmented_polygon_pub.publish(segmented_polygon_stamped);
        // and sleep for 3 seconds in order to show it in rviz before clearing it.
        // it's not the best way I guess, but it's better than publishing in a different topic each new polygon.
        // either way, it's for demonstration purposes only.

        CDT::Finite_faces_iterator faces_iterator = cdt.faces_begin();
          
          float color_iterator = 0.0;

          for(faces_iterator = cdt.finite_faces_begin(); faces_iterator != cdt.finite_faces_end(); ++faces_iterator){
            
            color_iterator++;

            CDT::Face_handle face = faces_iterator;
            
            // if the face is in the domain. we exclude the holes and all other constrains.
            if (face->is_in_domain()){ 
            
            // TODO: add each triangle surface to a total. after this sum, we should begin adding each triangle in 
              // order to reach to a desired surface coverage.
              //std::cout << cdt.triangle(face).area() << std::endl;
                area_sum += cdt.triangle(face).area();
                
               // CDT::Face_handle neighbor_face = face->neighbor(0);
               // cdt.triangle(neighbor_face).area();
               // std::cout << "current: " << cdt.triangle(face).area() << " and neighbor :" 
               //    << cdt.triangle(neighbor_face).area() << " areas. " << std::endl; 

            // create a visualization of a mesh which includes all the produced triangles.
            triangulation_mesh.points.push_back(cgal_triangulation_point_to_ros_geometry_point(cdt.triangle(face)[2]));  
            triangulation_mesh.points.push_back(cgal_triangulation_point_to_ros_geometry_point(cdt.triangle(face)[1]));
            triangulation_mesh.points.push_back(cgal_triangulation_point_to_ros_geometry_point(cdt.triangle(face)[0]));
            
            // creating the points of each triangle
            triangulation_Point point1 = cdt.triangle(face)[0];
            triangulation_Point point2 = cdt.triangle(face)[1];
            triangulation_Point point3 = cdt.triangle(face)[2];
            
            // calculate the center of the triangle in order to visualize it and later on to calculate the distances between each center.
            geometry_msgs::Point center;
            center.x =  ( (point1.x() + point2.x() + point3.x()) / 3);
            center.y =  ( (point1.y() + point2.y() + point3.y()) / 3);
            center.z = 0;
            center_points.points.push_back(center);

            // changing the color of each triangle in order to have visual distinction.
            std_msgs::ColorRGBA triangle_color;
            triangle_color.r = 0.0f;
            triangle_color.g = color_iterator*2.50/100;
            triangle_color.b = color_iterator*8.0/100;
            triangle_color.a = 1.0f;

            // adding the color to the color array
            triangulation_mesh.colors.push_back(triangle_color);
            
            // // publishing the triangulation mesh
            // triangulation_mesh_pub.publish(triangulation_mesh);
            //triangulation_mesh.points.clear();

            }
            //uncomment to see each triangulation
           //d.sleep();
          // we clear the polygon points
          segmented_polygon.points.clear();
        }

        //uncomment to see each polygon partition and directly the triangulation
        //d.sleep();
      }

  while (ros::ok())
  {

    marker_pub.publish(points);
    polygon_pub.publish(polygonStamped);
    // we publish each of the polygons
    segmented_polygon_pub.publish(segmented_polygon_stamped);
    // publishing the triangulation mesh
    triangulation_mesh_pub.publish(triangulation_mesh);
    center_pub.publish(center_points);
    r.sleep();

  }
}