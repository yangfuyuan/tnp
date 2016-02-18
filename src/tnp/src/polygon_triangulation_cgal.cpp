#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

#include <CGAL/Simple_cartesian.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

#include "utilities.h"
#include "polygon_triangulation.h"

#include <iostream> 
#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>
#include <cassert>
#include <list>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

float calculate_distance(geometry_msgs::Point center1, geometry_msgs::Point center2){

  return hypot(abs(center1.x - center2.x),abs(center1.y - center2.y));
  
}
  
bool distance_comparison (const Distance_Entry& i, const Distance_Entry& j) { 
  if (i.first != j.first)
    return (i.second < j.second);   
}

bool depth_comparison  (const Distance_Entry& i, const Distance_Entry& j) {
    return (i.first->info().depth < j.first->info().depth);
}

void shortest_path_coverage(CDT& cdt, nav_msgs::Path& path, CDT::Face_handle& starter_face, int agent, int target_face_id){

  // TODO: missing initialization function
  CDT::Face_handle target_face;
  // TODO: missing initialization function
  geometry_msgs::Point target_face_center;
  int target_face_depth = 0;

  CDT::Face_handle& current_face = starter_face;
  // put it in the path
  path.poses.push_back(build_pose_stamped(face_to_center(cdt, current_face)));  

  Distance_Vector distance_vector;
  
  // get target face
  for(CDT::Finite_faces_iterator faces_iterator = cdt.finite_faces_begin(); faces_iterator != cdt.finite_faces_end(); ++faces_iterator){
    if (faces_iterator->info().id == target_face_id){
      target_face = faces_iterator;
      target_face_center = face_to_center(cdt, target_face);
      target_face_depth = target_face->info().depth;
      // if it happens our target to be at the borders, we temporaly change its depth
      if (target_face_depth == 300){
      	for (int i=0; i<3; i++){
      		if (target_face->neighbor(i)->is_in_domain()){
      			target_face_depth = target_face->neighbor(i)->info().depth;
      		}	
      	}
      }
      break;
    }
  }  

  float previous_distance = calculate_distance( (face_to_center(cdt, current_face)) , target_face_center);
  std::cout << "Initial distance from start: " << previous_distance << std::endl;
  int depth_runs = 1;

  do {

    depth_runs+=4;

    for(CDT::Finite_faces_iterator faces_iterator = cdt.finite_faces_begin(); faces_iterator != cdt.finite_faces_end(); ++faces_iterator){
      if ( (faces_iterator->info().agent_id == agent) && (faces_iterator->info().depth < depth_runs) ){
        
        float distance = calculate_distance(face_to_center(cdt, faces_iterator), target_face_center);
        Distance_Entry this_entry = std::make_pair(faces_iterator, distance);
        distance_vector.push_back(this_entry);           
      }
    }

    std::sort(distance_vector.begin(), distance_vector.end(), distance_comparison);
    // put the nearer to path
    path.poses.push_back(build_pose_stamped(face_to_center(cdt, distance_vector.front().first)));  
    distance_vector.clear();

 
  }while (depth_runs < target_face_depth); 

}

void complete_path_coverage(CDT& cdt, nav_msgs::Path& path, CDT::Face_handle& starter_face, int agent){

    CDT::Face_handle& starter_cell = starter_face;
    Face_Handle_Vector borders_vector;
    Distance_Vector borders_distance_vector;
    int current_depth = 999;
    int smallest_depth = 999;
    bool not_finished = true;
    bool initial = true;

    starter_cell->info().path_visited = true;
    path.poses.push_back(build_pose_stamped(face_to_center(cdt, starter_cell)));

    do {

        not_finished = false;

        // go through all triangles, get the starter cell and the borders vector
        for(CDT::Finite_faces_iterator faces_iterator = cdt.finite_faces_begin();
            faces_iterator != cdt.finite_faces_end(); ++faces_iterator){

            if ( (faces_iterator->info().coverage_depth >= current_depth)
                 && (faces_iterator->info().agent_id == agent)
                 && (!faces_iterator->info().is_path_visited())) {

                borders_vector.push_back(faces_iterator);
                not_finished = true;
            }

            if (initial){
                if (faces_iterator->info().coverage_depth < smallest_depth){
                    smallest_depth = faces_iterator->info().coverage_depth;
                }
            }
        }

        initial = false;

        if (!not_finished){
            current_depth = current_depth - 10;
        } else {

            // calculate the distance from all borders to the starter cell in order to choose the first border cell to visit
            for (std::vector<CDT::Face_handle>::iterator it = borders_vector.begin(); it != borders_vector.end(); it++){
                float this_distance = calculate_distance(face_to_center(cdt, *it),face_to_center(cdt, starter_cell));
                Distance_Entry this_entry = std::make_pair(*it, this_distance);
                borders_distance_vector.push_back(this_entry);
            }

            std::sort(borders_distance_vector.begin(), borders_distance_vector.end(), distance_comparison);
            // and this is the closest
            CDT::Face_handle& first_of_the_border = borders_distance_vector.front().first;

            // an o geitonas tou starter_cell, diladi toy proigoymenoy vimatos, pou einai pio konta
            // ston first of the border, den exei ton firstOfTHeBor ws geitona,
            // tote vale ayton ton geitona sto path, kanton visited an den einai,
            // valton ws starter cell kai epanelave

            path.poses.push_back(build_pose_stamped(face_to_center(cdt, first_of_the_border)));
            first_of_the_border->info().path_visited = true;
            starter_cell = first_of_the_border;
        }
        borders_vector.clear();
        borders_distance_vector.clear();
    } while (current_depth > smallest_depth);
    // TODO: prepei na to kanoyme na min pidaei...
}

// create the initial CGAL triangulation polygon
void make_triangulation(CDT& cdt, std::vector<kernel_Point_2>& polygon_edges){

// ------------------ JUST A SQUARE - NOTHING TO SEE HERE, MOVE ALONG ----------

//   Vertex_handle va = cdt.insert(triangulation_Point(10,10));
//   Vertex_handle vb = cdt.insert(triangulation_Point(350,10));
//   cdt.insert_constraint(va,vb);
//   polygon_edges.push_back(kernel_Point_2(10,10));
//   polygon_edges.push_back(kernel_Point_2(350,10));

//   va = vb;
//   vb = cdt.insert(triangulation_Point(350,350));
//   cdt.insert_constraint(va,vb);
//   polygon_edges.push_back(kernel_Point_2(350,350));

//   va = vb;
//   vb = cdt.insert(triangulation_Point(10,350));
//   cdt.insert_constraint(va,vb);
//   polygon_edges.push_back(kernel_Point_2(10,350));

//   va = vb;
//   vb = cdt.insert(triangulation_Point(10,10));
//   cdt.insert_constraint(va,vb);

// ---------------------------------------------------

  Vertex_handle va = cdt.insert(triangulation_Point(391,374));
  Vertex_handle vb = cdt.insert(triangulation_Point(240,431));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(391,374));
  polygon_edges.push_back(kernel_Point_2(240,431));

  va = vb;
  vb = cdt.insert(triangulation_Point(252,340));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(252,340));

  va = vb;
  vb = cdt.insert(triangulation_Point(374,320));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(374,320));

  va = vb;
  vb = cdt.insert(triangulation_Point(289,214));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(289,214));

  va = vb;
  vb = cdt.insert(triangulation_Point(134,390));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(134,390));

  va = vb;
  vb = cdt.insert(triangulation_Point(68,186));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(68,186));

  va = vb;
  vb = cdt.insert(triangulation_Point(154,259));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(154,259));

  va = vb;
  vb = cdt.insert(triangulation_Point(161,108));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(161,108));

  va = vb;
  vb = cdt.insert(triangulation_Point(335,108));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(335,108));


  va = vb;
  vb = cdt.insert(triangulation_Point(335,128));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(335,128));


  va = vb;
  vb = cdt.insert(triangulation_Point(208,148));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(208,148));

  va = vb;
  vb = cdt.insert(triangulation_Point(295,160));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(295,160));

  va = vb;
  vb = cdt.insert(triangulation_Point(421,212));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(421,212));

  va = vb;
  vb = cdt.insert(triangulation_Point(441,303));
  cdt.insert_constraint(va,vb);
  polygon_edges.push_back(kernel_Point_2(441,303));

  // connecting the last with the first
  va = vb;
  vb = cdt.insert(triangulation_Point(391,374));
  cdt.insert_constraint(va,vb);

  // this is a hole in the center...
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

double latitudeDisplacement (double initialLatitude, double meters){

    double newLatitude = 0.0;
    newLatitude = initialLatitude + (((meters/1000) / r_earth) * (180/PI));
    return newLatitude;

}

double longitudeDisplacement(double initialLongitude, double displacedLatitude, double meters){

    double newLongitude = 0.0;
    newLongitude = initialLongitude + ((((meters/1000) / r_earth) * (180/PI)) / cos(displacedLatitude * (PI/180)));
    return newLongitude;

}

int main( int argc, char** argv )
{
    // einai lathos. den mporeis na ksereis to simeio 0.0.
    // ksereis mono ta simeia toy sximatos
    // i arxiki topothesia toy 0.0
    double initialLatitude = 12.1;
    double initialLongitude = 12.1;

    // placeholders
    int which_agent = 1;
    int numberOfStep = 1;

    // ta, x kai y tou centroid
    double pointY = 12.2;
    double pointX = 14.4;

    // ta x,y kathe waypoint
    double newLatitude = latitudeDisplacement(initialLatitude, pointY);
    double newLongitude = longitudeDisplacement(initialLongitude, newLatitude, pointX);

    // kai kanta oti thes
    //---------------FILES----------------pane prin apo toys ipologismous//

    std::stringstream coordinates_filename;
    std::stringstream flightplan_filename;

    coordinates_filename << currentDateTime() << ":forAgent:" << which_agent << ":coordinates" << ".txt";
    flightplan_filename << currentDateTime() << ":forAgent:" << which_agent << ":wpPlan" << ".txt";
    const std::string& tmp1 = coordinates_filename.str();
    const std::string& tmp2 = flightplan_filename.str();
    // see http://stackoverflow.com/questions/1374468/stringstream-string-and-char-conversion-confusion
    const char* cstr1 = tmp1.c_str();
    const char* cstr2 = tmp2.c_str();
    std::ofstream fPlanDataCoordinates(cstr1);
    std::ofstream fWPPlan(cstr2);


    fPlanDataCoordinates << "'UAVID', 'WptID', 'Lat', 'Lon',"<< std::endl; // only at first line
    fPlanDataCoordinates << which_agent << ", ";
    fPlanDataCoordinates << numberOfStep /* or id of cell.. */ << ", ";
    fPlanDataCoordinates << std::fixed << std::setprecision(7) << newLatitude << ", ";
    fPlanDataCoordinates << std::fixed << std::setprecision(7) << newLongitude << std::endl;

    // the following is the initial take off position - this is the initial 0.0 of the agent but for
    // now not the initial position (the one where the triangulation is counted).
    // it's the 0.0. it needs to fly to it's initial position
    fWPPlan << "0\t1\t0\t16\t0\t0\t0\t0\t" << std::fixed << std::setprecision(7) << initialLatitude << "\t"
                << std::fixed <<  std::setprecision(7) << initialLongitude << "\t585\t1" << std::endl;

    // this is the initial (where the agent should start...):
    fWPPlan << "1\t0\t3\t22\t15\t0\t0\t0\t" << std::fixed << std::setprecision(7) << newLatitude << "\t"
            << std::fixed << std::setprecision(7) << newLongitude << "\t100\t1" << std::endl;
    // this is for every other spot:
    fWPPlan << numberOfStep << "\t0\t3\t16\t0\t0\t0\t0\t" << std::fixed << std::setprecision(7) << newLatitude << "\t"
            << std::fixed << std::setprecision(7) << newLongitude << "\t100\t1" << std::endl;
    // and this for landing:
    fWPPlan << numberOfStep << "\t0\t3\t21\t480\t0\t0\t25\t" << std::fixed << std::setprecision(7) << initialLatitude << "\t"
            << std::fixed << std::setprecision(7) << initialLongitude  << "\t580\t1" << std::endl;

    // otan ola exoun mpei
    fPlanDataCoordinates.close();
    fWPPlan.close();


  // --------- CGAL CODE ----------------------
  // create a vector of kernel_points that are the edges of the polygon
  std::vector<kernel_Point_2> polygon_edges;

  // create a triangulation polygon
  CDT cdt;
  
  // make the triangulation and populate the points vector also
  make_triangulation(cdt, polygon_edges);
  
  // insert a single point inside the polygon to be triangulated.
  // By doing so we actually define which domain we consider as of interest.
  cdt.insert(triangulation_Point(198, 198));
  std::cout << "Number of vertices before meshing and refining: " << cdt.number_of_vertices() << std::endl;  
  std::cout << "Meshing the triangulation with default criteria..."
            << std::endl;
  Mesher mesher(cdt);
  mesher.refine_mesh();
  std::cout << "Number of vertices after meshing: " << cdt.number_of_vertices() << std::endl;
  std::cout << "Meshing again with new criteria..." << std::endl;
  
  // 0.125 is the default shape bound. It corresponds to abound 20.6 degree.
  // 0.5 is the upper bound on the length of the longest edge.
  // See reference manual for Delaunay_mesh_size_traits_2<K>.
  mesher.set_criteria(Criteria(0.125, 20)); // was 0.5
  mesher.refine_mesh();
  std::cout << "Number of vertices after meshing and refining with new criteria: "
            << cdt.number_of_vertices() << std::endl;

  //Adding a seed, inside the hole that was defined in the creation of the triangulation constrains.
  std::list<triangulation_Point> list_of_seeds;
  list_of_seeds.push_back(triangulation_Point(210, 210));
  std::cout << "Refining and meshing the domain with a seed defining the hole..." << std::endl;
  CGAL::refine_Delaunay_mesh_2(cdt, list_of_seeds.begin(), list_of_seeds.end(), Criteria());
  std::cout << "Number of vertices after meshing and refining with Delaunay triangulation: " << cdt.number_of_vertices() << std::endl;

  // JUMP COST ALGORITHM ------------------------------------------
  bool neverInside = false;
  int repeatIterator = 0;
  int jumpsIterator = 1;
  
  //initialize function - all faces are not visited and don't have a number (except the nth and kth)
  // the nth and kth are semi-random faces that have the number 1. it's our starting points
  // this is for the complicated shape
  int nth = 70; //these three produce a problem of paths
  int kth = 45;
  int zth = 1;
  // 695 is a triangle near the top right corner. ideal for path planning
  
  // int nth = 60;
  // int kth = 61;
  // int zth = 68;
  int agent_id = 1;
  int jumps_ad = 1;
  // define how many agents we have...
  int agents_count=3;
  int initialize_iterator = 0;

  geometry_msgs::Point centerOfField;
  centerOfField.x = 180.0;
  centerOfField.y = 180.0;

  for(CDT::Finite_faces_iterator faces_iterator = cdt.finite_faces_begin(); faces_iterator != cdt.finite_faces_end(); ++faces_iterator){

    if (faces_iterator->is_in_domain()){   
      
      faces_iterator->info().initialize(initialize_iterator);
      initialize_iterator++;
      
      // do it for X agents
      // the result is not optimal for numerous reasons. we need to tweak the distance of the central point and get 
      // the best result by comparing the % of coverage for each agent. choose the most equilibral or the one that suits our needs.
      // if ( (agent_id<4) && (calculate_distance(face_to_center(cdt, faces_iterator), centerOfField) < 22.0) ){
      //   faces_iterator->info().numbered = true;
      //   faces_iterator->info().depth = 1;
      //   faces_iterator->info().agent_id = agent_id++;
      // }
      // this is for the complicated shape
      if ((nth == initialize_iterator) || (kth == initialize_iterator) || (zth == initialize_iterator)){
        faces_iterator->info().numbered = true;
        faces_iterator->info().depth = 1;
        faces_iterator->info().agent_id = agent_id++;
        for (int j=0; j<3; j++){
        	faces_iterator->neighbor(j)->info().jumps_agent_id = jumps_ad;
        	jumps_ad++;
        }
      }
    }else {
      faces_iterator->info().agent_id = -1;
    }
  }
  std::cout << "Total cells in triangulation : " << initialize_iterator << std::endl;

  std::cout << "-----Beginning jump cost------" << std::endl;
  do {
    jumpsIterator++; // including non domain triangles
    
    neverInside = true;
    CDT::Face_handle face;

    for(CDT::Finite_faces_iterator faces_iterator = cdt.finite_faces_begin(); faces_iterator != cdt.finite_faces_end(); ++faces_iterator){

      face = faces_iterator;

      // too many comparisons. we need to refactor the algorithm
      if ((face->is_in_domain()) && (face->info().has_number()) 
          && !(face->info().is_visited()) && !(face->info().depth == jumpsIterator)) {   
        
        neverInside = false;
        face->info().visited = true;

        for (int i=0; i<3; i++){

          if ((face->neighbor(i)->is_in_domain()) && !(face->neighbor(i)->info().has_number())) {

			  // assign jumpers id. we do this in order to see which growing function has managed to reach the end or target.
           	if (face->info().depth != 1){
			   face->neighbor(i)->info().jumps_agent_id = face->info().jumps_agent_id;
            }

            face->neighbor(i)->info().depth = jumpsIterator;
            // increase depth if one of the neighbors are not in domain. this is done to avoid borders (or not..)
            face->neighbor(i)->info().numbered = true;
            // agent id propagation
            face->neighbor(i)->info().agent_id = face->info().agent_id;
          }
          // if a neighbor is not in the domain then give it, (and maybe it's neighbors), great depth in order for
          // the uav to visit the borders last, or not at all
          //------//
          // for now and for coverage purposes, each border cell (to non domain but agent seperating also)
          // is getting a great depth
          else if (!(face->neighbor(i)->is_in_domain())){

            // all borders have a great coverage depth
              face->info().coverage_depth = 999;
              face->info().cover_depth = true;
            //face->info().depth = 300;
            // for (int j=0; j<3; j++){
            //   face->neighbor(j)->info().depth = 300;
            //   face->neighbor(j)->info().numbered = true;
            //   face->neighbor(j)->info().agent_id = face->info().agent_id;
            // }
          }  
        }
      }

      repeatIterator++;

    }
   } while (neverInside == false);

  std::cout << "Ended. Maximum Jumps: " << jumpsIterator << " . While loop repetitions: " << repeatIterator << "." << std::endl;

  // -------- END OF JUMP COST ALGORITHM -------------------//
  // -------- BEGIN OF COMPLETE COVERAGE (BORDER-TO-INNER) COST ALGORITHM -------------------//
  std::cout << "----Beginning complete coverage algorithm----" << std::endl;

    // go through all triangles to give border depth to the borders between agents
    for(CDT::Finite_faces_iterator faces_iterator = cdt.finite_faces_begin();
    faces_iterator != cdt.finite_faces_end(); ++faces_iterator){
        // initialize again the path visited attribute
        for (int i=0;i<3;i++){
            if ( (faces_iterator->neighbor(i)->info().agent_id != faces_iterator->info().agent_id) ){
              faces_iterator->info().coverage_depth = 999;
              faces_iterator->info().cover_depth = true;
            }
        }
    }

    int so_many = 0;
    int da_coverage_depth = 999;
    bool never_ever_again = true;
    do {
        da_coverage_depth = da_coverage_depth - 10;
        never_ever_again = true;
        for(CDT::Finite_faces_iterator faces_iterator = cdt.finite_faces_begin();
            faces_iterator != cdt.finite_faces_end(); ++faces_iterator){

            if((faces_iterator->info().has_coverage_depth()) && (faces_iterator->info().coverage_depth > da_coverage_depth) ){
                for (int i=0;i<3;i++){
                    if (!faces_iterator->neighbor(i)->info().has_coverage_depth()){
                        faces_iterator->neighbor(i)->info().coverage_depth = da_coverage_depth;
                        faces_iterator->neighbor(i)->info().cover_depth = true;
                        never_ever_again = false;
                        so_many++;
                    }
                }
            }

        }
    } while (!never_ever_again);
    std::cout << "Total internal cells: " << so_many << std::endl;

  // -------- END OF COMPLETE COVERAGE (BORDER-TO-INNER) COST ALGORITHM -------------------//
  // endOf: --------- CGAL CODE ----------------------

  ros::init(argc, argv, "polygon_triangulation");
  ros::NodeHandle n;
  
  // advertise: topic, queue size
  // initial polygon edges
  ros::Publisher edges_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  // initial polygon
  ros::Publisher polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("visualization_polygon", 10);
  // triangulation collection of triangles
  ros::Publisher triangulation_mesh_pub = n.advertise<visualization_msgs::Marker>("triangulation_mesh", 150);
  // collection of points in center of each triangle
  ros::Publisher center_pub = n.advertise<visualization_msgs::Marker>("center_points", 150);
  // collection of poses to produce a path for path planning
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_planning", 150);

  // create collections of type visualization_msgs::Marker.
  visualization_msgs::Marker edges, triangulation_mesh, center_points;
  
  nav_msgs::Path path;

  // create a Polygon (in order to publish it, you must also create a polygonStamped)
  // PolygonStamped has a polygon and a header as members
  geometry_msgs::Polygon polygon;
  geometry_msgs::PolygonStamped polygonStamped;

  // set the variable "frame_id" (which is the frame reference for the objects)
  edges.header.frame_id = polygonStamped.header.frame_id = triangulation_mesh.header.frame_id =
    center_points.header.frame_id = path.header.frame_id = "/my_frame";
  
  // set the variable "stamp" (which refers to the time stamp) of the header of each
  // of the objects to the time that ROS has now
  edges.header.stamp = polygonStamped.header.stamp = triangulation_mesh.header.stamp =
    center_points.header.stamp = path.header.stamp = ros::Time::now();
  
  // set the namespace of each of the objects to "cgal_rviz_namespace" so to create a unique
  // ID (along with the Marker.id) of these markers. (any marker sent with the same namespace and id will overwrite the old one)
  edges.ns = triangulation_mesh.ns = center_points.ns = "cgal_rviz_namespace";
  
  // set these markers action. The options are ADD, DELETE, DELETEALL. Here we add them
  edges.action = triangulation_mesh.action = center_points.action = visualization_msgs::Marker::ADD;
  
  // set the initial orientation. x,y,z defaults to 0, set the w quaternion variable
  edges.pose.orientation.w = triangulation_mesh.pose.orientation.w = center_points.pose.orientation.w = 1.0;

  // set the id of the collections
  edges.id = 0;
  triangulation_mesh.id = 1;
  center_points.id = 2;

  // set their type
  edges.type = center_points.type = visualization_msgs::Marker::POINTS;
  triangulation_mesh.type = visualization_msgs::Marker::TRIANGLE_LIST;

  // POINTS markers use x and y scale for width/height respectively (how big the point is)
  // 1 means 1 meter so 0.5 is 50cm
  edges.scale.x = 5;
  edges.scale.y = 5;

  center_points.scale.x = 2; 
  center_points.scale.y = 2;
  
  triangulation_mesh.scale.x = 1;
  triangulation_mesh.scale.y = 1;
  triangulation_mesh.scale.z = 1;

  // Edges are green, centers are red
  // !-- insert here your quote --! (smth like 'kill the routine, your spirit is unfed')
  edges.color.g = center_points.color.r = 1.0f;
  edges.color.a = center_points.color.a = 1.0f;

  triangulation_mesh.color.b = 1.0f;
  triangulation_mesh.color.a = 1.0f;

  // create an iterator to go through all edges of polygon and add them as points to rviz
  // also create the polygon
  std::vector<kernel_Point_2>::iterator iterator;

  for(iterator=polygon_edges.begin(); iterator < polygon_edges.end(); iterator++ ){

    geometry_msgs::Point new_point = cgal_point_to_ros_geometry_point(*iterator);    
    edges.points.push_back(new_point);
    polygon.points.push_back(point_to_point_32(cgal_point_to_ros_geometry_point(*iterator)));  
  }

  // publish the edges and the polygon itself
  polygonStamped.polygon = polygon;

  // define a "pausing" duration for demonstration purposes
  ros::Duration d = ros::Duration(1, 0);

  // the triangulation face iterator
  //faces_iterator = cdt.faces_begin();

  // a color iterator in order to color each triangle differently
  float color_iterator = 0.0;

  // declare here the agent for path planning
  int for_agent = 1;
  // 695 is a good target triangle for use with agent 1
  int target_face_number = 595;
  int target_jumps_agent_id = 0;
  
  // finding the jump agent id for the target
  for(CDT::Finite_faces_iterator faces_iterator = cdt.finite_faces_begin(); faces_iterator != cdt.finite_faces_end(); ++faces_iterator){

    CDT::Face_handle face = faces_iterator;
    
    if (face->info().id == target_face_number){
      target_jumps_agent_id = face->info().jumps_agent_id;
      break;
    }
  }

  // applying either shortest path to target or coverage algorithms for path production
  for(CDT::Finite_faces_iterator faces_iterator = cdt.finite_faces_begin(); faces_iterator != cdt.finite_faces_end(); ++faces_iterator){

    CDT::Face_handle face = faces_iterator;
    
    if ((face->is_in_domain()) && (face->info().agent_id == for_agent) && (face->info().depth == 1)){
      //TODO: introduce also not over holes in complete coverage-shortest distance
      complete_path_coverage(cdt, path, face, for_agent);
      //shortest_path_coverage(cdt, path, face, for_agent, target_face_number);
      break;
    }
  }

  for(CDT::Finite_faces_iterator faces_iterator = cdt.finite_faces_begin(); faces_iterator != cdt.finite_faces_end(); ++faces_iterator){
      
    color_iterator += 1.0;
    // for every face, we need a face handle to perform the various operations.
    CDT::Face_handle face = faces_iterator;

    // if this face is in the domain, meaning inside the contrained borders but outside the defined holes 
    if (face->is_in_domain()){  

      // !-- create 3 jumpers. each one has an orientation.
      // each one starts jumping around starting from a direction and adding weight.

      // create a point for each of the edges of the face.
      triangulation_Point point1 = cdt.triangle(face)[0];
      triangulation_Point point2 = cdt.triangle(face)[1];
      triangulation_Point point3 = cdt.triangle(face)[2];

      // add these edges to the triangulation mesh array. This collection creates a triangle for every three points inserted. 
      // add also the z for z elevation       

      int face_depth = face->info().depth;
      //int face_depth = face->info().coverage_depth;
      float z = -face_depth;
      int the_agent = face->info().agent_id;

      // from utilities.cpp
      triangulation_mesh.points.push_back(cgal_triangulation_point_to_ros_geometry_point(point1, z));
      triangulation_mesh.points.push_back(cgal_triangulation_point_to_ros_geometry_point(point2, z));
      triangulation_mesh.points.push_back(cgal_triangulation_point_to_ros_geometry_point(point3, z));

      // adding the center of every triangle
      center_points.points.push_back(face_points_to_center(point1, point2, point3));

      std_msgs::ColorRGBA triangle_color;
      triangle_color.r = 0.0f + (face_depth/300.0);
      triangle_color.b = 0.0f + (the_agent/10.0);// + (face->info().depth/45.0);//color_iterator*2.50/100;
      triangle_color.g = 0.0f;// + (face_depth/75.0);//color_iterator*8.0/100;
      triangle_color.a = 1.0f;// + (face_depth/900.0);

      // also: if (face->info().jumps_agent_id == target_jumps_agent_id)
      if ((face->info().depth == 1) || (face_depth == (jumpsIterator - 1)) ){ // || (face->info().id == 695)) {
        triangle_color.r = 1.0f;// + (face->info().depth/30.0);
        triangle_color.g = 1.0f;// + (face->info().depth/50.0);//color_iterator*2.50/100;
        triangle_color.b = 1.0f;// + (face->info().depth/60.0);//color_iterator*8.0/100;
        triangle_color.a = 1.0f;
      }

      triangulation_mesh.colors.push_back(triangle_color);
     // triangulation_mesh_pub.publish(triangulation_mesh);
    }
  }

  // refresh/publishing rate in Hz (times per second)
  ros::Rate r(1);

  while (ros::ok())
  {

    edges_pub.publish(edges);
    polygon_pub.publish(polygonStamped);
    triangulation_mesh_pub.publish(triangulation_mesh);
    center_pub.publish(center_points);
    path_pub.publish(path);

    r.sleep();
  }
}
