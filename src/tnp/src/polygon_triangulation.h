#ifndef POLYGON_TRIANGULATION
#define POLYGON_TRIANGULATION

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

// the cartesian Kernel
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 kernel_Point_2;
// the constructions Kernel
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K,Fb>        CTFb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K,CTFb>    InfoFbb;
typedef CGAL::Triangulation_data_structure_2<Vb, InfoFbb> Tds;
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds, Itag> CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
typedef CGAL::Delaunay_mesher_2<CDT, Criteria> Mesher;
// The triangulation points
typedef CDT::Vertex_handle Vertex_handle;
typedef CDT::Point triangulation_Point;
// custom types
typedef std::pair<CDT::Face_handle ,float> Distance_Entry;
typedef std::vector<Distance_Entry> Distance_Vector;
typedef std::vector<CDT::Face_handle> Face_Handle_Vector;
typedef std::vector<int> Path_visited_faces_by_id;

float calculate_distance(geometry_msgs::Point center1, geometry_msgs::Point center2);
bool distance_comparison (const Distance_Entry& i, const Distance_Entry& j);
bool depth_comparison  (const Distance_Entry& i, const Distance_Entry& j);
void shortest_path_coverage(CDT& cdt, nav_msgs::Path& path, CDT::Face_handle& starter_face, int agent, int target_face_id);
void complete_path_coverage(CDT& cdt, nav_msgs::Path& path, CDT::Face_handle& starter_face, int agent);
// create the initial CGAL triangulation polygon
void make_triangulation(CDT& cdt, std::vector<kernel_Point_2>& polygon_edges);
double latitudeDisplacement(double initialLatitude, double meters);
double longitudeDisplacement(double initialLongitude, double displacedLatitude, double meters);
int main( int argc, char** argv );


#endif // POLYGON_TRIANGULATION
