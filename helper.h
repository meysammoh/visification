#ifndef _HELPER_H
#define _HELPER_H
#include <GL/glut.h>
#include <iostream>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Simple_polygon_visibility_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_naive_point_location.h>
#include <istream>
#include <vector>
#include <algorithm>
#include <CGAL/Arr_extended_dcel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/intersections.h>
#include <CGAL/Sweep_line_2_algorithms.h>
#include <CGAL/Aff_transformation_2.h>

class Robot;
class GraphNode;
class Graph;
class State;
class EventPointBase;
class EventPoint;
class RobotEventPoint;
struct ReflectivePoint;
struct ReflectiveToPathMapPoint;

struct COLOR
{
    GLint r,g,b;
};

struct VertexData
{
    bool mIsOuter;

};

struct EdgeData
{
    bool mIsOuter = false;
    std::vector<Robot>::iterator mOwner;
};

struct CellData
{

};

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

typedef CGAL::Exact_predicates_exact_constructions_kernel               Kernel;
typedef Kernel::Point_2                                                 Point_2;
typedef CGAL::Vector_2<Kernel>                                          Vector_2;
typedef CGAL::Polygon_2<Kernel>                                         Polygon_2;
typedef CGAL::Line_2<Kernel>                                         Line_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                              Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>                                 Pwh_list_2;
typedef Kernel::Segment_2                                               Segment_2;
typedef CGAL::Arr_segment_traits_2<Kernel>                              Traits_2;
//typedef CGAL::Arrangement_2<Traits_2>                                   Arrangement_2;
typedef CGAL::Arr_extended_dcel<Traits_2,
VertexData /*data for vertex*/ ,
EdgeData /*data for edge*/,
CellData /*data for cell*/>                                         Dcel;
typedef CGAL::Arrangement_2<Traits_2, Dcel>                             Arrangement_2;
typedef Arrangement_2::Face_handle                                      Face_handle;
typedef Arrangement_2::Edge_const_iterator                              Edge_const_iterator;
typedef Arrangement_2::Vertex_const_iterator                            Vertex_const_iterator;
typedef Arrangement_2::Edge_iterator                                    Edge_iterator;
typedef Arrangement_2::Ccb_halfedge_circulator                          Ccb_halfedge_circulator;
typedef CGAL::Aff_transformation_2<Kernel> Transformation;


#include "Graph.h"
#include "EventPoint.h"
#include "Robot.h"
#include "State.h"

void DrawCircle(float cx, float cy, float r, int num_segments);
double AngleBetweenThreePoints( Point_2 p1, Point_2 middlePoint, Point_2 p3 );
bool IsPointInsideSemiCircle( double radius, double centerX,double centerY,
                              double startAngle , double endAngle ,
                              double pointx  , double  pointy  );
double Cross( Vector_2 vec1, Vector_2 vec2);
void MakeTriangleCCW(Point_2 &p1, const Point_2& middlePoint, Point_2 &secondEndPoint );
enum IntersectionType
{
    IT_NONE,
    IT_POINT,
    IT_SEGMENT
};
enum PointPlacementType
{
    PPT_BEFORE_FIRST,
    PPT_BETWEEN,
    PPT_AFTER_SECOND,
    PPT_OUT
};

class IncrementingSequence
{
public:
    // Constructor, just set counter to 0
    IncrementingSequence() : i_(0) {}
    // Return an incrementing number
    int operator() () { return i_++; }
private:
    int i_;
};

IntersectionType FindIntersection(Segment_2 first, Segment_2 second, CGAL::Object& intersection );
Point_2 ExtendSegmentFromEnd( Point_2 segmentStart, Point_2 segmentEnd, double toSize, bool inOppositeDirection);
PointPlacementType CheckPointPlacement( Point_2 checkingPoint, Point_2 firstPoint, Point_2 secondPoint );
#endif
