/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/
/**
 * @file 
 * @brief header for a DDT example
 *@{
 */


/*CISC 829 Final Project
CISC829 Final project: Image reconstruction using Data-Dependent triangulation (DDT)
Spring 2008

The following is an implementation of the DDT algorithm presented in 
"Image Reconstruction Using Data-Dependent Triangulation,", Xiaohua Yu, Bryan S. Morse, Thomas W. Sederberg, IEEE Computer Graphics and Applications, vol. 21,  no. 3,  pp. 62-68,  May/Jun,  2001

Gowri Somanath
Feng Li
CIS
University of Delaware.
*/


// header for CGAL

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_euclidean_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Interpolation_traits_2.h>
#include <CGAL/natural_neighbor_coordinates_2.h>
#include <CGAL/interpolation_functions.h>

#include <CGAL/point_generators_2.h>
#include <CGAL/copy_n.h>
#include <CGAL/Origin.h>
#include <fstream>

struct K : CGAL::Exact_predicates_inexact_constructions_kernel {};
typedef K::FT                                         Coord_type;
typedef K::Vector_2                                   Vector;
typedef K::Point_2                                    Point;

//typedef CGAL::Triangulation_euclidean_traits_xy_3<K>  Gt;
//typedef CGAL::Delaunay_triangulation_2<Gt> Delaunay;
typedef CGAL::Delaunay_triangulation_2<K>             Delaunay;
typedef CGAL::Interpolation_traits_2<K>               Traits;

typedef Delaunay::Vertex_circulator		 Vertex_circulator;
typedef Delaunay::Edge_circulator		 Edge_circulator;
typedef Delaunay::Vertex                 Vertex;
typedef Delaunay::Face                   Face;
typedef Delaunay::Edge                   Edge;
typedef Delaunay::Vertex_handle          Vertex_handle;
typedef Delaunay::Face_handle            Face_handle;
typedef Delaunay::Face_circulator        Face_circulator;

typedef Delaunay::Locate_type			 Locate_type;
typedef Delaunay::Finite_edges_iterator  Edge_iterator;
typedef Delaunay::Vertex_iterator  Vertex_iterator;
typedef Delaunay::All_vertices_iterator  All_vertices_iterator;
typedef Delaunay::All_faces_iterator	 All_faces_iterator;

typedef std::vector< std::pair<Point, Coord_type> >   Coordinate_vector;
typedef std::map<Point, Coord_type, K::Less_xy_2>     Point_value_map;
typedef std::map<Point,  Vector, K::Less_xy_2 >       Point_vector_map;
typedef std::pair<Vertex_handle,Vertex_handle>	GEdge;

#include <iostream>
#include <algorithm>
#include <functional>
#include <vector>
typedef std::vector<GEdge> EdgeVector ;
typedef EdgeVector::iterator EdgeVectorIterator ;


#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2.h>
#include <iostream>

typedef CGAL::Cartesian<double> K1;
typedef K1::Point_2 GPoint;
typedef CGAL::Polygon_2<K1> Polygon_2;


bool getPlaneEqn(int x1,int y1,int z1,int x2,int y2,int z2,int x3,int y3,int z3,double *a,double *b);
double getCost(double a1,double b1,double a2,double b2);

bool processForCost(Face_handle f1,int i,Point_value_map values,Delaunay *dt);


#include "cv.h"
#include <highgui.h>
#include <assert.h>
