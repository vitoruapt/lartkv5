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
 * @addtogroup polygon_simplification
 * @{
 * @file polygon_simplification.cpp
 * @brief Defines several methods for simplifying polygons. The reason is that
 * for polygons to be processes using CGAL methods, they must be simple
 */

#ifndef _POLYGON_SIMPLIFICATION_CPP_
#define _POLYGON_SIMPLIFICATION_CPP_

#include <bo_polygon2d/polygon_simplification.h>


class_polygon_simplification::class_polygon_simplification(void){};
class_polygon_simplification::~class_polygon_simplification(void){};

int class_polygon_simplification::simplify_polygon(PS_CGALPolygon_2* p)
{

	if(!p->is_simple())
	{

		int keep_trying=1;
		int num_attempts=0;

		while(keep_trying)
		{

			int tt=0;
			double prev_x=-999999, prev_y=-999999;
			ROS_WARN("polygon_simplification: polygon is not simple. Printing all vertexes");

			int tttt=0;
			for (PS_CGALPolygon_2::Vertex_const_iterator j = p->vertices_begin(); j != p->vertices_end(); ++ j )
			{
				//ROS_ERROR("pt%d x=%f y=%f",tttt,j->x(), j->y());	
				tttt++;
			}


			PS_CGALPolygon_2::Vertex_const_iterator begin = p->vertices_begin();
			PS_CGALPolygon_2::Vertex_const_iterator end = --p->vertices_end();
			if ( begin->x() == end->x() && begin->y() == end->y())
			{
				ROS_WARN("Final and start point are equal. Erasing final point");
				p->erase(end);	
			}	


			for (PS_CGALPolygon_2::Vertex_const_iterator j = p->vertices_begin(); j != p->vertices_end(); ++ j )
			{
				if(tt!=0)
				{
					if (prev_x==j->x() && prev_y==j->y())
					{
						ROS_INFO("Erasing vertex %d",tt);
						p->erase(j);	
						break;
					}
					else
					{
						prev_x = CGAL::to_double(j->x());	
						prev_y = CGAL::to_double(j->y());	
					}

				}
				else
				{
					prev_x = CGAL::to_double(j->x());	
					prev_y = CGAL::to_double(j->y());	
				}


				//ROS_ERROR("pt%d x=%f y=%f",tt,j->x(), j->y());	
				tt++;
			}


			ROS_WARN("offset polygon %d: result of fix: %d",(int)p->size(), p->is_simple());

			int ttt=0;
			for (PS_CGALPolygon_2::Vertex_const_iterator j = p->vertices_begin(); j != p->vertices_end(); ++ j )
			{
				//ROS_ERROR("pt%d x=%f y=%f",ttt,j->x(), j->y());	
				ttt++;
			}

			if (p->is_simple()) keep_trying=0;

			num_attempts++;
			if (num_attempts>50)
			{
				//ROS_ERROR("Could not add polygon. Returning error");
				return 0;
			}
		}
	}

	if(!p->is_counterclockwise_oriented()) //check if its counterclockwise. If not reverse
	{
		ROS_INFO("Offset Polygon: ch  (%d points) is not counterclockwise oriented. Reversing orientation",(int)p->size());
		p->reverse_orientation();	
		ROS_INFO("Result Polygon: ch  (%d points)",(int)p->size());
	}


	ROS_INFO("Removing collinear points... %d", (int)p->size());

	int keep_going=true;

	while (keep_going)
	{
		keep_going=false;
		PS_CGALPolygon_2::Vertex_const_iterator j = p->vertices_begin();
		double x2 = CGAL::to_double((*j).x());
		double y2 = CGAL::to_double((*j).y());

		j++;
		double x1 = CGAL::to_double((*j).x());
		double y1 = CGAL::to_double((*j).y());
		j++;

		for (; j != p->vertices_end(); ++ j )
		{
			double m1 = (y2-y1)/(x2-x1);
			double m2 = ( CGAL::to_double((*j).y()) - y1)/( CGAL::to_double((*j).x())-x1);

			if (fabs(m1-m2)<0.01)
			{
				j--;
				p->erase(j);

				keep_going=true;	
				break;
			}

			x2=x1;
			y2=y1;
			x1= CGAL::to_double((*j).x());
			y1= CGAL::to_double((*j).y());
		}
	}

	ROS_INFO("End removing collinear points... %d", (int)p->size());
	return 1;
}

#endif
/**
 *@}
 */      
