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
 * @brief This is an example I got somewhere. It does DDT.
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
#include "preh.h"
#include "cv.h"
#include "highgui.h"
#include <assert.h>

#include <GL/glut.h>


GLuint mode;
IplImage* grab = NULL;
IplImage* res = NULL;
Delaunay dt;	
IplImage* src = NULL;
int ratio = 1;

////////////////////////////////////////////////////////

//
// GLUT keypress function
// 
void onKeyPress(unsigned char key, int x, int y) {
 
	if ( key == 'w' ) { // quit the program.
        mode = GL_LINE_LOOP;
    }
    else if ( key == 'p' ) { // quit the program.
        mode = GL_POINTS;
    }
    else if ( key == 's' ) { // quit the program.
        glShadeModel(GL_SMOOTH);								// Set Smooth Shading 
        mode = GL_POLYGON;
    }
    else if ( key == 'f' ) { // quit the program.
        glShadeModel(GL_FLAT);								// Set Smooth Shading 
        mode = GL_POLYGON;
    }
    else if ( key == 'q' ) { // quit the program.
//        delete data;
		exit(0);
    }
	else if ( key == 'r' ) { // reset the data
//		delete data;
//		data = new WavefrontObj( data_name );
	}
	else if (key == 'i') {
		//Subdivision_method_3::Loop_subdivision(P,1);
	}
	else if(key == 'd') {
	
	}
	else if(key == 'g') {
	
		glReadPixels(0, 0, grab->width, grab->height, 
			GL_RGB, GL_UNSIGNED_BYTE, grab->imageData);
		cvSaveImage("res.bmp", grab);
	}

    glutPostRedisplay();
}


void display() 
{
	double x = 0, y = 0, color = 0;

	// draw 
	glClear (GL_COLOR_BUFFER_BIT);

	All_faces_iterator  fai;
	int index = 0;
	Vertex_handle  vh; //f.vertex ( int i)  
	for(fai = dt.all_faces_begin(); fai != dt.all_faces_end(); fai++) {	
		if(dt.is_infinite(fai))
			continue;
		     
		glBegin(mode);
		for(int k = 0; k < 3; k ++) {
			Point pt = fai->vertex(k)->point();
			color = ((uchar*)(src->imageData + 
				int(pt.y()/ratio) * src->widthStep))[int(pt.x()/ratio)] / 255.0f;
			if(mode == GL_LINE_LOOP) 
				glColor3f(0.5, 0.5, 1);
			else
				glColor3f(color, color, color);
			x = pt.x();
			y = pt.y();
			glVertex2d(x, y);
		}
		glEnd();

	}

	glFlush();	
	glutSwapBuffers();
}


void init (void) 
{
   glClearColor (0.0, 0.0, 0.0, 0.0);
   glShadeModel (GL_FLAT);    
}


void reshape(int w, int h)
{
   glViewport(0, 0, (GLsizei) w, (GLsizei) h);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluOrtho2D(0.0, (GLdouble) w, 0.0, (GLdouble) h);
}




/////////////////////////////////////////////////////////

// fill the geometry trait using the src image 
//int wraper(Delaunay* dt, Point_value_map* values, IplImage* src, int ratio)
int wraper( Point_value_map* values, IplImage* src, int ratio)
{
	assert(src->nChannels == 1);
	int tmp = 0;
	Point pt;

	for(int j = 0; j < src->height; j ++) {
		for(int i = 0; i < src->width; i ++) {
			tmp = int(((uchar*)(src->imageData + j * src->widthStep))[i]);
			pt = Point(ratio * i, ratio * j);
			dt.insert(pt);
			values->insert(std::make_pair(pt, tmp));
		}
	}

	// some statistic info
	std::cout << "width, height of source img: " << src->width << ", " << src->height << std::endl;
	std::cout << "upsample ratio: " << ratio << std::endl;
	std::cout << "# of vertices: " << dt.number_of_vertices() << std::endl;

	return 0;
}

int interpolation_gl()
{
		// opengl drawing
    int w = res->width; 
	int h = res->height;

	int argcg = 1;
	char* argvg[] = {"interpolation"};
    glutInit(&argcg, argvg);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(w,h);
    glutInitWindowPosition(100,100);
    glutCreateWindow("Interpolation");
	//grab = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);

	init();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(onKeyPress);
	 glShadeModel(GL_SMOOTH);								// Set Smooth Shading 
       mode = GL_POLYGON;
	glutMainLoop();

	return 0;
}


// data dependent triangulation
IplImage* ddt(IplImage* src, int ratio) 
{
	assert(src->nChannels == 1);

	Point_value_map values;
	wraper(&values, src, ratio);	// generate the initial triangulation
	std::cout << "Done Delaunay " << std::endl;
	char n[15];

for(int itr=0;itr<4;itr++)
{
	std::cout<<"Iteration "<<itr<<std::endl;
	EdgeVector edges(0);
	//edges=(EdgeVector *)malloc(sizeof(Edge)*v);
	EdgeVectorIterator evi;
	Edge_iterator ei=dt.finite_edges_begin();
	//make_heap(edges.begin(), edges.end()) ;
	do
	{	

		//edges[i++]=Edge(ei->first,ei->second);
		Face_handle f=ei->first;
		if(dt.is_infinite(f))
			continue;

		int i=ei->second;
		if(!dt.is_edge(f->vertex(f->cw(i)),f->vertex(f->ccw(i))))
			continue;

		if(dt.is_infinite(f->vertex(f->cw(i))))
			continue;
		if(dt.is_infinite(f->vertex(f->ccw(i))))
			continue;
		edges.push_back(GEdge(f->vertex(f->cw(i)),f->vertex(f->ccw(i))));
		push_heap(edges.begin(), edges.end()) ;
/////		break;
	}while(++ei!=dt.finite_edges_end());
	//make_heap(edges.begin(), edges.end()) ;
//	push_heap(edges.begin(), edges.end()) ;
	//sort_heap(edges.begin(), edges.end()) ;
	
	int cnt=0;
	for(evi=edges.begin();evi!=edges.end();evi++)
	{
		cnt++;
		printf("edge %d\n",cnt);
		
		Vertex_handle v1=evi->first;
		
		Vertex_handle v2=evi->second;
		
		pop_heap(edges.begin(), edges.end()) ;
		
		Face_handle f1;
		
		int i,j;
		if(!v1->is_valid() || !v2->is_valid() || dt.is_infinite(v1) || dt.is_infinite(v2))
			continue;
		//std::cout << "--- " <<cnt << std::endl;
		//std::cout << "v1:" <<v1->point()<<"  v2:" <<v2->point()<< std::endl;	
		
		if(!dt.is_edge(v1,v2))
			continue;
		
		if(dt.is_edge ( v1, v2,f1,i))
		 {
		
			 if(!dt.is_infinite(f1))
			 {
		
				Face_handle f2;
				if(dt.is_edge(v2,v1,f2,j) && !dt.is_infinite(f2) && f1->has_neighbor(f2))
				{
		
					GPoint pts[]={GPoint(v1->point().x(),v1->point().y()),GPoint(f1->vertex(i)->point().x(),f1->vertex(i)->point().y()),GPoint(v2->point().x(),v2->point().y()),GPoint(f2->vertex(j)->point().x(),f2->vertex(j)->point().y())};
		
					Polygon_2 pgn(pts, pts+4);
		
					if(pgn.is_convex())
					{
		
							processForCost(f1,i,values,&dt);
		
					}
				}
			}
		 }
		//std::cout << "--- " << std::endl;
		 //else
		//	std::cout << "rubbish in the way " << std::endl;
			 //processForCost(Face_handle f1,int i,Point_value_map values,Delaunay *dt);
		//processForCost(f1,i,values,&dt);
	}
	

}

	std::cout << "Done DDT " << std::endl;

	// interpolation
	res = cvCreateImage(cvSize((src->width - 1) * ratio + 1, 
		(src->height - 1) * ratio + 1), IPL_DEPTH_8U, 1);
	grab = cvCreateImage(cvGetSize(res), IPL_DEPTH_8U, 3);
	cvZero(res);
	//interpolation(&dt, &values, res);
	interpolation_gl();
	std::cout << "Done interpolation " << std::endl;

	return res;
	//return NULL;
}


int main(int argc, char** argv)
{
	char* filename = NULL;
	ratio = 1;
	
	if(argc > 2) {
		filename = argv[1];
		ratio = atoi(argv[2]);
	}
	else {
		filename = "test.bmp";
		ratio = 4;
	}

	// load image
	src = cvLoadImage(filename, 0);		// load the image as gray image
	

	IplImage* res = ddt(src, ratio);
	cvSaveImage("ddt.bmp", res);


	return 0;
}
