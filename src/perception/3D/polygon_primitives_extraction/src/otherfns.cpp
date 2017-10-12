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
 * @brief auxiliary functions for Data dependent triangulation. Does not work
 * well
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


#define PFLN {printf("DEBUG PRINT FILE %s LINE %d\n",__FILE__,__LINE__);}
#include<math.h>
#include "preh.h"
Delaunay dttmp;
bool getPlaneEqn(int x0,int y0,int z0,int x1,int y1,int z1,int x2,int y2,int z2,double *a,double *b)
{
	/*double A = y1*(z2 - z3) + y2*(z3 - z1) + y3*(z1 - z2);
	double B = z1*(x2 - x3) + z2*(x3 - x1) + z3*(x1 - x2);
	double C = x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2);
if 
	*a=-A/C;
	*b=-B/C;*/
	if(x0==x1)
	{
		if(x0==x2)
		{
			*a=99999;
			*b=99999;
			return false;
		}
	}
	if(y0==y1)
	{
		if(y0==y2)
		{
			*a=99999;
			*b=99999;
			return false;
		}
	}
double d0[3],d1[3],n[3],s,c,d,a1,b1;
	d0[0] = x1 - x0;
	d0[1] = y1 - y0;
	d0[2] = z1 - z0;
	d1[0] = x2 - x0;
	d1[1] = y2 - y0;
	d1[2] = z2 - z0;
	n[0] = d1[1] * d0[2] - d1[2] * d0[1];
	n[1] = d1[2] * d0[0] - d1[0] * d0[2];
	n[2] = d1[0] * d0[1] - d1[1] * d0[0];
	if((n[0] * n[0] + n[1] * n[1] + n[2] * n[2] )==0) //collinear traingles
	{
		*a=99999;
		*b=99999;
		return false;
	}
	s = 1.0f / sqrt( n[0] * n[0] + n[1] * n[1] + n[2] * n[2] );
	a1 = n[0] * s;
	b1 = n[1] * s;
	c = n[2] * s;
	//d = -( a * x0 + b * y0 + c * z0 );
	*a=-a1/c;
	*b=-b1/c;
	return true;
}

double getCost(double a1,double b1,double a2,double b2)
{
	double cost=sqrt(a1*a1+b1*b1)*sqrt(a2*a2+b2*b2)-(a1*a2+b1*b2);
	return cost;
}



bool checkConvex(Point v1,Point v2,Point v3,Point v4)
{
	GPoint pts[]={GPoint(v1.x(),v1.y()),GPoint(v2.x(),v2.y()),GPoint(v3.x(),v3.y()),GPoint(v4.x(),v4.y())};
		Polygon_2 pgn(pts, pts+4);
		return pgn.is_convex();
}

bool processForCost(Face_handle f1,int i,Point_value_map values,Delaunay *dt)
{
	double z1,z2,z3,z4,za,zb,zc,zd,a1,a2,b1,b2,aa,ba,ab,bb,ac,bc,ad,bd;
	//Point pi,pccwi,pcwi,p0,p1,p2;
	double c1,c2,c3,c4,c5;
	double s1,s2,s3,s4,s5,s6;

	
	Vertex_handle vih=f1->vertex(i);
	Point pi=vih->point();
	
	Point pccwi=f1->vertex(f1->ccw(i))->point(); //shared by neighbor
	
	Point pcwi=f1->vertex(f1->cw(i))->point();	//shared by neighbor
	
	z1=values[pi];
	z2=values[pccwi];
	z3=values[pcwi];
		//int x=pi.x();
						//void getPlaneEqn(int x1,int y1,int z1,int x2,int y2,int z2,int x3,int y3,int z3,double *a,double *b);
	//f1
	if(!getPlaneEqn(pi.x(),pi.y(),z1,pccwi.x(),pccwi.y(),z2,pcwi.x(),pcwi.y(),z3,&a1,&b1))
		return false;
		

	
	Face_handle f2=f1->neighbor(i);
	if (!f1->has_neighbor(f2))
		return false;

	
	
	
	int vihn=f2->index(f1->vertex(f1->ccw(i))); //vertex index of ccw(i) in neighbor f2
	Vertex_handle vin=f2->vertex(f2->ccw(vihn));
	Point pin=f2->vertex(f2->ccw(vihn))->point(); //vertex opposite edge e in neighbor f2
	z4=values[pin];
	
	//f2
	if(!getPlaneEqn(pin.x(),pin.y(),z4,pcwi.x(),pcwi.y(),z3,pccwi.x(),pccwi.y(),z2,&a2,&b2))
		return false;
				//double getCost(double a1,double b1,double a2,double b2);
	
	
	//f1-f2
	c1=getCost(a1,b1,a2,b2); 
	
	
	

	///////////////////////
	Point a,b,c,d;
	Point pd1,pd2;
	c2=0;
	Face_handle fd=f1->neighbor(f1->ccw(i));
	Face_handle fd1,fd2;
	
	int id,id1,id2;
	
	if(f1->has_neighbor(fd))
	{
		
		id=fd->index(vih);  //i in fd
		
		 d=fd->vertex(fd->cw(id))->point(); 
		
		if(!checkConvex(pi,pccwi,pcwi,d)) return false;
		zd=values[d];		
		
		//fd
		if(!getPlaneEqn(pi.x(),pi.y(),z1,pcwi.x(),pcwi.y(),z3,d.x(),d.y(),zd,&ad,&bd))
			return false;
		c2=getCost(a1,b1,ad,bd);

		
		fd1=fd->neighbor(fd->ccw(id));
		
		fd2=fd->neighbor(id);
		
		 id1= fd1->index(vih);//i in fd1
		
		pd1=fd1->vertex(fd1->cw(id1))->point();
		

		 id2= fd2->index(fd->vertex(fd->cw(id)));//d in fd2
		
		pd2=fd2->vertex(fd2->cw(id2))->point();
		

	}
	else return false;

		
	c3=0;
		
	Face_handle fa=f1->neighbor(f1->cw(i));
		
	Face_handle fa1,fa2;
		
	Point pa1,pa2;
	int ia,ia1,ia2;
	if(f1->has_neighbor(fa))
	{
		
		ia=fa->index(vih); //i in fa
		
		a=fa->vertex(fa->ccw(ia))->point(); 
		
		if(!checkConvex(pi,a,pccwi,pcwi)) return false;
		
		za=values[a];		
		//fa
		if(!getPlaneEqn(pi.x(),pi.y(),z1,a.x(),a.y(),za,pccwi.x(),pccwi.y(),z2,&aa,&ba))
			return false;
		c3=getCost(a1,b1,aa,ba);

		
		fa2=fa->neighbor(ia);
		fa1=fa->neighbor(fa->cw(ia));
		
		 ia1=fa1->index(fa->vertex(fa->ccw(ia))); //a in fa1
		pa1=fa1->vertex(fa1->cw(ia1))->point();

		 ia2=fa2->index(fa->vertex(fa->ccw(ia))); //a in fa2
		pa2=fa2->vertex(fa2->ccw(ia2))->point();

	}
	else return false;

		
	c4=0;
		
	Face_handle fc=f2->neighbor(vihn);
		
	Face_handle fc1,fc2;
		
	Point pc1,pc2;
	int ic,ic1,ic2;
	if(f2->has_neighbor(fc))
	{
		ic=fc->index(vin);  //pin in fc
		c=fc->vertex(fc->ccw(ic))->point(); 
		if(!checkConvex(pccwi,pcwi,c,pin)) return false;
		zc=values[c];	
		//fc
		if(!getPlaneEqn(pin.x(),pin.y(),z4,c.x(),c.y(),zc,pcwi.x(),pcwi.y(),z3,&ac,&bc))
			return false;
		c4=getCost(a2,b2,ac,bc);

		fc1=fc->neighbor(ic);
		fc2=fc->neighbor(fc->cw(ic));

		 ic1=fc1->index(fc->vertex(fc->ccw(ic)));//c in fc1
		pc1=fc1->vertex(fc1->ccw(ic1))->point();

		 ic2=fc2->index(fc->vertex(fc->ccw(ic))); //c in fc2
		pc2=fc2->vertex(fc2->cw(ic2))->point();
	}
	else return false;
	
		
	c5=0;
	Face_handle fb=f2->neighbor(f2->cw(vihn));
	Face_handle fb1,fb2;
	Point pb1,pb2;
	int ib,ib1,ib2;
	if(f2->has_neighbor(fb))
	{
		 ib=fb->index(vin);  //pin in fb
		 b=fb->vertex(fb->cw(ib))->point(); 
		 if(!checkConvex(pccwi,pcwi,pin,b)) return false;
		zb=values[b];	
		//fb
		if(!getPlaneEqn(pin.x(),pin.y(),z4,pccwi.x(),pccwi.y(),z2,b.x(),b.y(),zb,&ab,&bb))
			return false;
		c5=getCost(a2,b2,ab,bb);

		fb1=fb->neighbor(ib);
		fb2=fb->neighbor(fb->ccw(ib));

		 ib1=fb1->index(fb->vertex(fb->cw(ib))); //b in fb1
		pb1=fb1->vertex(fb1->cw(ib1))->point();
		 ib2=fb2->index(fb->vertex(fb->cw(ib))); //b in fb2
		pb2=fb2->vertex(fb2->ccw(ib2))->point();

	}
	else return false;
	//s1- cost of triangulan as is
	s1=c1+c2+c3+c4+c5;

//**************************************************************
	//setting 2. edge f1-f2 flipped

		
	if(!f1->has_neighbor(fd)) return false;
	if(!f1->has_neighbor(fa)) return false;
	if(!f2->has_neighbor(fb)) return false;
	if(!f2->has_neighbor(fc)) return false;
	
		
	if(!checkConvex(pi,pccwi,pin,pcwi)) return false;
	//if e is flipped then the 2 new faces r made of pi,pin,pcwi and pi,pccwi,pin
	
	//if setting 2 flip is not allowed then no look ahead can happen
	if(!getPlaneEqn(pi.x(),pi.y(),z1,pin.x(),pin.y(),z4,pcwi.x(),pcwi.y(),z3,&a1,&b1))
		return false;
	if(!getPlaneEqn(pi.x(),pi.y(),z1,pccwi.x(),pccwi.y(),z2,pin.x(),pin.y(),z4,&a2,&b2))
		return false;

	c1=getCost(a1,b1,a2,b2);
	

	c2=0;
	c2=getCost(a1,b1,ad,bd);
	
	c3=0;
	c3=getCost(a2,b2,aa,ba);
	
	c4=0;
	c4=getCost(a1,b1,ac,bc);
	
	c5=0;
	c5=getCost(a2,b2,ab,bb);
	

	s2=c1+c2+c3+c4+c5;
	
	
		
	if(s2<s1)
	{
		dttmp=*dt;
		dttmp.flip(f1,i);
		
		if(dttmp.is_valid())
		{
		
				//dt->flip(f1,i);

		
			return true;
		}
		
		//else return false;
	}

		
//return false;

	//******************************************************************************************
	//******************************************************************************************
	//******************************************************************************************
	//////////////////////////// look ahead stuff

		
	//from here on all 13 edges r required
if(!fd->has_neighbor(fd1)) return false;
if(!checkConvex(pi,pcwi,d,pd1)) return false;
if(!fd->has_neighbor(fd2)) return false;
if(!checkConvex(pi,d,pd2,pcwi)) return false;

if(!fa->has_neighbor(fa1)) return false;
if(!checkConvex(pi,pa1,a,pccwi)) return false;
if(!fa->has_neighbor(fa2)) return false;
if(!checkConvex(pi,a,pa2,pccwi)) return false;

if(!fb->has_neighbor(fb1)) return false;
if(!checkConvex(pccwi,pb1,b,pin)) return false;
if(!fb->has_neighbor(fb2)) return false;
if(!checkConvex(pccwi,b,pb2,pin)) return false;

if(!fc->has_neighbor(fc1)) return false;
if(!checkConvex(pin,c,pc1,pcwi)) return false;
if(!fc->has_neighbor(fc2)) return false;
if(!checkConvex(pin,pc2,c,pcwi)) return false;


		

	double c6,c7,c8,c9,c10,c11,c12,c13;
	double aa1,ba1,aa2,ba2,ab1,bb1,ab2,bb2,ac1,bc1,ac2,bc2,ad1,bd1,ad2,bd2;
	double za1=values[pa1];
	double za2=values[pa2];
	
	double zb1=values[pb1];
	double zb2=values[pb2];

	double zc1=values[pc1];
	double zc2=values[pc2];

	double zd1=values[pd1];
	double zd2=values[pd2];

	if(!getPlaneEqn(pa1.x(),pa1.y(),za1,a.x(),a.y(),za,pi.x(),pi.y(),z1,&aa1,&ba1)) return false;
	if(!getPlaneEqn(pi.x(),pi.y(),z4,a.x(),a.y(),za,pccwi.x(),pccwi.y(),z2,&aa2,&ba2)) return false;
	
	if(!getPlaneEqn(pccwi.x(),pccwi.y(),z2,pb1.x(),pb1.y(),zb1,b.x(),b.y(),zb,&ab1,&bb1)) return false;
	if(!getPlaneEqn(b.x(),b.y(),zb,pb2.x(),pb2.y(),zb2,pin.x(),pin.y(),z4,&ab2,&bb2)) return false;


	if(!getPlaneEqn(pin.x(),pin.y(),z4,pc2.x(),pc2.y(),zc2,c.x(),c.y(),zc,&ac1,&bc1)) return false;
	if(!getPlaneEqn(c.x(),c.y(),zc,pc1.x(),pc1.y(),zc1,pcwi.x(),pcwi.y(),z3,&ac2,&bc2)) return false;
	
	if(!getPlaneEqn(pd1.x(),pd1.y(),zd1,pi.x(),pi.y(),z1,d.x(),d.y(),zd,&ad1,&bd1)) return false;
	if(!getPlaneEqn(d.x(),d.y(),zd,pcwi.x(),pcwi.y(),z3,pd2.x(),pd2.y(),zd2,&ad2,&bd2)) return false;
	
int tryset;
//**************************************************************
	//setting 3	
//c3 c5 same - surely calculated
	
	tryset=1;
	c2=0;
	c1=0;
	c13=0;
	c12=0;
	c4=0;
	double at1,bt1,at2,bt2;
	if(!checkConvex(pi,pin,pcwi,d)) tryset=0;
	//if(fd->has_neighbor(f1))
	{
		if(!getPlaneEqn(pi.x(),pi.y(),z1,pin.x(),pin.y(),z4,d.x(),d.y(),zd,&at1,&bt1)) 
		{
			tryset=0;
			//break;
		}
		c2=getCost(a2,b2,at1,bt1);
		c13=getCost(ad1,bd1,at1,bt1);

		if(!getPlaneEqn(pin.x(),pin.y(),z4,pcwi.x(),pcwi.y(),z3,d.x(),d.y(),zd,&at2,&bt2)) 
		{
			tryset=0;
			//break;
		}
		c1=getCost(at2,bt2,at1,bt1);
	//	if(fd->has_neighbor(fd2))
			c12=getCost(at2,bt2,ad2,bd2);
		
	//	if(f2->has_neighbor(fc))
			c4=getCost(at2,bt2,ac,bc);

	}
	
	c6=0;
	//if(fa->has_neighbor(fa1))
	c6=getCost(aa,ba,aa1,ba1);
	
	c7=0;
	//if(fa->has_neighbor(fa2))
		c7=getCost(aa,ba,aa2,ba2);
	
	c8=0;
	//if(fb->has_neighbor(fb1))
		c8=getCost(ab,bb,ab1,bb1);
	c9=0;
//	if(fb->has_neighbor(fb2))
		c9=getCost(ab,bb,ab2,bb2);
	

	c10=0;
//	if(fc->has_neighbor(fc2))
		c10=getCost(ac,bc,ac2,bc2);
	c11=0;
//	if(fc->has_neighbor(fc1))
		c11=getCost(ac,bc,ac1,bc1);
	if(tryset)
	{
		s3=c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12+c13;
		if(s3<s2)
		{
			//dttmp=*dt;
			//dttmp.flip(f1,i);
			//dttmp.flip(fd,fd->index(fd->vertex(fd->cw(id)) )     );
			//if(dttmp.is_valid()) 
			{dt->flip(f1,i);
			dt->flip(fd,fd->index(fd->vertex(fd->cw(id)) )     );
			return true;
			}
		}
	}
//**************************************************************
	//setting 4
	//c3 c5 c6 c7 c8 c9 reused - surely calc b4
	tryset=1;
	c1=0;
	c2=0;
	c4=0;
	c10=0;
	c11=0;
	c12=0;
	c13=0;
	if(!checkConvex(pi,pin,c,pcwi)) tryset=0;
	//if(f2->has_neighbor(fc))
	{
		if(!getPlaneEqn(pi.x(),pi.y(),z1,pin.x(),pin.y(),z4,c.x(),c.y(),zc,&at1,&bt1)) 
		{
			tryset=0;
			//break;
		}
		if(!getPlaneEqn(pi.x(),pi.y(),z1,c.x(),c.y(),zc,pcwi.x(),pcwi.y(),z3,&at2,&bt2)) 
		{
			tryset=0;
			//break;
		}
		c1=getCost(at1,bt1,at2,bt2);
		c2=getCost(at1,bt1,a2,b2);
	//	if(fc->has_neighbor(fc2))
			c4=getCost(at1,bt1,ac2,bc2);
	//	if(fc->has_neighbor(fc1))
			c10=getCost(at2,bt2,ac1,bc1);
	//	if(f1->has_neighbor(fd))
		{
			c11=getCost(at2,bt2,ad,bd);
	//		if(fd->has_neighbor(fd1))
				c13=getCost(ad,bd,ad1,bd1);
	//		if(fd->has_neighbor(fd2))
				c12=getCost(ad,bd,ad2,bd2);
		}

	}
	if(tryset)
	{
		s4=c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12+c13;
		if(s4<s2)
		{
			//dttmp=*dt;
			//dttmp.flip(f1,i);
			//dttmp.flip(fc,fc->index(fc->vertex(fc->ccw(ic) )) );
			//if(dttmp.is_valid())
			{dt->flip(f1,i);
			dt->flip(fc,fc->index(fc->vertex(fc->ccw(ic) )) );
			return true;
			}
		}
	}
//**************************************************************
	///setting 5
	tryset=1;
	c5=0;c6=0;c10=0;c11=0;c12=0;c13=0;
	if(!checkConvex(pi,a,pccwi,pin)) tryset=0;
	//if(f2->has_neighbor(fa))
	{
		if(!getPlaneEqn(a.x(),a.y(),za,pin.x(),pin.y(),z4,pi.x(),pi.y(),z1,&at1,&bt1))
		{
			tryset=0;
			//break;
		}
		if(!getPlaneEqn(a.x(),a.y(),za,pccwi.x(),pccwi.y(),z2,pin.x(),pin.y(),z4,&at2,&bt2))
		{
			tryset=0;
			//break;
		}
		c5=getCost(aa1,ba1,at1,bt1);
		c6=getCost(aa2,ba2,at2,bt2);
		c9=getCost(a1,b1,at1,bt1);
		c10=getCost(at1,bt1,at2,bt2);
		c11=getCost(ab,bb,at2,bt2);
		c12=getCost(ab1,bb1,ab,bb);
		c13=getCost(ab2,bb2,ab,bb);


	}
		c1=0;
		c7=0;
		c2=0;
	//	if(f1->has_neighbor(fd))
		{
			c7=getCost(ad,bd,a1,b1);
//			if(fd->has_neighbor(fd1))
				c1=getCost(ad,bd,ad1,bd1);
//			if(fd->has_neighbor(fd2))
				c2=getCost(ad,bd,ad2,bd2);

		}
		c4=0;c3=0;c8=0;
	//	if(f2->has_neighbor(fc))
		{
			c8=getCost(a1,b1,ac,bc);
//			if(fc->has_neighbor(fc1))
				c3=getCost(ac,bc,ac1,bc1);
//			if(fc->has_neighbor(fc2))
				c4=getCost(ac,bc,ac2,bc2);
		}
	if(tryset)
	{
		s5=c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12+c13;
		if(s5<s2)
		{
			//dttmp=*dt;
			
			//dttmp.flip(f1,i);
			//dttmp.flip(fa,fa->index(fa->vertex(fa->ccw(ia) )) );
			//if(dttmp.is_valid())
			{
				dt->flip(f1,i);
				dt->flip(fa,fa->index(fa->vertex(fa->ccw(ia) )) );
			
			return true;
			}
		}

	}





//*****************************************************************************
	//last possible setting
	//setting 6
	if(!checkConvex(pi,pccwi,b,pin)) return false;;
	if(!getPlaneEqn(pi.x(),pi.y(),z1,b.x(),b.y(),zb,pin.x(),pin.y(),z4,&at1,&bt1))
		return false;
	
	if(!getPlaneEqn(pi.x(),pi.y(),z1,pccwi.x(),pccwi.y(),z2,b.x(),b.y(),zb,&at2,&bt2))
		return false;

	c11=getCost(a1,b1,at1,bt1);
	c12=getCost(at1,bt1,at2,bt2);
	c5=getCost(at1,bt1,ab2,bb2);
	c6=getCost(at2,bt2,ab1,bb1);
	c9=getCost(at2,bt2,aa,ba);
	c10=getCost(aa1,ba1,aa,ba);
	c13=getCost(aa,ba,aa2,ba2);
	s6=c1+c2+c3+c4+c5+c6+c7+c8+c9+c10+c11+c12+c13;
	if(s6<s2)
	{
		//dttmp=*dt;
		
		//dttmp.flip(f1,i);
		//dttmp.flip(fb,fb->index(fb->vertex(fb->cw(ib) )) );
		//if(dttmp.is_valid())
		{dt->flip(f1,i);
		dt->flip(fb,fb->index(fb->vertex(fb->cw(ib) )) );
		return true;
		}
	}
	return false;
}
