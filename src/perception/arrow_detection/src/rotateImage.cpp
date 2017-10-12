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

/*! 
 *	@file 	test1.cpp
 * 	@brief	helper function to rotate an image.
 *  
 *	@author		César Sousa, cesarsousa@ua.pt
 *	@date 6-3-2014
 * 	@version V0.0
 *	@internal
 * 
 * 		Revision	---
 * 		Compiler	gcc
 * 		Company		DEM - Universidade de Aveiro
 * 		Copyright	Copyright (c) 2014, César Sousa
 * 
 * 		Info:		
 * 	
 *	command to make doxyfile: "make doxyfile" than "make doc"
 * 
 */

#include <arrow_detection/rotateImage.h>


double rotateImage::round(double number) {
    return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
}

double rotateImage::degreeToRadian(double degree) {
    return (M_PI*degree)/180.0;
}

double rotateImage::radianToDegree(double radian) {
    return (180*radian)/M_PI;
}

Size rotateImage::rotationNewCanvasSize(double degree,double angle,double h) {
    Size canvas(0,0);
    double radian = degreeToRadian(degree);
    if((degree >= 0 && degree <= 90) || (degree >= 181 && degree <= 270)) {
        canvas.height = round((h*fabs(sin(angle+radian)))*2);
        canvas.width = round((h*fabs(cos(M_PI-angle+radian)))*2);
    } else {
        canvas.height = round((h*fabs(sin(M_PI-angle+radian)))*2);
        canvas.width = round((h*fabs(cos(angle+radian)))*2);
    }
    return canvas;
}

double rotateImage::solveEquationY(Equ e,double x) {
    
    cout << x << endl;
    
    if(e.isVertical) return x;
    
    return e.m*x+e.c;
}

double rotateImage::solveEquationX(Equ e, double y) {
    
    if(e.isVertical) return e.c;
    if(e.isHorizontal) return 0;

    return (y-e.c)/e.m;
}

map<string,int> rotateImage::rotationExtraMargins(Size &original, Size &newSize) {
    
    map<string,int> m = map<string,int>();
    
    if(newSize.height >= original.height) {
        m["top"] = round((newSize.height-original.height)/2.0);
        m["bottom"] = round((newSize.height-original.height)/2.0);
    } else {
        m["top"] = 0;
        m["bottom"] = 0;
    }
    
    if(newSize.width >= original.width) {
        m["left"] = round((newSize.width-original.width)/2.0);
        m["right"] = round((newSize.width-original.width)/2.0);
    } else {
        m["left"] = 0;
        m["right"] = 0;
    }
    
    return m;
}

map<string,Point> rotateImage::getCorners(Size &original,map<string,int> &margins) {
    map<string,Point> m = map<string,Point>();
    
    m["tl"] = Point(margins["left"],margins["top"]);
    m["tr"] = Point(margins["left"]+original.width,margins["top"]);
    m["bl"] = Point(margins["left"],margins["top"]+original.height);
    m["br"] = Point(margins["left"]+original.width,margins["top"]+original.height);
    
    return m;
    
}

map<string,Point> rotateImage::getProjectedCorners(Size &s,double h,double degree,double angle) {
    
    map<string,Point> m = map<string,Point>();
    double radian = degreeToRadian(degree);
    int top,left;
    
    m["tl"] = Point(
                round(s.width/2.0+h*cos(M_PI-angle+radian)),
                round(s.height/2.0-h*sin(M_PI-angle+radian)));
    
    top = m["tl"].y;
    left = m["tl"].x;
    
    m["tr"] = Point(
                round(s.width/2.0+h*cos(angle+radian)),
                round(s.height/2.0-h*sin(angle+radian)));
    
    if(top>m["tr"].y) top=m["tr"].y;
    if(left>m["tr"].x) left=m["tr"].x;
    
    m["br"] = Point(
                round(s.width/2.0+h*cos(-angle+radian)),
                round(s.height/2.0-h*sin(-angle+radian)));
    
    if(top>m["br"].y) top=m["br"].y;
    if(left>m["br"].x) left=m["br"].x;
    
    m["bl"] = Point(
                round(s.width/2.0+h*cos(M_PI+angle+radian)),
                round(s.height/2.0-h*sin(M_PI+angle+radian)));
    
    if(top>m["bl"].y) top=m["bl"].y;
    if(left>m["bl"].x) left=m["bl"].x;
    
    m["tl"].x -= left;
    m["tr"].x -= left;
    m["bl"].x -= left;
    m["br"].x -= left;
    
    m["tl"].y -= top;
    m["tr"].y -= top;
    m["bl"].y -= top;
    m["br"].y -= top;
    
    
    return m;
    
}

Point rotateImage::getCentreBetweenPoints(Point &a, Point &b) {
    Point c(0,0);
    double tmp;
    
    tmp = abs(a.x-b.x)/2.0;
    
    if(b.x <= a.x) c.x = round(tmp+b.x);
    else c.x = round(tmp+a.x);
    
    tmp = abs(a.y-b.y)/2.0;
    
    if(b.y <= a.y) c.y = round(tmp+b.y);
    else c.y = round(tmp+a.y);
    
    return c;
}

map<string,Point> rotateImage::getCentreBetweenOriginalsAndProjections(map<string,Point> &originals,map<string,Point> &projections) {
    map<string,Point> m = map<string,Point>();
    
    m["tl"] = getCentreBetweenPoints(originals["tl"],projections["tl"]);
    m["tr"] = getCentreBetweenPoints(originals["tr"],projections["tr"]);
    m["bl"] = getCentreBetweenPoints(originals["bl"],projections["bl"]);
    m["br"] = getCentreBetweenPoints(originals["br"],projections["br"]);
    
    return m;
    
}

Equ rotateImage::getLinearEquation(Point &a,Point &b) {
    
    Equ equation;
    
    equation.isVertical = false;
    equation.isHorizontal = false;
    
    if(a.x-b.x==0) {
        equation.c = a.x;
        equation.m = 0;
        equation.isVertical = true;
    } else if(a.y-b.y==0) {
        equation.c = a.y;
        equation.m = 0;
        equation.isHorizontal = true;
    } else {
        equation.m = (double)(a.y-b.y)/(a.x-b.x);
        equation.c = a.y-equation.m*a.x;
    }
    
    return equation;
    
}

Equ rotateImage::getPerpendicular(Equ e,Point p) {
    Equ equation;
    
    equation.isVertical = false;
    equation.isHorizontal = false;
    
    if(e.isHorizontal) {
        equation.c = p.x;
        equation.m = 0;
        equation.isVertical = true;
    } else if(e.isVertical) {
        equation.c = p.y;
        equation.m = 0;
        equation.isHorizontal = true;        
    } else {
        equation.m = -1.0/e.m;
        equation.c = -p.x*equation.m+p.y;
    }
    
    return equation;
    
}

map<string,Equ> rotateImage::getLinearEquationBetweenOriginalsAndProjections(map<string,Point> &originals,map<string,Point> &projections) {
    map<string,Equ> m = map<string,Equ>();
    
    m["tl"] = getLinearEquation(originals["tl"],projections["tl"]);
    m["tr"] = getLinearEquation(originals["tr"],projections["tr"]);
    m["bl"] = getLinearEquation(originals["bl"],projections["bl"]);
    m["br"] = getLinearEquation(originals["br"],projections["br"]);
    
    return m;
}

map<string,Equ> rotateImage::getPerpendicularLinearEquation(map<string,Point> &originals,map<string,Point> &projections,map<string,Point> &centre) {
    map<string,Equ> m = map<string,Equ>();
    
    m["tl"] = getPerpendicular(getLinearEquation(originals["tl"],projections["tl"]),centre["tl"]);
    m["tr"] = getPerpendicular(getLinearEquation(originals["tr"],projections["tr"]),centre["tr"]);
    m["bl"] = getPerpendicular(getLinearEquation(originals["bl"],projections["bl"]),centre["bl"]);
    m["br"] = getPerpendicular(getLinearEquation(originals["br"],projections["br"]),centre["br"]);
    
    return m;
}

Point rotateImage::getColisionPoint(Equ e1,Equ e2) {
    
    Point2d p = Point2d(0,0);
    
    if(e1.isHorizontal && e2.isVertical) {
        
        p.x = e2.c;
        p.y = e1.c;
        
    } else if(e1.isVertical && e2.isHorizontal) {
        
        p.x = e1.c;
        p.y = e2.c;
        
    } else if(e1.isHorizontal) {
        
        p.y = e1.c;
        p.x = (p.y-e2.c)/e2.m;
        
    } else if(e2.isHorizontal) {
        
        p.y = e2.c;
        p.x = (p.y-e1.c)/e1.m;
        
    } else if(e1.isVertical) {
        
        p.x = e1.c;
        p.y = p.x*e2.m+e2.c;
        
    } else if(e2.isVertical) {
        
        p.x = e2.c;
        p.y = p.x*e1.m+e1.c;
        
    } else {
        
        p.y = fabs(e1.c*e2.m-e2.c*e1.m)/fabs(e1.m-e2.m);
        p.x = (p.y-e1.c)/e1.m;
        
    }
    
    return Point(round(p.x),round(p.y));
}
/**
 * *Main function*
 * 
 * */
void rotateImage::rotate_image(Mat & src, Mat & dst,const double degree)
    {
    
    Mat copy,schema,rot_mat; 
    map<string,int> margins; //?? index por "string" e guarda inteiros 
    Size img,canvas,workspace;// estrutura do tipo size!
    
    double h,angle;//degree; 
    map<string,Point> original, projection, centre; //??
    
    map<string,Equ> pEqu;
    
    Point2f src_centre; //center point
   
    img = Size(src.cols,src.rows);
    h = sqrt(pow(img.width/2.0,2) + pow(img.height/2.0,2));
    angle = atan((double)img.height/img.width);
    
        canvas = rotationNewCanvasSize(degree,angle,h);
        margins = rotationExtraMargins(img,canvas);
        copyMakeBorder(src,copy,margins["top"],margins["bottom"],margins["left"],
            margins["right"],BORDER_CONSTANT,Scalar(0,0,0));
        copy.copyTo(schema);
        workspace = Size(copy.cols,copy.rows);
        
        original = getCorners(img,margins);
        projection = getProjectedCorners(canvas,h,degree,angle);
        centre = getCentreBetweenOriginalsAndProjections(original,projection);
        pEqu = getPerpendicularLinearEquation(original,projection,centre);
    
        
        if(img.width > canvas.width || img.height > canvas.height) {
            Point p = getColisionPoint(pEqu["tl"],pEqu["tr"]);
            src_centre = Point2f(p.x,p.y);
        } else {
            src_centre = Point2f(canvas.width/2.0,canvas.height/2.0);
        }
        
     
            dst.create(canvas.height,canvas.width,src.channels());
            rot_mat = getRotationMatrix2D(src_centre, degree, 1.0);
            cv::warpAffine(copy, dst, rot_mat,canvas);
     
}
