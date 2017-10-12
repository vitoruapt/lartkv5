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
\file
\brief Plplot auxiliary drawing class declaration
*/

#ifndef _PLPLOT_GRAPH_H_
#define _PLPLOT_GRAPH_H_

#ifdef __GNUC__
#define DEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(func) func
#endif

#include <plplot.h>
#include <iostream>
#include <string>

#include <stdlib.h>
#include <math.h>

#include "types_declaration.h"

using namespace std;

enum plcolors {PL_BLACK=14,PL_CYAN=11,PL_WHITE=15,PL_YELLOW=2,PL_RED=1,PL_BLUE=9};
enum plshape {PL_SQUARE=1,PL_NOTRELEVANT=0};

class singlePlot
{
	public:
		uint id;
		double*x;
		double*y;
		uint Lsize;
		uint allocated_size;
		
		double xmin;
		double xmax;
		double ymin;
		double ymax;
		bool do_not_draw;
		
		string title;
		string x_label;
		string y_label;
		
		plshape shape;
		
		plcolors color;
		
		double ex_timestamp;
		
		singlePlot(uint id_);
		
		void SetColor(plcolors color_);
		
		void SetXLinspace(double min,double max,uint size);
		
		void SetEnv(double xmin_,double xmax_,double ymin_,double ymax_);
		
		void DynamicScaleEnv(std::vector<t_posePtr>& path,plshape shape_);
		
		void Env();
		
		void SetLabel(string x_label_,string y_label_,string title_);
		
		void Label();
		
		void SetSize(uint size);
		
		void Plot();
};

typedef boost::shared_ptr<singlePlot> singlePlotPtr;

class plSpace
{
	public:
		
		bool draw;
		
		vector<singlePlotPtr> plotVector;
		vector<singlePlotPtr> plotVector_2;
		vector<singlePlotPtr> plotVector_3;
		vector<singlePlotPtr> plotVector_4;
		
		plSpace(uint width,uint height, bool draw_=true);
		
		~plSpace();
	
		void Plot();
		
		void SetThirdPlot(uint pos,vector<t_posePtr>& p,double (func)(t_pose&),plcolors color, double Dt);
		
		void SetFourthPlot(uint pos,vector<t_posePtr>& p,double (func)(t_pose&),plcolors color, double Dt);
		
		void SetSecondaryPlot(uint pos,vector<t_posePtr>& p,double (func)(t_pose&),plcolors color, double Dt);
		
		void SetMember_UseTimestamp(uint pos,vector<t_posePtr>& p,double (func)(t_pose&),plcolors color,pair<double,double> limits,string Xlabel,string Ylabel,string title, double Dt);
									
		void SetRunTime(uint pos,vector<double>& rt,plcolors color,string title,double tmax, double max);
		
		void SetMember(uint pos,vector<t_posePtr>& pt,double(func)(s_pose&),plcolors color,double Vmax,double Vmin,string Xlabel,string Ylabel,string title, double max);
		
		void SetLinearSpeed(uint pos,vector<t_posePtr>& pt,plcolors color,double tmax, double max);
		
		void SetPath(uint pos,std::vector<t_posePtr>& path,string title);
		
		void SetSecondaryPath(uint pos,std::vector<t_posePtr>& path,plcolors color);
	private:
		
};

#endif
