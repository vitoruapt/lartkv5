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
\brief Plplot auxiliary drawing class source code 
*/

#include "plplot_graph.h"

singlePlot::singlePlot(uint id_)
{
	id=id_;
	x=NULL;
	y=NULL;
	Lsize=0;
	allocated_size=0;
	color=PL_BLACK;
	do_not_draw=false;
	shape=PL_NOTRELEVANT;
}

void singlePlot::SetColor(plcolors color_)
{
	color=color_;
}

void singlePlot::SetXLinspace(double min,double max,uint size)
{
	if(x!=NULL)
		free(x);
	
	x=(double*)malloc(size*sizeof(double));
	
	double step=(max-min)/size;
	uint i=0;
	for(double v=min;v<max;v+=step,i++)
		x[i]=v;
	
	Lsize=size;
}

void singlePlot::SetEnv(double xmin_,double xmax_,double ymin_,double ymax_)
{	
	xmin=xmin_;
	xmax=xmax_;
	ymin=ymin_;
	ymax=ymax_;
}

void singlePlot::DynamicScaleEnv(std::vector<t_posePtr>& path,plshape shape_)
{
	shape=shape_;
	
	if(!path.size())
	{
		xmin=-1;
		xmax=1;
		ymin=-1;
		ymax=1;
		return;
	}
	
	xmax=max_x(path);
	xmin=min_x(path);
	
	ymax=max_y(path);
	ymin=min_y(path);
	
	double dx=xmax-xmin;
	double dy=ymax-ymin;
	
	xmax+=0.1*dx;
	xmin-=0.1*dx;
	
	ymax+=0.1*dy;
	ymin-=0.1*dy;

	double xc,yc;
	
	if(shape==PL_SQUARE)
	{
		if(dx>dy)
		{
			yc=(ymin+ymax)/2;
			ymin=yc-dx/2.;
			ymax=yc+dx/2.;
		}else
		{
			xc=(xmin+xmax)/2;
			xmin=xc-dy/2.;
			xmax=xc+dy/2.;
		}
	}
	
	if(xmin==xmax)
	{
		xmin-=1;
		xmax+=1;
	}
	
	if(ymin==ymax)
	{
		ymin-=1;
		ymax+=1;
	}
}

void singlePlot::Env()
{
	if(xmin==xmax)
	{
		xmin=-1;
		xmax=1;
	}
	
	if(ymin==ymax)
	{
		ymin=-1;
		ymax=1;
	}
	
	plenv(xmin,xmax,ymin,ymax,shape,2);
}

void singlePlot::SetLabel(string x_label_,string y_label_,string title_)
{
	title=title_;
	x_label=x_label_;
	y_label=y_label_;
}

void singlePlot::Label()
{
	pllab(x_label.c_str(),y_label.c_str(),title.c_str());
}

void singlePlot::SetSize(uint size)
{
	if(size>allocated_size)
	{
		allocated_size=1024+2*size;
		
		x=(double*)realloc(x,allocated_size*sizeof(double));
		y=(double*)realloc(y,allocated_size*sizeof(double));
	}
	
	Lsize=size;
}

void singlePlot::Plot()
{
	if(!Lsize)
		return;//nothing to draw
	
	if(do_not_draw)
		return;
	
	plcol0(color);
	plline(Lsize,x,y);
	plcol0(PL_BLACK);
}

plSpace::plSpace(uint width,uint height, bool draw_)
{
	draw=draw_;
	
	if(!draw)
		return;
	
	plsdev("xcairo");

	plsetopt("np",NULL);
	plsetopt("geometry","800x800");

	plscolor(true);
	plscolbg(255,255,255);
	plssub(width,height);
	
	for(uint i=0;i<width*height;i++)
	{
		singlePlotPtr sPlot(new singlePlot(i));
		
		sPlot->id=i;
		sPlot->SetEnv(0,1,0,1);
		sPlot->SetLabel("N/A","N/A","Unused");
		
		plotVector.push_back(sPlot);
		
		sPlot.reset(new singlePlot(i));
		sPlot->SetEnv(0,1,0,1);
		sPlot->id=i;
		
		plotVector_2.push_back(sPlot);
		
		sPlot.reset(new singlePlot(i));
		sPlot->SetEnv(0,1,0,1);
		sPlot->id=i;
		
		plotVector_3.push_back(sPlot);
		
		sPlot.reset(new singlePlot(i));
		sPlot->SetEnv(0,1,0,1);
		sPlot->id=i;
		
		plotVector_4.push_back(sPlot);
	}
	
	plinit ();
	plscol0(14,0,0,0);
	plscol0(2,255,210,0);
	plcol0(14);
}

plSpace::~plSpace()
{};
	
void plSpace::Plot()
{
	if(!draw)
		return;
	
	for(uint i=0;i<plotVector.size();i++)//create empty plots for every non existing plot member
	{
		plotVector[i]->Env();//redraw all plots
		plotVector[i]->Label();
		plotVector[i]->Plot();
		
		plotVector_2[i]->Plot();
		plotVector_3[i]->Plot();
		plotVector_4[i]->Plot();
	}
}

void plSpace::SetThirdPlot(uint pos,vector<t_posePtr>& p,double (func)(t_pose&),plcolors color, double Dt)
{
	if(!draw)
		return;
	
	if(!p.size())
		return;
	
	t_posePtr pose=p.back();

	double latest_timestamp=pose->timestamp;
	double iterating_timestamp;
	
	uint vector_size=0;
	
	for(vector<t_posePtr>::reverse_iterator it=p.rbegin();it<p.rend();++it)
	{
		iterating_timestamp=(*it)->timestamp;
		if(fabs(latest_timestamp-iterating_timestamp)<Dt)
			vector_size++;
	}
	
	plotVector_3[pos]->SetSize(vector_size);
	
	uint i=0;
	for(vector<t_posePtr>::reverse_iterator it=p.rbegin();it<p.rend();++it,i++)
	{
		iterating_timestamp=(*it)->timestamp;
		if(fabs(latest_timestamp-iterating_timestamp)>Dt)
			break;
		
		plotVector_3[pos]->x[i]=iterating_timestamp-latest_timestamp;
		plotVector_3[pos]->y[i]=func(*(*it));
	}
	
	plotVector_3[pos]->SetColor(color);
}

void plSpace::SetFourthPlot(uint pos,vector<t_posePtr>& p,double (func)(t_pose&),plcolors color, double Dt)
{
	if(!draw)
		return;
	
	if(!p.size())
		return;
	
	t_posePtr pose=p.back();

	double latest_timestamp=pose->timestamp;
	double iterating_timestamp;
	
	uint vector_size=0;
	
	for(vector<t_posePtr>::reverse_iterator it=p.rbegin();it<p.rend();++it)
	{
		iterating_timestamp=(*it)->timestamp;
		if(fabs(latest_timestamp-iterating_timestamp)<Dt)
			vector_size++;
	}
	
	plotVector_4[pos]->SetSize(vector_size);
	
	uint i=0;
	for(vector<t_posePtr>::reverse_iterator it=p.rbegin();it<p.rend();++it,i++)
	{
		iterating_timestamp=(*it)->timestamp;
		if(fabs(latest_timestamp-iterating_timestamp)>Dt)
			break;
		
		plotVector_4[pos]->x[i]=iterating_timestamp-latest_timestamp;
		plotVector_4[pos]->y[i]=func(*(*it));
	}
	
	plotVector_4[pos]->SetColor(color);
}

void plSpace::SetSecondaryPlot(uint pos,vector<t_posePtr>& p,double (func)(t_pose&),plcolors color, double Dt)
{
	if(!draw)
		return;
	
	if(!p.size())
		return;
	
	t_posePtr pose=p.back();

	double latest_timestamp=pose->timestamp;
	double iterating_timestamp;
	
	uint vector_size=0;
	
	for(vector<t_posePtr>::reverse_iterator it=p.rbegin();it<p.rend();++it)
	{
		iterating_timestamp=(*it)->timestamp;
		if(fabs(latest_timestamp-iterating_timestamp)<Dt)
			vector_size++;
	}
	
	plotVector_2[pos]->SetSize(vector_size);
	
	uint i=0;
	for(vector<t_posePtr>::reverse_iterator it=p.rbegin();it<p.rend();++it,i++)
	{
		iterating_timestamp=(*it)->timestamp;
		if(fabs(latest_timestamp-iterating_timestamp)>Dt)
			break;
		
		plotVector_2[pos]->x[i]=iterating_timestamp-latest_timestamp;
		plotVector_2[pos]->y[i]=func(*(*it));
	}

	plotVector_2[pos]->SetColor(color);
}

void plSpace::SetMember_UseTimestamp(uint pos,vector<t_posePtr>& p,double (func)(t_pose&),plcolors color,pair<double,double> limits,string Xlabel,string Ylabel,string title, double Dt)
{
	if(!draw)
		return;
	
	if(!p.size())
		return;
	
	t_posePtr pose=p.back();

	double latest_timestamp=pose->timestamp;
	double iterating_timestamp;
	
	uint vector_size=0;
	
	for(vector<t_posePtr>::reverse_iterator it=p.rbegin();it<p.rend();++it)
	{
		iterating_timestamp=(*it)->timestamp;
		if(fabs(latest_timestamp-iterating_timestamp)<Dt)
			vector_size++;
	}
	
	plotVector[pos]->SetSize(vector_size);
	
	uint i=0;
	for(vector<t_posePtr>::reverse_iterator it=p.rbegin();it<p.rend();++it,i++)
	{
		iterating_timestamp=(*it)->timestamp;
		if(fabs(latest_timestamp-iterating_timestamp)>Dt)
			break;
		
		plotVector[pos]->x[i]=iterating_timestamp-latest_timestamp;
		plotVector[pos]->y[i]=func(*(*it));
	}
	
	plotVector[pos]->ex_timestamp=latest_timestamp;
	plotVector[pos]->SetEnv(-Dt,0.5,limits.first,limits.second);
	
	plotVector[pos]->SetLabel(Xlabel.c_str(),Ylabel.c_str(),title.c_str());
	plotVector[pos]->SetColor(color);
}

void plSpace::SetRunTime(uint pos,vector<double>& rt,plcolors color,string title,double tmax, double max)
{
	if(!draw)
		return;
	
	if(rt.size()>max)
	{
		plotVector[pos]->SetSize(max);
		
		uint i=0;
		for(vector<double>::iterator it=rt.end()-max;it!=rt.end();it++,i++)
		{
			plotVector[pos]->x[i]=i;
			plotVector[pos]->y[i]=*it;
		}

		plotVector[pos]->SetEnv(0,max,0,tmax);

	}else
	{
		plotVector[pos]->SetSize(rt.size());
		
		for(uint i=0;i<rt.size();i++)
		{
			plotVector[pos]->x[i]=i;
			plotVector[pos]->y[i]=rt[i];
		}
		
		plotVector[pos]->SetEnv(0,rt.size(),0,tmax);
	}
	
	plotVector[pos]->SetColor(color);
	plotVector[pos]->SetLabel("Steps (each 1/50 s)","Run time (ms)",title.c_str());
}

void plSpace::SetMember(uint pos,vector<t_posePtr>& pt,double(func)(s_pose&),plcolors color,double Vmax,double Vmin,string Xlabel,string Ylabel,string title, double max)
{
	if(!draw)
		return;
	
	if(pt.size()>max)
	{
		plotVector[pos]->SetSize(max);
		
		uint i=0;
		for(vector<t_posePtr>::iterator it=pt.end()-max;it!=pt.end();it++,i++)
		{
			plotVector[pos]->x[i]=i;
			plotVector[pos]->y[i]=func(*(*it));
		}

		plotVector[pos]->SetEnv(0,max,Vmin,Vmax);
		
	}else
	{
		plotVector[pos]->SetSize(pt.size());
		
		for(uint i=0;i<pt.size();i++)
		{
			plotVector[pos]->x[i]=i;
			plotVector[pos]->y[i]=func(*pt[i]);
		}
		
		plotVector[pos]->SetEnv(0,pt.size(),Vmin,Vmax);
	}
	
	plotVector[pos]->SetLabel(Xlabel.c_str(),Ylabel.c_str(),title.c_str());
	plotVector[pos]->SetColor(color);
}

void plSpace::SetLinearSpeed(uint pos,vector<t_posePtr>& pt,plcolors color,double tmax, double max)
{
	if(!draw)
		return;
	
	if(pt.size()>max)
	{
		plotVector[pos]->SetSize(max);
		
		uint i=0;
		for(vector<t_posePtr>::iterator it=pt.end()-max;it!=pt.end();it++,i++)
		{
			plotVector[pos]->x[i]=i;
			plotVector[pos]->y[i]=(*it)->vl;
		}

		plotVector[pos]->SetEnv(0,max,0,tmax);
	}else
	{
		plotVector[pos]->SetSize(pt.size());
		
		for(uint i=0;i<pt.size();i++)
		{
			plotVector[pos]->x[i]=i;
			plotVector[pos]->y[i]=pt[i]->vl;
		}
		
		plotVector[pos]->SetEnv(0,pt.size(),0,tmax);
	}
	
	plotVector[pos]->SetLabel("Steps (each 1/50 s)","Linear Speed (m/s)","Linear Speed");
	plotVector[pos]->SetColor(color);
}

void plSpace::SetPath(uint pos,std::vector<t_posePtr>& path,string title)
{
	if(!draw)
		return;
	
	plotVector[pos]->SetSize(path.size());
	
	for(uint i=0;i<path.size();i++)
	{
		plotVector[pos]->x[i]=path[i]->x;
		plotVector[pos]->y[i]=path[i]->y;
	}
	
	plotVector[pos]->DynamicScaleEnv(path,PL_SQUARE);
	plotVector[pos]->SetLabel("x (m)","y (m)",title);
}

void plSpace::SetSecondaryPath(uint pos,std::vector<t_posePtr>& path,plcolors color)
{
	if(!draw)
		return;
	
	plotVector_2[pos]->SetSize(path.size());
	
	for(uint i=0;i<path.size();i++)
	{
		plotVector_2[pos]->x[i]=path[i]->x;
		plotVector_2[pos]->y[i]=path[i]->y;
	}
	
	plotVector_2[pos]->SetColor(color);
}

