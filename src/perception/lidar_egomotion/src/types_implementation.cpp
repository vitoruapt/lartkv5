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
\brief Miscellaneous functions source code
*/

#include "types_declaration.h"

double get_m_phi(t_pose&i)
{
	return i.m_phi;
}

double get_m_vl(t_pose&i)
{
	return i.m_vl;
}

double get_phi(t_pose&i)
{
	return i.phi;
}

double get_orientation(t_pose&i)
{
	return i.orientation;
}

double get_vl(t_pose&i)
{
	return i.vl;
}

double max_orientation(const std::vector<t_posePtr>& vec)
{
    if(vec.empty())return 0;
    
    double max= vec[0]->orientation;
    for(std::vector<t_posePtr>::const_iterator i = vec.begin(); i != vec.end(); ++i)
	    if((*i)->orientation > max)
		    max = (*i)->orientation;
 
    return max;
}

double min_orientation(const std::vector<t_posePtr>& vec)
{
    if(vec.empty())return 0;

    double min= vec[0]->orientation;
    for(std::vector<t_posePtr>::const_iterator i = vec.begin(); i != vec.end(); ++i)
       if((*i)->orientation < min)
		    min = (*i)->orientation;

	return min;
}

double max_x(const std::vector<t_posePtr>& vec)
{
    if(vec.empty())return 0;
    
    double max = vec[0]->x;
    for(std::vector<t_posePtr>::const_iterator i = vec.begin(); i != vec.end(); ++i)
	    if((*i)->x > max)
		    max = (*i)->x;
 
    return max;
}

double min_x(const std::vector<t_posePtr>& vec)
{
    if(vec.empty())return 0;

    double min = vec[0]->x;
    for(std::vector<t_posePtr>::const_iterator i = vec.begin(); i != vec.end(); ++i)
       if((*i)->x < min)
		    min = (*i)->x;

	return min;
}

double max_y(const std::vector<t_posePtr>& vec)
{
    if(vec.empty())return 0;
    
    double max = vec[0]->y;
    for(std::vector<t_posePtr>::const_iterator i = vec.begin(); i != vec.end(); ++i)
	    if((*i)->y > max)
		    max = (*i)->y;
 
    return max;
}

double min_y(const std::vector<t_posePtr>& vec)
{
    if(vec.empty())return 0;

    double min = vec[0]->y;
    for(std::vector<t_posePtr>::const_iterator i = vec.begin(); i != vec.end(); ++i)
       if((*i)->y < min)
		    min = (*i)->y;

	return min;
}


void RemoveOverlappingPoints(ray_config_t*config,ray_measurment_t*rays)
{
	double *range_n;
	double *range_l;
	double diff;
	
	for(int i=0;i<config->num_rays;i++)
	{
		if(rays[i].range->total<2)//single measurment
			continue;
		
		for(int e=1;e<rays[i].range->total;e++)
		{
			range_l=(double*)cvGetSeqElem(rays[i].range,e-1);
			range_n=(double*)cvGetSeqElem(rays[i].range,e);
			
			diff=fabs(*range_l-*range_n);
			
			if(diff<config->minimum_delta)
			{
				*range_n=(*range_l+*range_n)/2;//i don't know if i should use the average or the closest measurment
				cvSeqRemove(rays[i].range,e);
				e--;
			}
		}
	}
}

void RemoveOverlappingPoints(ray_definition*src)
{
	if(!src)
	{
		printf("Watch out for memory allocation, null pointer in %s\n",__FUNCTION__);
		return;
	}
	
	RemoveOverlappingPoints(&src->cfg,src->dt);
}

int Theta2Index(double theta,ray_config_t*config)
{
	int index=cvRound(theta/config->angular_resolution);

	if(index==config->num_rays)
		index=0;
	
	return index;
}

void Pointcloud2Rays(pcl::PointCloud<pcl::PointXYZ>& point_cloud,ray_config_t*config,ray_measurment_t*rays)
{
	double theta,range,*rg;
	int ix;
	int e=0;
	
// 	config->timestamp=point_cloud.header.stamp.toSec();
    printf("PCL timestamp not correct, error during migration lar3 to lar4\n");
	
	
	//this function does not trim the resulting rays so handle with care
	for(uint i=0;i<point_cloud.points.size();i++)
	{	
		theta=atan2(point_cloud.points[i].y,point_cloud.points[i].x);
		if(theta<0)
			theta+=2*M_PI;
		
		range=sqrt(pow(point_cloud.points[i].x,2)+pow(point_cloud.points[i].y,2));
		
		if(range<1.5)
			continue;
		
		ix=Theta2Index(theta,config);
		
		if(ix<0 || ix>config->num_rays)
		{
			printf("Error!, index outside grid, maybe you didn't initialized the ray config\n");
			continue;
		}
		
		for(e=0;e<rays[ix].range->total;e++)
		{
			rg=(double*)cvGetSeqElem(rays[ix].range,e);
			
			if(range<*rg)
				break;
		}
		
		cvSeqInsert(rays[ix].range,e,&range);
	}
}

void PointCloud2Ray(pcl::PointCloud<pcl::PointXYZ>& point_cloud,ray_definition*dst)
{
	if(!dst)
	{
		printf("Wrong pointer, be carefull with memory allocation!\n");
		return;
	}
	
// 	dst->cfg.timestamp=ptc->timestamp;
	dst->cfg.timestamp=ros::Time::now().toSec();
// 	printf("TS %f\n",carmen_get_time()-dst->cfg.timestamp);
	Pointcloud2Rays(point_cloud,&dst->cfg,dst->dt);
}

void ClearRays(ray_measurment_t*rays,ray_config_t*ray_config)
{
	for(int i=0;i<ray_config->num_rays;i++)
		cvClearSeq(rays[i].range);
}

void ClearRays(ray_definition*src)
{
	if(!src)
	{
		printf("Watch out for memory allocation, null pointer in %s\n",__FUNCTION__);
		return;
	}
	
	ClearRays(src->dt,&src->cfg);
}

void InitRayDefinition(ray_definition*src)
{
	src->cfg.angular_resolution=deg2rad(0.5);
	src->cfg.minimum_delta=0.30;
	src->cfg.max_points_per_ray=-1;
	src->cfg.num_rays=2*M_PI/src->cfg.angular_resolution;
	src->cfg.timestamp=0;
	src->cfg.x=0;
	src->cfg.y=0;
	src->cfg.t=0;
	src->dt=(ray_measurment_t*)malloc(src->cfg.num_rays*sizeof(ray_measurment_t));
	
	//start cvseqs for all rays
	for(int i=0;i<src->cfg.num_rays;i++)
	{
		src->dt[i].theta=i*src->cfg.angular_resolution;
		src->dt[i].range_storage=cvCreateMemStorage(0);
		src->dt[i].range=cvCreateSeq(0,sizeof(CvSeq),sizeof(double),src->dt[i].range_storage);
	}
}

void CopyRays(ray_definition*src,ray_definition*dst)
{
	if(!src || !dst)
	{
		printf("Watch out for memory allocation, null pointer in %s\n",__FUNCTION__);
		return;
	}
	
	memcpy(&dst->cfg,&src->cfg,sizeof(ray_config_t));
	
	CopyRays(&src->cfg,src->dt,dst->dt);
}

void CopyRays(ray_config_t*cfg,ray_measurment_t*src,ray_measurment_t*dst)
{
	//this function does not copy cfg
	ClearRays(dst,cfg);
	double*range;

	for(int i=0;i<cfg->num_rays;i++)
	{
		dst[i].theta=src[i].theta;	
		
		for(int e=0;e<src[i].range->total;e++)
		{
			range=(double*)cvGetSeqElem(src[i].range,e);
			cvSeqInsert(dst[i].range,e,range);
		}
	}
}

void InitRayHistory(ray_history*h_rays)
{
	h_rays->data_storage=cvCreateMemStorage(0);;
	h_rays->data=cvCreateSeq(0,sizeof(CvSeq),sizeof(ray_definition),h_rays->data_storage);
}

void AddToHistory(ray_definition*src,ray_history*h_rays)
{
	ray_definition rays;
	
	InitRayDefinition(&rays);
	
	CopyRays(src,&rays);
	
	cvSeqPushFront(h_rays->data,&rays);
	
	if(h_rays->data->total>50)
	{
		ray_definition*rmv=(ray_definition*)cvGetSeqElem(h_rays->data,h_rays->data->total-1);
		
		for(int i=0;i<rmv->cfg.num_rays;i++)
			cvReleaseMemStorage(&(rmv->dt[i].range_storage));

		cvSeqPop(h_rays->data);
	}
}

double get_fps(double dt,t_fps*acc)
{
	static bool initialise=TRUE;
	static unsigned int max_size=30;
	
	if(initialise)
	{
		memset(acc->fps,0,max_size*sizeof(double));
		acc->position=0;
		acc->current_size=0;
		
		initialise=FALSE;
	}
	
	acc->fps[acc->position]=1./(double)dt;
	acc->position++;
	
	if(acc->current_size<max_size)
		acc->current_size++;

	if(acc->position==max_size)
		acc->position=0;
	
	double mean_fps=0;
	
	for(unsigned int i=0;i<acc->current_size;i++)
		mean_fps+=acc->fps[i];
	
	mean_fps/=acc->current_size;
	
	return mean_fps;
}
