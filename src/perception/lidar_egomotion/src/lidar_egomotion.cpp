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
\brief Egomotion from lidar main source code
*/

#include "lidar_egomotion.h"

ray_definition c_rays;//current rays
ray_history h_rays;//rays history
ray_definition*l_rays=NULL;
ray_definition*l2_rays=NULL;

scan_matching_method scan_method = MBICP;

extern TSMparams params;
struct sm_params params_sm;
struct sm_result results_sm;

Tsc sensorMotion,solution,solution2;

int step=2;//only 1/50 between scans

LDP ldp_1=NULL;//PlICP
LDP ldp_2=NULL;

PMScan pm1;//PSM
PMScan pm2;

Tpfp*fp1=NULL;//MbICP
Tpfp*fp2=NULL;
Tpfp*fp3=NULL;

double bwa=3.40;
double gl=2.6;

double z[2];

t_flag flags;

std::vector<t_posePtr> path_odo;
std::vector<t_posePtr> path_laser;

std::vector<double>runtime_PlICP;
std::vector<double>runtime_MbICP;
std::vector<double>runtime_PSM;

std::vector<pair<double,double> >odometry_readings;
std::vector<pair<double,double> >measurments;

tf::TransformListener *transform_listener;
pcl::PointCloud<pcl::PointXYZ> point_cloud_2;
pcl::PointCloud<pcl::PointXYZ> point_cloud_3;
bool new_laser_2=false;
bool new_laser_3=false;
bool new_plc_status=false;
bool new_velocity_status=false;

void CleanZone(ray_definition*rays,double steering_angle)
{
	if(rays==NULL)
		return;
	
	double left=0;
	double right=0;
// 	double mid_point=0;
	double min_angle=0;
	double max_angle=0;
// 	double new_range = 10;
	
	if(steering_angle>0.2 || steering_angle<-0.2)
	{
		if(steering_angle>0.1)
			left=fabs(steering_angle)*100.;
		else
			right=fabs(steering_angle)*100.;
		
		min_angle=(180.0-left)*M_PI/180.;
		max_angle=(180.0+right)*M_PI/180.;
		
		for(int i=0; i<rays->cfg.num_rays; i++)
			if(rays->dt[i].theta>min_angle && rays->dt[i].theta<max_angle)
				cvClearSeq(rays->dt[i].range);
	}
	
// 	if(steering_angle>0.4 || steering_angle<-0.4)
// 	{
// 		if(steering_angle>0.4)
// 		{
// 			mid_point=60.0;
// 			left=10.;
// 			right=10.;
// 		}else
// 		{
// 			mid_point=-60.0;
// 			left=10.;
// 			right=10.;
// 		}
// 		
// 		min_angle=(mid_point-left)*M_PI/180.;
// 		max_angle=(mid_point+right)*M_PI/180.;
// 		
// 		for(int i=0; i<rays->cfg.num_rays; i++)
// 			if(rays->dt[i].theta>min_angle && rays->dt[i].theta<max_angle)
// 				cvClearSeq(rays->dt[i].range);
// 	
// 	if(fabs(steering_angle)<0.1)
// 	{
// 		mid_point=180.0;
// 		left=15.;right=15.;
// 		
// 		min_angle=(mid_point-left)*M_PI/180.;
// 		max_angle=(mid_point+right)*M_PI/180.;
// 		
// 		for(int i=0; i<rays->cfg.num_rays; i++)
// 		{
// 			if(rays->dt[i].range->total>0)
// 			{
// 				if(rays->dt[i].theta>min_angle && rays->dt[i].theta<max_angle)
// 				{
// 					//The measurment exists and is in the correct interval
// 					cvClearSeq(rays->dt[i].range);
// // 					cvSeqInsert(rays->dt[i].range,0,&new_range);
// 				}
// 			}
// 		}
// 	}
// 	
	
// 	if(fabs(steering_angle)<0.1)
// 	{
// 		mid_point=180.0;
// 		left=15.;right=15.;
// 		
// 		min_angle=(mid_point-left)*M_PI/180.;
// 		max_angle=(mid_point+right)*M_PI/180.;
// 		
// 		for(int i=0; i<rays->cfg.num_rays; i++)
// 			if(rays->dt[i].theta>min_angle && rays->dt[i].theta<max_angle)
// 				cvClearSeq(rays->dt[i].range);
// 	}
	
// 	vector<pair<double,double> >points;
// 
// 	for(int i=0; i<rays->cfg.num_rays-2; i++)
// 	{
// 		double x,y;
// 		double *r;
// 		pair<double,double> p;
// 		
// 		if(rays->dt[i].range->total>0)
// 		{
// 			r=(double*)cvGetSeqElem(rays->dt[i].range,rays->dt[i].range->total-1);//use longer measurment
// 			
// 			x=*r*cos(rays->dt[i].theta);
// 			y=*r*sin(rays->dt[i].theta);
// 			
// 			p=make_pair(x,y);
// 		}else
// 		{
// 			p=make_pair(-1,-1);
// 		}
// 		
// 		points.push_back(p);
// 	}
	
// 	double a01,a12,a02;
// 	
// 	
// 	for(uint i=0;i<points.size()-2;i++)
// 	{
// 		pair<double,double> p0=points[i];
// 		pair<double,double> p1=points[i+1];
// 		pair<double,double> p2=points[i+2];
// 		
// 		
// 		if(p0.second==-1 || p1.second ==-1 || p2.second==-1)
// 		{}
// 		else
// 		{
// 			a01=atan2(p0.second-p1.second,p0.first-p1.first);
// 			a12=atan2(p1.second-p2.second,p1.first-p2.first);
// 			a02=atan2(p0.second-p2.second,p0.first-p2.first);
// 			
// 			if(fabs(a02-a01) < 2.*M_PI/180. && fabs(a02-a12) < 2.*M_PI/180.)
// 			{
// 				cvClearSeq(rays->dt[i].range);
// // 				cvSeqInsert(rays->dt[i].range,0,&new_range);
// 				
// 				cvClearSeq(rays->dt[i+1].range);
// // 				cvSeqInsert(rays->dt[i+1].range,0,&new_range);
// 				
// 				cvClearSeq(rays->dt[i+2].range);
// // 				cvSeqInsert(rays->dt[i+2].range,0,&new_range);
// 			}
// 		}
// 	}
}

void RaysToPMScan(ray_definition*src,PMScan*dst)
{
	if(!src)
	{
		printf("Watch out for memory allocation, null pointer in %s\n",__FUNCTION__);
		for(int i=0; i<PM_L_POINTS; i++)
		{
			dst->bad[i]=1;
			dst->r[i]=-1;
			dst->x[i]=-1;
			dst->y[i]=-1;
		}
		return;
	}
	
	RaysToPMScan(&src->cfg,src->dt,dst);
}

void ConvertEstimatedToMeasurment(double vl,double dir,float*dx,float*dy,float*dtheta,double dt,double l,double bwa)
{
	double R,Rfw,s,db,b1;
	Vector cc(2);
	
	if(isnan(vl) || isnan(dir))
	{
		printf("Warning, invalid estimated values in %s\n",__FUNCTION__);
		*dx=0;
		*dy=0;
		*dtheta=0;
		return;
	}
	
	R=l/tan(dir);

	cc(2)=R;
	cc(1)=-bwa;

	Rfw=sqrt(cc(1)*cc(1)+cc(2)*cc(2));
	Rfw=R>0?Rfw:-Rfw;
	
	s=vl*dt;

	db=fabs(s/Rfw);
	db=vl>0?db:-db;

	b1=atan2(bwa,fabs(cc(2)));

	*dy=Rfw*cos(b1)-Rfw*cos(b1+db);

	Rfw=R>0?Rfw:-Rfw;
	*dx=Rfw*sin(b1+db)-Rfw*sin(b1);

	*dtheta=db;
	
	if(isnan(*dx)||isnan(*dy))
	{
		*dx=0;
		*dy=0;
	}
	
	*dx*=-1;
	*dy*=-1;
	*dtheta*=-1;
	
	if(dir<0)
		*dtheta*=-1;
	
	return;
}

void RaysToPMScan(ray_config_t*cfg,ray_measurment_t*src,PMScan*dst)
{
	if(dst==NULL)
	{
		printf("PMScan not alloc\n");
		return;
	}
	
	double*range;
	dst->rx = 0;
	dst->ry = 0;
	dst->th = 0;  
	dst->t = -1.0;

	for(int i=0; i<cfg->num_rays; i++)
	{
		if(src[i].range->total>0)
		{
			range=(double*)cvGetSeqElem(src[i].range,0);
			dst->bad[i]=0;
			dst->r[i]=(*range)*100;
			dst->x[i]=(dst->r[i])*cos(src[i].theta);
			dst->y[i]=(dst->r[i])*sin(src[i].theta);
		}else
		{
			dst->bad[i]=1;
			dst->r[i]=-1;
			dst->x[i]=-1;
			dst->y[i]=-1;
		}
	}
	
	return;
}

void RaysToLDP(ray_definition*src,LDP dst,bool test_angle,double angle)
{
	if(!src || !dst)
	{
		if(!dst)
		{
			printf("Both pointers are null, how can this be? serious bug!! :), in %s!\n",__FUNCTION__);
			return;
		}
		
		printf("Wrong pointer, be carefull with memory allocation, in %s!\n",__FUNCTION__);
		for(int i=0;i<dst->nrays;i++)
		{			
			dst->valid[i]=0;
			dst->readings[i]=0;
			dst->theta[i]=-1;
		}
		return;
	}
	
	RaysToLDP(&src->cfg,src->dt,dst,test_angle,angle);
}

void RaysToLDP(ray_config_t*cfg,ray_measurment_t*src,LDP dst,bool test_angle,double angle)
{
	if(dst==NULL)
	{
		printf("LDP not alloc\n");
		for(int i=0;i<cfg->num_rays;i++)
		{			
			dst->valid[i]=0;
			dst->readings[i]=0;
			dst->theta[i]=-1;
		}
		return;
	}
	
	dst->min_theta=src[0].theta;
	dst->max_theta=src[cfg->num_rays-1].theta;
	
	double*range;
// 	double x0,y0,x1,y1,a0;
	
	for(int i=0;i<cfg->num_rays;i++)
	{			
		if(src[i].range->total>0)
		{
// 			range=(double*)cvGetSeqElem(src[i].range,0);
			range=(double*)cvGetSeqElem(src[i].range,src[i].range->total-1);//use longer measurment
			
			dst->valid[i]=1;
			dst->readings[i]=*range;
			dst->theta[i]=src[i].theta;
			/*
			if(i>0  && src[i-1].range->total>0 && test_angle)
			{
				x1=dst->readings[i]*cos(dst->theta[i]);
				y1=dst->readings[i]*sin(dst->theta[i]);
				
				x0=dst->readings[i-1]*cos(dst->theta[i-1]);
				y0=dst->readings[i-1]*sin(dst->theta[i-1]);
				
				a0=atan2(y1-y0,x1-x0);
				
				if( (fabs(a0-M_PI/2) > angle) && (fabs(a0+M_PI/2) > angle) )
					dst->valid[i]=0;
				
			}*/
		
		}else
		{
			dst->valid[i]=0;
			dst->readings[i]=0;
			dst->theta[i]=src[i].theta;
		}
		
// 		if(src[i].theta>M_PI/2+M_PI/4 && src[i].theta<M_PI/2+M_PI/4+M_PI/2)
// 		{
// 			dst->valid[i]=0;
// 			dst->readings[i]=0;
// 			dst->theta[i]=src[i].theta;
// 		}
	}
	
	dst->estimate[0]=0;
	dst->estimate[1]=0;
	dst->estimate[2]=0;
	
	dst->odometry[0]=0;
	dst->odometry[1]=0;
	dst->odometry[2]=0;

	dst->tv.tv_sec=0;
	dst->tv.tv_usec=0;
}

int RaysToMbICP(ray_definition*src,Tpfp*dst)
{
	if(!src)
	{
		printf("Watch out for memory allocation, null pointer in %s\n",__FUNCTION__);
		for(int i=0; i<MAXLASERPOINTS;i++)
		{
			dst[i].r=200.;
			dst[i].t=-1;
		}
		return -1;
	}
	
	return RaysToMbICP(&src->cfg,src->dt,dst);
}

int RaysToMbICP(ray_config_t*cfg,ray_measurment_t*src,Tpfp*dst,bool test_angle,double angle)
{
	if(dst==NULL)
	{
		printf("MbICP not alloc\n");
		return -1;
	}
	
	double*range;
	
// 	double x0,y0,x1,y1,a0,*r1,*r0;
	
// 	int remove[cfg->num_rays+10];
	
// 	memset(remove,0,sizeof(remove));
	
// 	int cnt=1;
	
	for(int i=0; i<cfg->num_rays; i++)
	{
		if(src[i].range->total>0)
		{
// 			if(i>0 && src[i-1].range->total>0/* && test_angle*/)
// 			{
// 				r1=(double*)cvGetSeqElem(src[i].range,0);
// 				x1=*r1*cos(src[i].theta);
// 				y1=*r1*sin(src[i].theta);
// 			
// 				r0=(double*)cvGetSeqElem(src[i-1].range,0);
// 				x0=(*r0)*cos(src[i-1].theta);
// 				y0=(*r0)*sin(src[i-1].theta);
// 				
// 				a0=atan2(y1-y0,x1-x0);
// 				
// // 				if( (fabs(a0-M_PI/2) > angle) && (fabs(a0+M_PI/2) > angle) )
// // 				{
// // 					remove[i]=1;
// // 					remove[i-1]=1;
// // 					counter++;
// // 					dst[i].r=200.;
// // 					dst[i-1].r=200.;
// // 					dst[i].t=src[i].theta;
// // 					continue;//jump current point
// // 				}
// 			}
// 			
// 			range=(double*)cvGetSeqElem(src[i].range,0);//use first measurment			
			range=(double*)cvGetSeqElem(src[i].range,src[i].range->total-1);//use longer measurment
			
			
			
// 			if(*range<2. && 0)
// 			{
// 				if(cnt%3)
// 				{
// 					dst[i].r=200.;
// 					dst[i].t=src[i].theta;
// 				}else
// 				{
// 					dst[i].r=*range;
// 					dst[i].t=src[i].theta;
// 				}
				
// 				cnt++;
				
// 				flip=!flip;
// 			}else
// 			{
// 				cnt=1;
				dst[i].r=*range;
				dst[i].t=src[i].theta;
// 			}
			
// 			if(src[i].theta>M_PI/2+M_PI/4 && src[i].theta<M_PI/2+M_PI/4+M_PI/2)
// 				dst[i].r=200.;
		}else
		{
			dst[i].r=200.;
			dst[i].t=src[i].theta;
		}
	}
	/*
	if(counter>300)//remove points that lead to no good
	{
		for(int i=0;i<cfg->num_rays;i++)
			if(remove[i])
				dst[i].r=200.;
	}
	*/
// 	return counter;
	return 0;
}

void CreateMeasurementFromDisplacement(double dx,double dy,double dtheta,double z[2],double dt,double l,double bwa)
{
	double t1,t2,dbeta,k,R=0,s,rx,ry,Rfw=0;
	
	rx=-sin(dtheta)*dy-dx*cos(dtheta);
	ry=-cos(dtheta)*dy+dx*sin(dtheta);
	
	if(isnan(rx) || isnan(ry))
	{
		printf("Warning, invalid measurments in %s\n",__FUNCTION__);
		z[0]=0;
		z[1]=0;
		return;
	}

	Vector v0(2);
	Vector v1(2);
	Vector p0(2);
	Vector p1(2);
	Vector p2(2);
	Vector cc(2);
	Vector mp(2);
	
// 	transTest[0]=rx;
// 	transTest[1]=ry;
// 	transTest[2]=-dtheta;
	
	p0(1)=0;//first point of the reference axis
	p0(2)=0;
	
	p1(1)=-bwa;//rear axis position
	p1(2)=0;
	
	p2(1)=rx;
	p2(2)=ry;
	
	v0(1)=p2(1)-p0(1);
	v0(2)=p2(2)-p0(2);

	v1(1)=-v0(2);
	v1(2)=v0(1);
	
	mp(1)=(p2(1)+p0(1))/2.;
	mp(2)=(p2(2)+p0(2))/2.;
	
	k=(-bwa-mp(1))/v1(1);

	if(isinf(k))
	{
		z[1]=0;
		s=rx;
	}else
	{
		cc(1)=mp(1)+k*v1(1);
		cc(2)=mp(2)+k*v1(2);

		R=cc(2);
		
		Rfw=sqrt(cc(1)*cc(1)+cc(2)*cc(2));
		Rfw=R>0?Rfw:-Rfw;
		
		z[1]=atan(l/R);
		
		t1=atan2(p0(1)-cc(1),p0(2)-cc(2));
		t2=atan2(p2(1)-cc(1),p2(2)-cc(2));
		
		dbeta=t1-t2;
		s=dbeta*Rfw;
	}
	
	z[0]=s/dt;
	
// 	printf("S %f\n",s);
// 	printf("Speed %f\n",z[0]);
}

void SetDefaultConfiguration(void)
{
	cout<<"Setting default configuration"<<endl;
	
	srand(time(NULL));

	InitRayDefinition(&c_rays);
	InitRayHistory(&h_rays);
	
	ldp_1=ld_alloc_new(c_rays.cfg.num_rays);
	ldp_2=ld_alloc_new(c_rays.cfg.num_rays);
	
	ConfigurePLICP(&params_sm,ldp_1,ldp_2);
 
// 	MbICP configuration
	float max_laser_range=100.;
// 	float Bw=deg2rad(20);
	float Bw=deg2rad(5);
// 	float Bw=deg2rad(10);
// 	float Br=1.0;
	float Br=1.5;
// 	float Br=1.0;
// 	float Br=0.5;
	float L=2.5;
// 	float L=3.0;
	int   laserStep=1;
// 	float MaxDistInter=0.8;
	float MaxDistInter=1.5;
	float filter=0.80;
// 	float filter=0.80;
// 	float filter=0.60;
	int   ProjectionFilter=0;
	float AsocError=0;
	int   MaxIter=500;
	float errorRatio = 0.0001;// 0.00001;
	float errx_out   = 0.001;
	float erry_out   = 0.001;
	float errt_out   = 0.001;
	int IterSmoothConv=4;
	
	Init_MbICP_ScanMatching(max_laser_range,Bw,Br,L,laserStep,MaxDistInter,filter,ProjectionFilter,AsocError,MaxIter,errorRatio,errx_out,erry_out,errt_out,IterSmoothConv);
	fp1=(Tpfp*)malloc(1080*sizeof(Tpfp));
	fp2=(Tpfp*)malloc(1080*sizeof(Tpfp));
	fp3=(Tpfp*)malloc(1080*sizeof(Tpfp));
	
	memset(&solution,0,sizeof(Tsc));
	
// 	ego_motionCV_x=CreateModelCV_SC();
// 	ego_motionCV_y=CreateModelCV_SC();
// 	modelFwdCt=CreateModelFwdCt();
	
	pm_init();

	return;
}

void ConfigurePLICP(struct sm_params*params,LDP ref,LDP sens)
{
	params->laser_ref=ref;
	params->laser_sens=sens;
	params->use_point_to_line_distance=1;//PLICP
	params->use_corr_tricks=1;//use the tricks described in the PLICP paper
	params->max_iterations=500;
	
	/** A threshold for stopping. */
	params->epsilon_xy=0.001;
	/** A threshold for stopping. */
	params->epsilon_theta=0.001;
	
	/** Maximum angular displacement between scans (deg)*/
	params->max_angular_correction_deg=5.;
	/** Maximum translation between scans (m) */
	params->max_linear_correction=0.8;
	
	params->max_correspondence_dist=1.0;
	/** Noise in the scan */
	params->sigma = 0.01;
	params->restart = 1;
	
	params->min_reading = 0.5;
	params->max_reading = 100.;
	
	params->use_sigma_weights = 0;
	
	params->first_guess[0]=results_sm.x[0];
	params->first_guess[1]=results_sm.x[1];
	params->first_guess[2]=results_sm.x[2];
	
	params->use_ml_weights=0;

	params->restart_threshold_mean_error = 8.0 / 300.0;
	params->restart_dt= 0.01;
	params->restart_dtheta= 1* M_PI /180;

	params->clustering_threshold = 0.8;
	params->orientation_neighbourhood = 0;

	params->do_alpha_test = 0;
	
	/** Percentage of correspondences to consider: if 0.9,
	    always discard the top 10% of correspondences with more error */
	params->outliers_maxPerc = 0.8;

	params->outliers_adaptive_order=1;
	params->outliers_adaptive_mult=4;
	params->do_visibility_test = 1;
	params->do_compute_covariance = 0;
	
	return;
}

pcl::PointCloud<pcl::PointXYZ> wrapper_Laserscan2PointCloud(const sensor_msgs::LaserScan& scan)
{
	//Laser scan projector
	laser_geometry::LaserProjection projector;
	//Input point cloud	in ROS format
	sensor_msgs::PointCloud2 point_cloud_input;
	//Input point cloud in PCL format
	pcl::PointCloud<pcl::PointXYZ> point_cloud_pcl_input;
	//Output point cloud in PCL format
	pcl::PointCloud<pcl::PointXYZ> point_cloud_pcl_output;
	//Transformation
	tf::StampedTransform transform;
	
	//Convert from laser scan to point cloud
	projector.transformLaserScanToPointCloud(scan.header.frame_id,scan,point_cloud_input,*transform_listener);
	
	//Convert from ros format to pcl
// 	pcl::fromROSMsg(point_cloud_input,point_cloud_pcl_input);
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(point_cloud_input, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, point_cloud_pcl_input);
    
	//Lookup the current transform
	try
	{
		transform_listener->lookupTransform(scan.header.frame_id,ros::names::remap("/tracking_frame"), ros::Time(0), transform);
	}catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}
	
	//Transform points
	pcl_ros::transformPointCloud(point_cloud_pcl_input,point_cloud_pcl_output,transform.inverse());
	//Change frame id of the point cloud
	point_cloud_pcl_output.header.frame_id = ros::names::remap("/tracking_frame");
	return point_cloud_pcl_output;
}

void laser_2_Handler(const sensor_msgs::LaserScan& scan)
{
	point_cloud_2 = wrapper_Laserscan2PointCloud(scan);	
	new_laser_2=true;
}

void laser_3_Handler(const sensor_msgs::LaserScan& scan)
{
	point_cloud_3 = wrapper_Laserscan2PointCloud(scan);
	new_laser_3=true;
}

// void plcStatusHandler(const atlascar_base::AtlascarStatus& scan)
// {
// 	cout<<"Received plc status"<<endl;
// 	new_plc_status=true;
// }

// void velocityStatusHandler(const atlascar_base::AtlascarVelocityStatus& scan)
// {
// 	cout<<"Received velocity status"<<endl;
// 	new_velocity_status=true;
// }

int main(int argc,char**argv)
{
	// Initialize ROS
	ros::init(argc,argv,"lidar_egomotion");
	
	ros::NodeHandle nh("~");
	
	transform_listener= new tf::TransformListener(nh,ros::Duration(10));
	
	//Subscribe to left and right laser and atlascar egomotion
	//laser 2 is the left laser (supposedly)
	ros::Subscriber sub_laser2 = nh.subscribe("/snr/las/2/scan",1,laser_2_Handler);
	//laser 3 is the left laser (supposedly)
	ros::Subscriber sub_laser3 = nh.subscribe("/snr/las/3/scan",1,laser_3_Handler);
	//atlascar plc status message
// 	ros::Subscriber sub_plc_status = nh.subscribe ("/vhc/plc/status",1,plcStatusHandler);
	//atlascar velocity status
//     ros::Subscriber sub_velocity_status = nh.subscribe ("/vhc/velocity/status",1,velocityStatusHandler);
	
	cout<<"Start to spin"<<endl;
	
	ros::Rate r(500);
	
	constant_velocity_nh motion_model(gl,1./50.,scan_method);
	
	Vector u(0);
	
	double time_interval=(double)step/50.;
	double dtsm;

	double rtPlICP=0;
	double rtMbICP=0;
	double rtPSM=0;
	int ret=0;
	
	t_fps mean_fps;
	
	bool invalid=false;

	Vector zekf(2);
	long counter=0;
	double t_elapsed=0;
	
	SetDefaultConfiguration();
	
	plSpace PlotSpace(2,2,true);
	
	
	ros::Time t=ros::Time::now();
	
	while(ros::ok())
	{
		ros::spinOnce();
		
		if(!new_laser_2 || !new_laser_3)
			continue;
		
        ROS_ERROR("PCL timestamp not correct, error during migration lar3 to lar4");
// 		double freq=1./((point_cloud_2.header.stamp-t).toSec());
// 		cout<<"Input freq:"<<freq;
// 		if(freq<40)
// 			cout<<"\033[1;31m"<<" Error!! messages too far apart"<<"\033[0m"<<endl;
// 		else
// 			cout<<endl;
		
// 		t=point_cloud_2.header.stamp;
		
		new_laser_2=false;
		new_laser_3=false;
		
		continue;
		
		
		ClearRays(&c_rays);
		
		PointCloud2Ray(point_cloud_2,&c_rays);
		PointCloud2Ray(point_cloud_3,&c_rays);
		RemoveOverlappingPoints(&c_rays);
		
		Vector first_guess_estimated=motion_model.predict(u);

// 		if(first_guess_estimated(3)<1.)
// 			step=1;
// 		else if(first_guess_estimated(3)<3.)
// 			step=2;
// 		else if(first_guess_estimated(3)<6.)
// 			step=3;
// 		else
// 			step=4;
		
// 		if(first_guess_estimated(3)<1.)
// 			step=5;
// 		else if(first_guess_estimated(3)<3.)
// 			step=4;
// 		else if(first_guess_estimated(3)<6.)
// 			step=2;
// 		else
// 			step=1;
		
// 		cout<<"STEP "<<step<<endl;
		
// 		if(timedisp)printf("Ray comp %4.2f\n",(carmen_get_time()-dtsm)*1000.);

		if(h_rays.data->total>step)
		{
			l_rays=(ray_definition*)cvGetSeqElem(h_rays.data,step-1);
			time_interval=c_rays.cfg.timestamp-l_rays->cfg.timestamp;
		}
		
		cout<<"Time interval "<<time_interval<<endl;
		
		if(time_interval>0.05)
		{
			cout<<endl<<endl<<"Jumping messages"<<endl<<endl<<endl;
		}
		
		dtsm=ros::Time::now().toSec();
		
		/*Clean a small zone in the scan in the curves*/
		CleanZone(&c_rays,first_guess_estimated(5));
		CleanZone(l_rays,first_guess_estimated(5));
		
		switch(scan_method)
		{
			case MBICP:
				RaysToMbICP(&c_rays,fp1);
				RaysToMbICP(l_rays,fp2);
				break;
				
			case PSM:
				RaysToPMScan(&c_rays,&pm1);
				RaysToPMScan(l_rays,&pm2);
				break;
				
			case PLICP:
				RaysToLDP(&c_rays,ldp_1,1);
				RaysToLDP(l_rays,ldp_2,1);
				break;
		}
		
		/*******************************************/
		/* Common zone *****************************/
		
		dtsm=ros::Time::now().toSec();
		
		ConvertEstimatedToMeasurment(first_guess_estimated(3),first_guess_estimated(5),&sensorMotion.x,&sensorMotion.y,&sensorMotion.tita,time_interval,gl,bwa);

		/*******************************************/
		
		invalid=false;
		
		double flip;
		int np1=0,np2=0;
		
		switch(scan_method)
		{
			case MBICP:
				break;
				if(fabs(first_guess_estimated(5))>0.35)
					params.filter=0.95;
				else if(fabs(first_guess_estimated(5))>0.25)
					params.filter=0.90;
				else if(fabs(first_guess_estimated(5))>0.20)
					params.filter=0.85;
				else
					params.filter=0.85;
				
// 				else if(fabs(first_guess_estimated(5))>0.20)
// 					params.filter=0.80;
// 				else
// 					params.filter=0.90;
				
// 				if(fabs(first_guess_estimated(5))>0.25)
// 					params.filter=0.6;
// 				else
// 					params.filter=0.8;
				
				dtsm=ros::Time::now().toSec();
				if(!flags.fi)
					ret = MbICPmatcher(fp1,fp2,&sensorMotion,&solution);
				
				rtMbICP=(ros::Time::now().toSec()-dtsm)*1000;
				
				switch(ret)
				{
					case 1:
// 						printf("%c[1mMbICP%c[0m All ok\n",27,27);
						break;
					case 2:
						printf("%c[1mMbICP%c[0m Max iterrations reached\n",27,27);
						invalid=true;
						memset(&solution,0,sizeof(Tsc));
						break;
					case -1:
						printf("%c[1mMbICP%c[0m Failed in association\n",27,27);
						invalid=true;
						break;
					case -2:
						printf("%c[1mMbICP%c[0m Failed in minimization\n",27,27);
						invalid=true;
						break;
					default:
						printf("%c[1mMbICP%c[0m Error code %d\n",27,27,ret);
						invalid=true;
						memset(&solution,0,sizeof(Tsc));
						break;
				}

				runtime_MbICP.push_back(rtMbICP);

				solution.x=fabs(solution.x)<0.0005?0:solution.x;
				solution.y=fabs(solution.y)<0.0005?0:solution.y;
				solution.tita=fabs(solution.tita)<0.0005?0:solution.tita;

				break;
			case PSM:
				pm_preprocessScan(&pm1);
				pm_preprocessScan(&pm2);
				
				//match the last agains the current
// 				pm1.rx=-sensorMotion.y*100.;
// 				pm1.ry=sensorMotion.x*100.;
// 				pm1.th=-sensorMotion.tita;
				
				pm1.rx=0;
				pm1.ry=0;
				pm1.th=0;
				
// 				printf("FG %f %f %f\n",pm1.rx,pm1.ry,pm1.th);
				
				pm2.rx=-sensorMotion.y*100.;
				pm2.ry=sensorMotion.x*100.;
				pm2.th=-sensorMotion.tita;
				
// 				pm2.rx=0;
// 				pm2.ry=0;
// 				pm2.th=0;
				
				dtsm=ros::Time::now().toSec();
				
				for(int i=0;i<PM_L_POINTS;i++)
				{
					if(!pm1.bad[i])np1++;
					if(!pm2.bad[i])np2++;
				}
				
				if(!flags.fi && np1!=0 && np2!=0)
				{
					try{
						pm_psm(&pm2,&pm1);
					}catch(int e){
						printf("%c[1mPM_PSM throw an error!%c[0m %d\n",27,27,e);
					}
				}
				
				rtPSM=(ros::Time::now().toSec()-dtsm)*1000;
				
				runtime_PSM.push_back(rtPSM);
				
				flip=pm1.rx;
				
				pm1.rx=pm1.ry/100.;
				pm1.ry=-flip/100.-0.003;
				pm1.th=-pm1.th;
				
				solution.x=fabs(pm1.rx)<0.001?0:pm1.rx;
				solution.y=fabs(pm1.ry)<0.001?0:pm1.ry;
				solution.tita=fabs(pm1.th)<0.001?0:pm1.th;
				break;
				
			case PLICP:
				ConfigurePLICP(&params_sm,ldp_1,ldp_2);
				params_sm.first_guess[0]=sensorMotion.x;
				params_sm.first_guess[1]=sensorMotion.y;
				params_sm.first_guess[2]=sensorMotion.tita;
			
				dtsm=ros::Time::now().toSec();
				sm_icp(&params_sm, &results_sm);
				rtPlICP=(ros::Time::now().toSec()-dtsm)*1000;
				
				runtime_PlICP.push_back(rtPlICP);
				
				solution.x=fabs(results_sm.x[0])<0.0005?0:results_sm.x[0];
				solution.y=fabs(results_sm.x[1])<0.0005?0:results_sm.x[1];
				solution.tita=fabs(results_sm.x[2])<0.0005?0:results_sm.x[2];
				break;
		}
		
		/*******************************************/
		/* Common zone *****************************/
		
		CreateMeasurementFromDisplacement(solution.x,solution.y,solution.tita,z,time_interval,motion_model.l,bwa);
		
		//PlICP
// 		if(sm_method==PLICP)
// 			z[1]=z[1]-0.005;
		
		if(scan_method==PLICP)
			z[1]=z[1]-0.009;
		
// 		if(sm_method==MBICP)
// 			z[1]=z[1]+0.002;
		
		//Truncate invalid measurments
		
		if(z[0]<-10.)
			z[0]=-10.;
		
		if(z[0]>33.)
			z[0]=33.;
		
		if(z[1]<-0.43)//.43
			z[1]=-0.43;
		
		if(z[1]>0.43)
			z[1]=0.43;
		
		if(scan_method==MBICP)
		{
			if(fabs(z[1])>0.41)
				z[1]=z[1]>0?0.41:-0.41;
		}
		
		if(fabs(z[0])<0.02)
			z[1]=0;
		
		/*******************************************/
		double z_ori[2]={z[0],z[1]};
			
		
// 		double faultDir=0.05;
// 		double faultSpeed=0.8;
		
// 		double faultDir=0.05;//alboi_2
// 		double faultSpeed=1.2;
		
		double faultDir=0.1;//alboi_1
		double faultSpeed=1.2;

		bool faultA=fabs(z[1]-first_guess_estimated(5))>faultDir;//direction
		bool faultB=fabs(z[0]-first_guess_estimated(3))>faultSpeed;//speed
		bool fault= faultA || faultB;

		//counter 2 3
		
		if( fault && counter<2 && !invalid)
		{
			if(faultA && faultB)
				printf("%c[1mFault AB%c[0m\n",27,27);
			else if(faultA)
				printf("%c[1mFault A%c[0m\n",27,27);
			else
				printf("%c[1mFault B%c[0m\n",27,27);
			
			invalid=true;
			counter++;
		}else if( fault && counter<3)
		{
			counter++;
			printf("%c[1mCorrecting%c[0m\n",27,27);
			
			if(faultA)
			{
				if(z[1]>first_guess_estimated(5))
				{
					printf("Steering weel Truncating SW rate of change, positive high\n");
					z[1]=first_guess_estimated(5)+faultDir;
				}else
				{
					printf("Steering weel Truncating SW rate of change, negative high\n");
					z[1]=first_guess_estimated(5)-faultDir;
				}
			}
			
			if(faultB)
			{
				if(z[0]>first_guess_estimated(3))
				{
					printf("Linear Speed Truncating LS rate of change, positive high\n");
					z[0]=first_guess_estimated(3)+faultSpeed;
				}else
				{
					printf("Linear Speed Truncating LS rate of change, negative high\n");
					z[0]=first_guess_estimated(3)-faultSpeed;
				}
			}
		}else
			counter=0;
		
// 		AfterMeasurement:
		
		if(!invalid)//if valid
		{
			printf("%c[1mFg   %c[0m %6.4f %6.4f %6.4f\n",27,27,sensorMotion.x,sensorMotion.y,sensorMotion.tita);
			printf("%c[1mSM   %c[0m %6.4f %6.4f %6.4f\n",27,27,solution.x,solution.y,solution.tita);
			printf("%c[1mDiff %c[0m %6.4f %6.4f %6.4f\n",27,27,fabs(solution.x-sensorMotion.x),fabs(solution.y-sensorMotion.y),fabs(solution.tita-sensorMotion.tita));
			printf("\n");
			
		}else//if invalid
		{
			printf("%c[1mMbICP%c[0m Invalid solution, resetting to prediction\n",27,27);
			
			z[0]=first_guess_estimated(3);
			z[1]=first_guess_estimated(5);
			
			//Truncate invalid measurments
			if(z[0]<-10.)
				z[0]=-10.;
			
			if(z[0]>33.)
				z[0]=33.;
			
			if(z[1]<-0.5)
				z[1]=-0.5;
			
			if(z[1]>0.5)
				z[1]=0.5;
		}
		
		if(path_laser.size()<20)
		{
			z[0]=z_ori[0];
			z[1]=z_ori[1];
		}
		
// 		printf("%c[1mPlICP%c[0m Num Iterations %d\n",27,27,results_sm.iterations);
// 		printf("%c[1mPlICP%c[0m Valid correspondences %d\n",27,27,results_sm.nvalid);
		
// 		printf("%c[1mPlICP%c[0m Error %f\n",27,27,results_sm.error);
		
		zekf(1)=z[0];
		zekf(2)=z[1];
		
		motion_model.step(u,zekf);
		
		Vector x_corrected=motion_model.getX();
		
		t_posePtr path_node(new t_pose);

		path_node->phi=x_corrected(5);
		
		path_node->x=x_corrected(1);
		path_node->y=x_corrected(2);
		path_node->vl=x_corrected(3);
		path_node->orientation=x_corrected(4);
		
		path_node->m_vl=z[0];
		path_node->m_phi=z[1];
		
// 		path_node->timestamp=point_cloud_2.header.stamp.toSec();
        ROS_ERROR("PCL timestamp not correct, error during migration lar3 to lar4");
		
		path_laser.push_back(path_node);
		
		
		/**************************************************/
		/*Save to file*/
		static bool innnn=true;
		if(innnn)
		{
			ret=::system("rm /home/atlas/Desktop/XY_laser.txt");
			ret=::system("rm /home/atlas/Desktop/Laser_Pose.txt");
			ret=::system("rm /home/atlas/Desktop/Filter.txt");
			ret=::system("rm /home/atlas/Desktop/RunTime.txt");
			innnn=false;
		}
		
		ofstream Path("/home/atlas/Desktop/XY_laser.txt",ios::app);
		Path<<path_node->x<<" "<<path_node->y<<endl;
		
		ofstream Pose("/home/atlas/Desktop/Laser_Pose.txt",ios::app);
		Pose<<*path_node<<endl;
		
		ofstream Filter("/home/atlas/Desktop/Filter.txt",ios::app);
		Filter<<x_corrected(1)<<" "<<x_corrected(2)<<" "<<x_corrected(3)<<" "<<x_corrected(4)<<" "<<x_corrected(5)<<" "<<x_corrected(6)<<endl;
		
		ofstream RunTime("/home/atlas/Desktop/RunTime.txt",ios::app);
		
		switch(scan_method)
		{
			case MBICP:
				RunTime<<rtMbICP<<endl;
				break;
				
			case PSM:
				RunTime<<rtPSM<<endl;
				break;
				
			case PLICP:
				RunTime<<rtPlICP<<endl;
				break;
		}
		
		/**************************************************/
		
		dtsm=ros::Time::now().toSec();
		
		PlotSpace.SetPath(0,path_odo,"Map");		
		PlotSpace.SetSecondaryPath(0,path_laser,PL_RED);
		
		PlotSpace.SetMember_UseTimestamp(1,path_odo,get_vl,PL_BLUE,make_pair(-1.0,15.0),"Time (s)","Linear Speed (m/s)","Linear Speed",20.0);
		PlotSpace.SetSecondaryPlot(1,path_laser,get_m_vl,PL_CYAN,20.0);
		PlotSpace.SetThirdPlot(1,path_laser,get_vl,PL_RED,20.0);
		
// 		PlotSpace.SetMember_UseTimestamp(2,path_odo,get_m_phi,PL_BLUE,make_pair(-0.5,0.5),"Time (s)","Phi (rad)","Steering wheel",20.0);
// 		PlotSpace.SetSecondaryPlot(2,path_laser,get_m_phi,PL_CYAN,20.0);
// 		PlotSpace.SetThirdPlot(2,path_laser,get_phi,PL_RED,20.0);
		
		PlotSpace.SetMember_UseTimestamp(2,path_odo,get_orientation,PL_BLUE,make_pair(-5,5),"Time (s)","yaw (rad)","YAW",20.0);
		PlotSpace.SetSecondaryPlot(2,path_laser,get_orientation,PL_YELLOW,20.0);
// 		PlotSpace.SetThirdPlot(2,path_laser,get_phi,PL_RED,20.0);
		
		
		switch(scan_method)
		{
			case MBICP:
				PlotSpace.SetRunTime(3,runtime_MbICP,PL_YELLOW,"RunTime MbICP",40.,500.);
				break;
			case PLICP:
				PlotSpace.SetRunTime(3,runtime_PlICP,PL_YELLOW,"RunTime PlICP",40.,500.);
				break;
			case PSM:
				PlotSpace.SetRunTime(3,runtime_PSM,PL_YELLOW,"RunTime PSM",40.,500.);
				break;
		}
		
		PlotSpace.Plot();
		
		AddToHistory(&c_rays,&h_rays);
		
		double fps=get_fps(ros::Time::now().toSec()-t_elapsed,&mean_fps);
		t_elapsed=ros::Time::now().toSec();
		printf("Fps %.1lf   \r",fps); fflush(stdout);
					
		flags.fi=false;
	}
	
	return 0;
}
