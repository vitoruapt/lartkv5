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
#include <navigability_map/navigability_map.h>

/**
  \brief Library functions for Navigability_Map.
  This library provides a stack of function for a Accessibility Map.
  \file navigability_map.cpp
  \author Diogo Matos
  \date June 2013
 */

using namespace std;
using namespace cv;

void Navigability_Map::Filter_PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_filter)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	//Xfilter
	pass.setInputCloud (cloud_in);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (Xmin_filter, Xmax_filter); 
	pass.filter (*pc_filter);
	//Yfilter
	pass.setInputCloud (pc_filter);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (Ymin_filter, Ymax_filter); 
	pass.filter (*pc_filter);
	//zfilter
	pass.setInputCloud (pc_filter);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (Zmin_filter, Zmax_filter);
	pass.filter (*pc_filter);	
	
	////////              ///////////
	///////  DEBUG STUF  ////////////
	//////			    ////////////
	
			// 	// Car filter ///
			// 	pc_filter=cloud_in;
			// 	pcl::PointCloud<pcl::PointXYZ>::Ptr aux_Xcloud (new pcl::PointCloud<pcl::PointXYZ>);
			// 	pass.setInputCloud (pc_filter);
			// 	pass.setFilterFieldName ("x");
			// 	pass.setFilterLimits (0, 3.5);
			// 	pass.setFilterLimitsNegative (true);
			// 	pass.filter (*aux_Xcloud);
			// 	
			// 	pcl::PointCloud<pcl::PointXYZ>::Ptr aux_Ycloud (new pcl::PointCloud<pcl::PointXYZ>);
			// 	pass.setInputCloud (pc_filter);
			// 	pass.setFilterFieldName ("y");
			// 	pass.setFilterLimits (-1.2, 1.2);
			// 	pass.setFilterLimitsNegative (true);
			// 	pass.filter (*aux_Ycloud);
			// 	
			// 	*pc_filter=*aux_Ycloud+*aux_Xcloud;
				
			// 	//save cloud
			// 	sensor_msgs::PointCloud2 msg;
			// 	pcl::PointCloud<pcl::PointXYZ> pc;
			// 	pc=*pc_filter;
			// 	pcl::toROSMsg(pc, msg);
			// 	std::string name = "cenas.pcd";
			// 	pcl::io::savePCDFile(name.c_str(), msg);
}

void Navigability_Map::Normal_Estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	normals.reset(new pcl::PointCloud<pcl::Normal>);
// 	Normal estimation
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	
	n.useSensorOriginAsViewPoint();
	n.setViewPoint (0.8, 0, 4); //attention to this !!!!!!!!!!!!!!!!!!!!! , this the position in of the laser scan in the car
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	
	//neighbours search method
	if (Use_Radius_Search)
		n.setRadiusSearch(Radius_neighbors);
	else
		n.setKSearch (K_neighbors);
	//compute
	n.compute (*normals);
	
	////////              ///////////
	///////  DEBUG STUF  ////////////
	//////			    ////////////
	
			// 	//save normals pcd file
			// 	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
			// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cenas (new pcl::PointCloud<pcl::PointXYZ>);
			// 	cenas=cloud;
			// 	pcl::concatenateFields (*cenas, *normals, *cloud_with_normals);
			// 	sensor_msgs::PointCloud2 msg;
			// 	pcl::PointCloud<pcl::PointNormal> pc;
			// 	pc=*cloud_with_normals;
			// 	pcl::toROSMsg(pc, msg);
			// 	std::string name = "cenas.pcd";
			// 	pcl::io::savePCDFile(name.c_str(), msg);
}
	
void Navigability_Map::setGrid_parameter(double max_Xval,double max_Yval)
{
	//_____________________
	//|                    |
	//|  Save Total Row    |
	//|____________________|
	total_row=(int)roundl((max_Xval/Sx)+0.5); //+0.5 to round up 
	//______________________
	//|                    |
	//|   Get Center Col   |
	//|____________________|
	int auxCenterCol=(int)roundl((max_Yval/Sy)+0.5); //+0.5 to round up 
	int center_col;
	if (auxCenterCol % 2) //auxCenterCol is odd  
		center_col=auxCenterCol;	
	else
		center_col=auxCenterCol+1;
	//______________________
	//|                    |
	//|  Save Total Col    |
	//|____________________|
	total_col=center_col*2+1;
	//______________________
	//|                    |
	//|  Save Center Col   |
	//|____________________|
	CARaxis_col=center_col;	 //Grid center is coincident with the axis
}

void Navigability_Map::inicialize_grid(void)
{	
	//first resize the Grid acording to the parameters
	grid.resize(total_row,total_col);
	grid_nodePtr node_data;
	
	for(uint row=0;row<grid.rows();row++)
	{
		for(uint col=0;col<grid.cols();col++)
		{
			node_data.reset(new grid_node);
			grid(row,col)=node_data;
		}
	}
}


void Navigability_Map::setGrid_data(pcl::PointCloud<pcl::PointXYZ>& cloud)
{		
	int row_pos;
	int col_pos;
	for (size_t i = 0; i < cloud.points.size (); ++i)
	{
		//do not save car data 
		if (cloud.points[i].x<3.8 && (cloud.points[i].y<1.4 && cloud.points[i].y>-1.4))
			continue;
		
		//Point posistion in the Grid
		//x
		row_pos=(int)(cloud.points[i].x/Sx);
		//y
		if (cloud.points[i].y>0)
			col_pos=CARaxis_col+(int)((cloud.points[i].y/Sy)+0.5); //axis car is the center of the mid cell so +0.5 
		else
			col_pos=CARaxis_col+(int)((cloud.points[i].y/Sy)-0.5); //axis car is the center of the mid cell so -0.5 
		
		//pointer to node_data
		grid_nodePtr node_data= grid(row_pos,col_pos);
		
		// ___________________________________________
		//|                                          |
		//| Matrix Points and Direction_Cosine       |
		//|__________________________________________|
			
		node_data->Matrix_Points.conservativeResize(node_data->Matrix_Points.rows()+1,3);
		
		//Set data to Matrix
		
		//Points
		node_data->Matrix_Points(node_data->Matrix_Points.rows()-1,0)=(double)cloud.points[i].x;
		node_data->Matrix_Points(node_data->Matrix_Points.rows()-1,1)=(double)cloud.points[i].y;
		node_data->Matrix_Points(node_data->Matrix_Points.rows()-1,2)=(double)cloud.points[i].z;
		
		//Angle
		if (!isnan(normals->points[i].normal[0]) && !isnan(normals->points[i].normal[1]) && !isnan(normals->points[i].normal[2]))
		{	
			node_data->Angles.conservativeResize(node_data->Angles.rows()+1,3);
			
			node_data->Angles(node_data->Angles.rows()-1,0)=acos((double)normals->points[i].normal[0]);
			node_data->Angles(node_data->Angles.rows()-1,1)=acos((double)normals->points[i].normal[1]);
			node_data->Angles(node_data->Angles.rows()-1,2)=acos((double)normals->points[i].normal[2]);

			node_data->has_normal=true;
		}
		
		//num_points
		node_data->num_points=node_data->Matrix_Points.rows();
		
	}
}

void Navigability_Map::calcGrid_data(void)
{
	grid_nodePtr node_data;

	for(uint row=0;row<grid.rows();row++)
	{
		for(uint col=0;col<grid.cols();col++)
		{				
			node_data=grid(row,col);

			double med_z=0;		
			double standard_deviance_z=0;
			
			//angles
			double standard_deviance_anglex=0;
			double standard_deviance_angley=0;
			double standard_deviance_anglez=0;
			double med_angle_X=0;
			double med_angle_Y=0;
			double med_angle_Z=0;

			if (node_data->num_points>0)
			{
				//get points
				for (uint i=0; i<node_data->Matrix_Points.rows(); i++)
					med_z+=node_data->Matrix_Points(i,2);	

					//Calc mean
					med_z/=node_data->num_points;
			
					//set data to the grid (mean val)
					node_data->Zmed=med_z;
					
					//Calc standard_deviance for Z
					if (node_data->num_points>1)
					{
						for (uint i=0; i<node_data->Matrix_Points.rows(); i++)
							standard_deviance_z+=pow(node_data->Matrix_Points(i,2)-med_z,2);
						
						standard_deviance_z=sqrt(standard_deviance_z/(node_data->num_points-1));
						
						//set data to the grid (standard_deviance_z)
						node_data->Standard_Deviation_Z=standard_deviance_z;
					
					}//endl Calc standard_deviance for Z
					
					/************* Calc Z_confidence *************/
						
					if (node_data->num_points>1)
						node_data->Z_confidence=1-(node_data->Standard_Deviation_Z/Standard_Deviation_max); //factor
					else
						node_data->Z_confidence=default_confidence; //for cell with only 1 points give a residual confidence
					
					if (node_data->Z_confidence<0)
						node_data->Z_confidence=0;
						
					
					/************* calc med Angle *************/
					
					if (node_data->has_normal)
					{
						
						for (uint i=0; i<node_data->Angles.rows(); i++)
						{						
							med_angle_X+=node_data->Angles(i,0);
							med_angle_Y+=node_data->Angles(i,1);
							med_angle_Z+=node_data->Angles(i,2);
							
						}

						med_angle_X/=node_data->Angles.rows();
						med_angle_Y/=node_data->Angles.rows();
						med_angle_Z/=node_data->Angles.rows();
						
						//set data
						node_data->med_angle_X=med_angle_X;
						node_data->med_angle_Y=med_angle_Y;
						node_data->med_angle_Z=med_angle_Z;

						//Calc standard_deviance Angles
						if (node_data->num_points>1)
						{
							for (uint i=0; i<node_data->Angles.rows(); i++)
							{
								standard_deviance_anglex+=pow(node_data->Angles(i,0)-med_angle_X,2);
								standard_deviance_angley+=pow(node_data->Angles(i,1)-med_angle_Y,2);
								standard_deviance_anglez+=pow(node_data->Angles(i,2)-med_angle_Z,2);
								
							}
							
							standard_deviance_anglex=sqrt(standard_deviance_anglex/(node_data->Angles.rows()-1));
							standard_deviance_angley=sqrt(standard_deviance_angley/(node_data->Angles.rows()-1));
							standard_deviance_anglez=sqrt(standard_deviance_anglez/(node_data->Angles.rows()-1));
							
							//set data to the grid

							double Angle_confidence_x=1-(standard_deviance_anglex/Standard_Deviation_anglex_max_confidence);
							double Angle_confidence_y=1-(standard_deviance_angley/Standard_Deviation_angley_max_confidence);
							double Angle_confidence_z=1-(standard_deviance_anglez/Standard_Deviation_anglez_max_confidence);

							if (Angle_confidence_x<0)
								node_data->angle_confidence_x=0;
							else
								node_data->angle_confidence_x=Angle_confidence_x;
							if (Angle_confidence_y<0)
								node_data->angle_confidence_y=0;
							else
								node_data->angle_confidence_y=Angle_confidence_y;
							if (Angle_confidence_z<0)
								node_data->angle_confidence_z=0;
							else
								node_data->angle_confidence_z=Angle_confidence_z;

						}
						else //set a residual value
						{
							node_data->angle_confidence_x=default_confidence;
							node_data->angle_confidence_y=default_confidence;
							node_data->angle_confidence_z=default_confidence;
	
						}
						
						
					}//end if (node_data->has_normal) //// calc med Angle
					
			}//end node_data->num_points>0
			
		}//end for col
	}//end for row
}


void Navigability_Map::fill_data_cells(void)
{
	std::vector<grid_nodePtr> vector_node;
	grid_nodePtr node_data;
	grid_nodePtr aux_data;
	int empty_neighbours=0;
	int number_neighbours=0;
	
	std::vector<double> Zval_neighbours;
	std::vector<double> angle_x_neighbours;
	std::vector<double> angle_y_neighbours;
	std::vector<double> angle_z_neighbours;
	
	std::vector<double> Zconfidence_neighbours;
	std::vector<double> angle_x_confidence_neighbours;
	std::vector<double> angle_y_confidence_neighbours;
	std::vector<double> angle_z_confidence_neighbours;
	
	for(uint row=0;row<grid.rows();row++)
	{
		for(uint col=0;col<grid.cols();col++)
		{
			Zval_neighbours.erase(Zval_neighbours.begin(), Zval_neighbours.end());
			Zconfidence_neighbours.erase(Zconfidence_neighbours.begin(), Zconfidence_neighbours.end());
			
			angle_x_neighbours.erase(angle_x_neighbours.begin(), angle_x_neighbours.end());
			angle_y_neighbours.erase(angle_y_neighbours.begin(), angle_y_neighbours.end());
			angle_z_neighbours.erase(angle_z_neighbours.begin(), angle_z_neighbours.end());
			
			angle_x_confidence_neighbours.erase(angle_x_confidence_neighbours.begin(), angle_x_confidence_neighbours.end());
			angle_y_confidence_neighbours.erase(angle_y_confidence_neighbours.begin(), angle_y_confidence_neighbours.end());
			angle_z_confidence_neighbours.erase(angle_z_confidence_neighbours.begin(), angle_z_confidence_neighbours.end());
			
			empty_neighbours=0;
			number_neighbours=0;
			
			//angle
			double med_angle_X=0;
			double med_angle_Y=0;
			double med_angle_Z=0;
			double angle_x_confidence=0;
			double angle_y_confidence=0;
			double angle_z_confidence=0;
			int number_normal_neighbours=0;
			
			bool neighbours_has_normal=false;
			
			node_data=grid(row,col);
			
			// ___________________________
			//|                          |
			//|        Empty Cell        |
			//|__________________________|
			
			if (node_data->num_points==0) 
			{
				vector_node=cell_neighbours(row,col,empty_neighbours,number_neighbours);
				
				if (!empty_neighbours && number_neighbours>3) //this cell has neighbours so lets set data acording to them
				{	
					//acess the data in the cell neighbours
					for (uint i=0; i<8; i++)
					{
						aux_data=vector_node[i];
						
						// ___________________________
						//|                          |
						//|        Get Normal        |
						//|__________________________|
						if (aux_data->has_normal && !aux_data->interpolate_normal_data)
						{
							neighbours_has_normal=true;
							
// 							//angle
							med_angle_X+=aux_data->med_angle_X;
							med_angle_Y+=aux_data->med_angle_Y;
							med_angle_Z+=aux_data->med_angle_Z;

// 							angle mediana
							angle_x_neighbours.push_back(aux_data->med_angle_X);
							angle_y_neighbours.push_back(aux_data->med_angle_Y);
							angle_z_neighbours.push_back(aux_data->med_angle_Z);

							//confidence
							angle_x_confidence+=aux_data->angle_confidence_x;
							angle_y_confidence+=aux_data->angle_confidence_y;
							angle_z_confidence+=aux_data->angle_confidence_z;
							
							angle_x_confidence_neighbours.push_back(aux_data->angle_confidence_x);
							angle_y_confidence_neighbours.push_back(aux_data->angle_confidence_y);
							angle_z_confidence_neighbours.push_back(aux_data->angle_confidence_z);
							
							number_normal_neighbours++;
						}
						
						// _________________________________________
						//|                                         |
						//|        Get Z val  &   Z Confidence      |
						//|_________________________________________|
						
						if (aux_data->num_points>0)
						{
							Zval_neighbours.push_back(aux_data->Zmed);
							Zconfidence_neighbours.push_back(aux_data->Z_confidence);
						}

					}//end for
					
					// ___________________________
					//|                          |
					//|      use zmediana        |
					//|__________________________|
					
					std::sort (Zval_neighbours.begin(), Zval_neighbours.end());
					std::sort (Zconfidence_neighbours.begin(), Zconfidence_neighbours.end());
					
					double Zmediana=0;
					double Zmediana_confidence=0;
					if (Zval_neighbours.size() % 2) /* is odd */ 
					{
						Zmediana=Zval_neighbours.at(((Zval_neighbours.size()+1)/2)-1);	
						Zmediana_confidence=Zconfidence_neighbours.at(((Zconfidence_neighbours.size()+1)/2)-1);	
					}
					else
					{
						int pos_vect1=((Zval_neighbours.size()+1)/2)-1;
						int pos_vect2=(Zval_neighbours.size()+1)/2;
						
						Zmediana=(Zval_neighbours.at(pos_vect1)+Zval_neighbours.at(pos_vect2))/2;
						Zmediana_confidence=(Zconfidence_neighbours.at(pos_vect1)+Zconfidence_neighbours.at(pos_vect2))/2;
					}		
					
// 					//set the data to this new node
					node_data->Zmed=Zmediana;
					node_data->Z_confidence=Zmediana_confidence;
					node_data->interpolate_z_data=true;
					
// 					// ___________________________
// 					//|                          |
// 					//|  Set the mediana normal  |
// 					//|__________________________|
					
					if (neighbours_has_normal && number_normal_neighbours>1)
					{
						std::sort (angle_x_neighbours.begin(), angle_x_neighbours.end());
						std::sort (angle_y_neighbours.begin(), angle_y_neighbours.end());
						std::sort (angle_z_neighbours.begin(), angle_z_neighbours.end());
						
						std::sort (angle_x_confidence_neighbours.begin(), angle_x_confidence_neighbours.end());
						std::sort (angle_y_confidence_neighbours.begin(), angle_y_confidence_neighbours.end());
						std::sort (angle_z_confidence_neighbours.begin(), angle_z_confidence_neighbours.end());
						
						double xmediana=0;
						double ymediana=0;
						double zmediana=0;
						double x_confidence_mediana=0;
						double y_confidence_mediana=0;
						double z_confidence_mediana=0;
						if (angle_x_neighbours.size() % 2) /* is odd */ 
						{
							xmediana=angle_x_neighbours.at(((angle_x_neighbours.size()+1)/2)-1);	
							ymediana=angle_y_neighbours.at(((angle_y_neighbours.size()+1)/2)-1);	
							zmediana=angle_z_neighbours.at(((angle_z_neighbours.size()+1)/2)-1);	
							
							x_confidence_mediana=angle_x_confidence_neighbours.at(((angle_x_confidence_neighbours.size()+1)/2)-1);	
							y_confidence_mediana=angle_y_confidence_neighbours.at(((angle_y_confidence_neighbours.size()+1)/2)-1);	
							z_confidence_mediana=angle_z_confidence_neighbours.at(((angle_z_confidence_neighbours.size()+1)/2)-1);	
							
						}
						else
						{
							int pos_vect1=((angle_x_neighbours.size()+1)/2)-1;
							int pos_vect2=(angle_x_neighbours.size()+1)/2;
							
							xmediana=(angle_x_neighbours.at(pos_vect1)+angle_x_neighbours.at(pos_vect2))/2;
							ymediana=(angle_y_neighbours.at(pos_vect1)+angle_y_neighbours.at(pos_vect2))/2;
							zmediana=(angle_z_neighbours.at(pos_vect1)+angle_z_neighbours.at(pos_vect2))/2;
							
							x_confidence_mediana=(angle_x_confidence_neighbours.at(pos_vect1)+angle_x_confidence_neighbours.at(pos_vect2))/2;	
							y_confidence_mediana=(angle_y_confidence_neighbours.at(pos_vect1)+angle_y_confidence_neighbours.at(pos_vect2))/2;	
							z_confidence_mediana=(angle_z_confidence_neighbours.at(pos_vect1)+angle_z_confidence_neighbours.at(pos_vect2))/2;
							
						}
						
						node_data->med_angle_X=xmediana;
						node_data->med_angle_Y=ymediana;
						node_data->med_angle_Z=zmediana;

						node_data->angle_confidence_x=x_confidence_mediana;
						node_data->angle_confidence_y=y_confidence_mediana;
						node_data->angle_confidence_z=z_confidence_mediana;
										
						node_data->has_normal=true;
						node_data->interpolate_normal_data=true;
						

					}
					else
					{
						node_data->has_normal=true;
						node_data->interpolate_normal_data=true;
						
						node_data->med_angle_X=med_angle_X/number_normal_neighbours;
						node_data->med_angle_Y=med_angle_Y/number_normal_neighbours;
						node_data->med_angle_Z=med_angle_Z/number_normal_neighbours;
						
						node_data->angle_confidence_x=angle_x_confidence/number_normal_neighbours;
						node_data->angle_confidence_y=angle_y_confidence/number_normal_neighbours;
						node_data->angle_confidence_z=angle_z_confidence/number_normal_neighbours;
					}			
					
				}//end (!empty_neighbours && number_neighbours>3) //this cell has neighbours so lets set data acording to them
				
			}//end node_data->num_points==0
			
			// ___________________________
			//|                          |
			//|  Cell with no normal     |
			//|__________________________|
			
			if (!node_data->has_normal && node_data->num_points>0) 
			{
				vector_node=cell_neighbours(row,col,empty_neighbours,number_neighbours);
				
				if (!empty_neighbours && number_neighbours>2) //this cell has neighbours so lets set data acording to them
				{
					//acess the data in the cell neighbours
					for (uint i=0; i<8; i++)
					{
						aux_data=vector_node[i];
						
						// ___________________________
						//|                          |
						//|        Get Normal        |
						//|__________________________|
						if (aux_data->has_normal && !aux_data->interpolate_normal_data)
						{

							med_angle_X+=aux_data->med_angle_X;
							med_angle_Y+=aux_data->med_angle_Y;
							med_angle_Z+=aux_data->med_angle_Z;
							
							//angle mediana
							angle_x_neighbours.push_back(aux_data->med_angle_X);
							angle_y_neighbours.push_back(aux_data->med_angle_Y);
							angle_z_neighbours.push_back(aux_data->med_angle_Z);
							
							//confidence
							angle_x_confidence+=aux_data->angle_confidence_x;
							angle_y_confidence+=aux_data->angle_confidence_y;
							angle_z_confidence+=aux_data->angle_confidence_z;
							
							angle_x_confidence_neighbours.push_back(aux_data->angle_confidence_x);
							angle_y_confidence_neighbours.push_back(aux_data->angle_confidence_y);
							angle_z_confidence_neighbours.push_back(aux_data->angle_confidence_z);
							
							number_normal_neighbours++;
						}			
					}
						
					// ___________________________
					//|                          |
					//|   Set the med normal     |
					//|__________________________|
					if (neighbours_has_normal && number_normal_neighbours>1)
					{			
						node_data->interpolate_normal_data=true;
						node_data->has_normal=true;
						
						// use mediana for angle
						std::sort (angle_x_neighbours.begin(), angle_x_neighbours.end());
						std::sort (angle_y_neighbours.begin(), angle_y_neighbours.end());
						std::sort (angle_z_neighbours.begin(), angle_z_neighbours.end());
						
						std::sort (angle_x_confidence_neighbours.begin(), angle_x_confidence_neighbours.end());
						std::sort (angle_y_confidence_neighbours.begin(), angle_y_confidence_neighbours.end());
						std::sort (angle_z_confidence_neighbours.begin(), angle_z_confidence_neighbours.end());
						
						double xmediana=0;
						double ymediana=0;
						double zmediana=0;
						double x_confidence_mediana=0;
						double y_confidence_mediana=0;
						double z_confidence_mediana=0;
						if (angle_x_neighbours.size() % 2) /* is odd */ 
						{
							xmediana=angle_x_neighbours.at(((angle_x_neighbours.size()+1)/2)-1);	
							ymediana=angle_y_neighbours.at(((angle_y_neighbours.size()+1)/2)-1);	
							zmediana=angle_z_neighbours.at(((angle_z_neighbours.size()+1)/2)-1);	
							
							
							x_confidence_mediana=angle_x_confidence_neighbours.at(((angle_x_confidence_neighbours.size()+1)/2)-1);	
							y_confidence_mediana=angle_y_confidence_neighbours.at(((angle_y_confidence_neighbours.size()+1)/2)-1);	
							z_confidence_mediana=angle_z_confidence_neighbours.at(((angle_z_confidence_neighbours.size()+1)/2)-1);	
						}
						else
						{
							int pos_vect1=((angle_x_neighbours.size()+1)/2)-1;
							int pos_vect2=(angle_x_neighbours.size()+1)/2;
							
							xmediana=(angle_x_neighbours.at(pos_vect1)+angle_x_neighbours.at(pos_vect2))/2;
							ymediana=(angle_y_neighbours.at(pos_vect1)+angle_y_neighbours.at(pos_vect2))/2;
							zmediana=(angle_z_neighbours.at(pos_vect1)+angle_z_neighbours.at(pos_vect2))/2;
							
								
							x_confidence_mediana=(angle_x_confidence_neighbours.at(pos_vect1)+angle_x_confidence_neighbours.at(pos_vect2))/2;	
							y_confidence_mediana=(angle_y_confidence_neighbours.at(pos_vect1)+angle_y_confidence_neighbours.at(pos_vect2))/2;	
							z_confidence_mediana=(angle_z_confidence_neighbours.at(pos_vect1)+angle_z_confidence_neighbours.at(pos_vect2))/2;
							

						}
						
													
						node_data->med_angle_X=xmediana;
						node_data->med_angle_Y=ymediana;
						node_data->med_angle_Z=zmediana;

						
						node_data->angle_confidence_x=x_confidence_mediana;
						node_data->angle_confidence_y=y_confidence_mediana;
						node_data->angle_confidence_z=z_confidence_mediana;

						
					}
					else
					{
						node_data->has_normal=true;
						node_data->interpolate_normal_data=true;
						
						node_data->med_angle_X=med_angle_X/number_normal_neighbours;
						node_data->med_angle_Y=med_angle_Y/number_normal_neighbours;
						node_data->med_angle_Z=med_angle_Z/number_normal_neighbours;
						
						node_data->angle_confidence_x=angle_x_confidence/number_normal_neighbours;
						node_data->angle_confidence_y=angle_y_confidence/number_normal_neighbours;
						node_data->angle_confidence_z=angle_z_confidence/number_normal_neighbours;
					}
						
				}//end (!empty_neighbours && number_neighbours>2) //this cell has neighbours so lets set data acording to them
				
			}//end (!node_data->has_normal && node_data->num_points>0) 	
			
		}//end for col
	}//end for row
}


void Navigability_Map::set_Cells_accessibility(void)
{
	std::vector<grid_nodePtr> vector_node;
	grid_nodePtr node_data;
	grid_nodePtr aux_data;
	int empty_neighbours=0;
	int number_neighbours=0;
	
	double z_accessibility=0;
	
	double angleX_accessibility=0;
	double angleY_accessibility=0;
	double angleZ_accessibility=0;
	
	
	for(uint row=0;row<grid.rows();row++)
	{
		for(uint col=0;col<grid.cols();col++)
		{
			node_data=grid(row,col);
			
			double neighbours_full_data=0;
			double z_difference_med_confidence=0;
			double angleX_difference_med_confidence=0;
			double angleY_difference_med_confidence=0;
			double angleZ_difference_med_confidence=0;

			// ________________________________________________________________________________
			//|                          					                                    |
			//| Cell is occupied (with Z and normal) or has interpolated data (Z and normal)  |
			//|______________________________________________________________________________|
			
			if ((node_data->num_points>0 && node_data->has_normal) || (node_data->interpolate_z_data && node_data->interpolate_normal_data) || (node_data->num_points>0 && node_data->interpolate_normal_data)) 
			{
				vector_node=cell_neighbours(row,col,empty_neighbours,number_neighbours);
				//acess the data in the cell neighbours
				for (uint i=0; i<8; i++)
				{
					aux_data=vector_node[i];
					
					//Z difference
					node_data->z_difference_neighbours.push_back(abs(aux_data->Zmed-node_data->Zmed));
					//Z confidence
					node_data->z_confidence_neighbours.push_back(sqrt(aux_data->Z_confidence*node_data->Z_confidence));
					
					//angle difference
					node_data->angleX_difference_neighbours.push_back(abs(aux_data->med_angle_X-node_data->med_angle_X));
					node_data->angleY_difference_neighbours.push_back(abs(aux_data->med_angle_Y-node_data->med_angle_Y));
					node_data->angleZ_difference_neighbours.push_back(abs(aux_data->med_angle_Z-node_data->med_angle_Z));
					//angle confidence
					node_data->angleX_confidence_neighbours.push_back(sqrt(aux_data->angle_confidence_x*node_data->angle_confidence_x));
					node_data->angleY_confidence_neighbours.push_back(sqrt(aux_data->angle_confidence_y*node_data->angle_confidence_y));
					node_data->angleZ_confidence_neighbours.push_back(sqrt(aux_data->angle_confidence_z*node_data->angle_confidence_z));
					
					//only for cell fully data
					if ((aux_data->num_points>0 && aux_data->has_normal) || (aux_data->interpolate_z_data && aux_data->interpolate_normal_data) || (aux_data->num_points>0 && aux_data->interpolate_normal_data))	
						node_data->has_fulldata_direction.push_back(true);
					else
						node_data->has_fulldata_direction.push_back(false);
				}
				
				double z_difference_med_confidence_aux=0;
				double angleX_difference_med_confidence_aux=0;
				double angleY_difference_med_confidence_aux=0;
				double angleZ_difference_med_confidence_aux=0;
				
				for (uint i=0; i<8; i++)
				{	
					if (node_data->has_fulldata_direction[i])
					{
					
						//Z
						z_difference_med_confidence_aux=node_data->z_difference_neighbours[i]/node_data->z_confidence_neighbours[i];
						if (z_difference_med_confidence_aux>fator_confidence_neighbour_limit*Zmax_heigh_difference)
							z_difference_med_confidence+=fator_confidence_neighbour_limit*Zmax_heigh_difference;
						else
							z_difference_med_confidence+=z_difference_med_confidence_aux;
						
						//angle
						angleX_difference_med_confidence_aux=node_data->angleX_difference_neighbours[i]/node_data->angleX_confidence_neighbours[i];	
						if (angleX_difference_med_confidence_aux>fator_confidence_neighbour_limit*angleX_max_difference)
							angleX_difference_med_confidence+=fator_confidence_neighbour_limit*angleX_max_difference;
						else
							angleX_difference_med_confidence+=angleX_difference_med_confidence_aux;
						
						angleY_difference_med_confidence_aux=node_data->angleY_difference_neighbours[i]/node_data->angleY_confidence_neighbours[i];	
						if (angleY_difference_med_confidence_aux>fator_confidence_neighbour_limit*angleY_max_difference)
							angleY_difference_med_confidence+=fator_confidence_neighbour_limit*angleY_max_difference;
						else
							angleY_difference_med_confidence+=angleY_difference_med_confidence_aux;
						
						angleZ_difference_med_confidence_aux=node_data->angleZ_difference_neighbours[i]/node_data->angleZ_confidence_neighbours[i];	
						if (angleZ_difference_med_confidence_aux>fator_confidence_neighbour_limit*angleZ_max_difference)
							angleZ_difference_med_confidence+=fator_confidence_neighbour_limit*angleZ_max_difference;
						else
							angleZ_difference_med_confidence+=angleZ_difference_med_confidence_aux;
						
						//neighbours
						neighbours_full_data++;
					}
						
				}
				
				
			// _______________________________________________________________________________
			//|                          					                                  |
			//|   calc z_accessibility , based on the med of the neighbours and confidence  |
			//|_____________________________________________________________________________|
				
				
				if (neighbours_full_data>0)
				{
					z_difference_med_confidence/=neighbours_full_data;
					z_accessibility=1-(abs(z_difference_med_confidence)/Zmax_heigh_difference);
					if (z_accessibility>=0)
						node_data->z_accessibility=z_accessibility;
					else
						node_data->z_accessibility=0;	
				
			// _______________________________________________________________________
			//|                          					                    		|
			//|   calc angle , based on the med of the neighbours and confidence 	    |
			//|_______________________________________________________________________|

					angleX_difference_med_confidence/=neighbours_full_data;
					angleX_accessibility=1-(abs(angleX_difference_med_confidence)/angleX_max_difference);
					
					if (angleX_accessibility>=0)
						node_data->angleX_accessibility=angleX_accessibility;
					else
						node_data->angleX_accessibility=0;
					
					angleY_difference_med_confidence/=neighbours_full_data;
					angleY_accessibility=1-(abs(angleY_difference_med_confidence)/angleY_max_difference);
					
					if (angleY_accessibility>=0)
						node_data->angleY_accessibility=angleY_accessibility;
					else
						node_data->angleY_accessibility=0;
					
					angleZ_difference_med_confidence/=neighbours_full_data;
					angleZ_accessibility=1-(abs(angleZ_difference_med_confidence)/angleZ_max_difference);
					
					if (angleZ_accessibility>=0)
						node_data->angleZ_accessibility=angleZ_accessibility;
					else
						node_data->angleZ_accessibility=0;
				}
						
				// ______________________________
				//|                          	 |
				//|     total  accessibility   |
				//|____________________________|
				
				node_data->total_accessibility=node_data->z_accessibility*node_data->angleX_accessibility*node_data->angleY_accessibility*node_data->angleZ_accessibility;

			}//end (node_data->num_points>0 || (node_data->interpolate_z_data && node_data->interpolate_normal_data) 
			
			
		}//end for col
		
	}//end for row
	
}


std::vector<grid_nodePtr> Navigability_Map::cell_neighbours(int row_pos, int col_pos, int &empty_neighbours, int &number_neighbours)
{
	std::vector<grid_nodePtr> vector_node;
	grid_nodePtr node_data;
	
	bool empty_cell=true;
	int counter=0;
	
	//4 main directions
	int N_pos=row_pos+1;
	int S_pos=row_pos-1;
	int E_pos=col_pos+1;
	int W_pos=col_pos-1;

	int max_row_pos=grid.rows()-1;
	int max_col_pos=grid.cols()-1;
	
	//  Acess N direction  
	
	if (N_pos>max_row_pos) //outside the grid
		node_data.reset(new grid_node); //set a default node
	else
		node_data=grid(N_pos,col_pos);
	//node has full data
	if (node_data->num_points>0 && node_data->has_normal && !node_data->interpolate_z_data && !node_data->interpolate_normal_data)
	{
		empty_cell=false;
		counter++;
	}
	
	vector_node.push_back(node_data);
	
	//  Acess NE direction
	
	if (N_pos>max_row_pos || E_pos>max_col_pos) //outside the grid
		node_data.reset(new grid_node); //set a default node
	else
		node_data=grid(N_pos,E_pos);
	
	if (node_data->num_points>0 && node_data->has_normal && !node_data->interpolate_z_data && !node_data->interpolate_normal_data)
	{
		empty_cell=false;
		counter++;
	}
	
	vector_node.push_back(node_data);
	
	//  Acess E direction  
	
	if (E_pos>max_col_pos) //outside the grid
		node_data.reset(new grid_node); //set a default node
	else
		node_data=grid(row_pos,E_pos);
	
	if (node_data->num_points>0 && node_data->has_normal && !node_data->interpolate_z_data && !node_data->interpolate_normal_data)
	{
		empty_cell=false;
		counter++;
	}
	
	vector_node.push_back(node_data);
	
	//  Acess SE direction
	
	if (S_pos<0 || E_pos>max_col_pos) //outside the grid
		node_data.reset(new grid_node); //set a default node
	else
		node_data=grid(S_pos,E_pos);
	
	if (node_data->num_points>0 && node_data->has_normal && !node_data->interpolate_z_data && !node_data->interpolate_normal_data)
	{
		empty_cell=false;
		counter++;
	}
	
	vector_node.push_back(node_data);
	
	//  Acess S direction  
	
	if (S_pos<0) //outside the grid
		node_data.reset(new grid_node); //set a default node
	else
		node_data=grid(S_pos,col_pos);
	
	if (node_data->num_points>0 && node_data->has_normal && !node_data->interpolate_z_data && !node_data->interpolate_normal_data)
	{
		empty_cell=false;
		counter++;
	}
	
	vector_node.push_back(node_data);
	
	//  Acess SW direction
	
	if (S_pos<0 || W_pos<0) //outside the grid
		node_data.reset(new grid_node); //set a default node
	else
		node_data=grid(S_pos,W_pos);
	
	if (node_data->num_points>0 && node_data->has_normal && !node_data->interpolate_z_data && !node_data->interpolate_normal_data)
	{
		empty_cell=false;
		counter++;
	}
	
	vector_node.push_back(node_data);
	
	//  Acess W direction  
	
	if (W_pos<0) //outside the grid
		node_data.reset(new grid_node); //set a default node
	else
		node_data=grid(row_pos,W_pos);
	
	if (node_data->num_points>0 && node_data->has_normal && !node_data->interpolate_z_data && !node_data->interpolate_normal_data)
	{
		empty_cell=false;
		counter++;
	}
	
	vector_node.push_back(node_data);
	
	//  Acess NW direction
	
	if (N_pos>max_row_pos || W_pos<0) //outside the grid
		node_data.reset(new grid_node); //set a default node
	else
		node_data=grid(N_pos,W_pos);
	
	if (node_data->num_points>0 && node_data->has_normal && !node_data->interpolate_z_data && !node_data->interpolate_normal_data)
	{
		empty_cell=false;
		counter++;
	}
	
	vector_node.push_back(node_data);
	
	//return
	
	if (empty_cell)
		empty_neighbours=1;
	
	number_neighbours=counter;
	
	return vector_node;
	
}


void Navigability_Map::polygon_groundtruth(void)
{
	geometry_msgs::Point p;
	
// 	//________________________
// 	//                        |
// 	//  6 cloud ground truth  |
// 	//________________________|
// 	

			p.x=4.4;
			p.y=-2.3;
			p.z=1.2;
			polygon_points.push_back(p);	
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=4.4;
			p.y=3.5;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			//junto ao passeio
			p.x=7;
			p.y=7.6;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=19;
			p.y=7.8;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=19;
			p.y=-1.5;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			//descer ao pe do carro da direita
			p.x=13.5;
			p.y=-1.5;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			//zona entre os carros
			p.x=13.5;
			p.y=-5.3;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			//zona entre os carros
			p.x=8;
			p.y=-3;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			//contornar buraco			
			p.x=8.55;
			p.y=-1.4;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=8.55;
			p.y=-0.25;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=7.4;
			p.y=-0.25;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=7.4;
			p.y=-1.4;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=8.5;
			p.y=-1.4;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=8.05;
			p.y=-2.7;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=4.4;
			p.y=-2.3;
			p.z=1.2;
			polygon_points.push_back(p);	
			xy_polygon.push_back(Point2f(p.x,p.y));

			
			//obstaculo
			p.x=7.8;
			p.y=-1.35;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=8.5;
			p.y=-1.35;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=8.5;
			p.y=-0.35;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=7.5;
			p.y=-0.35;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=7.5;
			p.y=-1.35;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=7.8;
			p.y=-1.35;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
		
// 	//________________________
// 	//                        |
// 	//  9 cloud ground truth  |
// 	//________________________|
/*	

			p.x=4.4;
			p.y=-2.1;
			p.z=1.2;
			polygon_points.push_back(p);	
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=4.4;
			p.y=2;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=6;
			p.y=6.2;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=7.2;
			p.y=4.5;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
		
			p.x=8;
			p.y=5.7;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=9.5;
			p.y=4;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));

			
			p.x=9;
			p.y=3;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=18;
			p.y=-0.8;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
// 			ir para o lado direito
			p.x=15.5;
			p.y=-5.75;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
// 			descer ponte
			p.x=6.8;
			p.y=-2.4;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=6.3;
			p.y=-3.3;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=4.8;
			p.y=-2.8;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=4.4;
			p.y=-2.1;
			p.z=1.2;
			polygon_points.push_back(p);	
			xy_polygon.push_back(Point2f(p.x,p.y));
			
// 			obstáculo
			p.x=15.5;
			p.y=-5.8;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
// 			descer ponte
			p.x=7;
			p.y=-2.5;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=6.8;
			p.y=-3.05;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=15.2;
			p.y=-6.45;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=15.5;
			p.y=-5.8;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
	*/


	
	//________________________
	//                        |
	// 11 cloud ground truth  |
	//________________________|
/*
			p.x=4.6;
			p.y=-1.5;
			p.z=1.2;
			polygon_points.push_back(p);	
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=4.6;
			p.y=1.25;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=12;
			p.y=3.9;
			p.z=1.2;
			polygon_points.push_back(p);
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			//ponto ao pe da descida
			p.x=14.5;
			p.y=2.2;
			p.z=1.2;
			polygon_points.push_back(p);	
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			//descer um pouco
			p.x=9.3;
			p.y=0.2;
			p.z=1.2;
			polygon_points.push_back(p);
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			//contornar devisoria
			p.x=9.8;
			p.y=-1;
			p.z=1.2;
			polygon_points.push_back(p);	
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=18;
			p.y=-1;
			p.z=1.2;
			polygon_points.push_back(p);
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=18;
			p.y=-4;
			p.z=1.2;
			polygon_points.push_back(p);	
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=16;
			p.y=-8;
			p.z=1.2;
			polygon_points.push_back(p);
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=13.5;
			p.y=-8;
			p.z=1.2;
			polygon_points.push_back(p);
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=11.5;
			p.y=-4;
			p.z=1.2;
			polygon_points.push_back(p);
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=10;
			p.y=-4.5;
			p.z=1.2;
			polygon_points.push_back(p);
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=6.8;
			p.y=-6.5;
			p.z=1.2;
			polygon_points.push_back(p);
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=4.6;
			p.y=-1.5;
			p.z=1.2;
			polygon_points.push_back(p);	
			xy_polygon.push_back(Point2f(p.x,p.y));
	
			//obstaculo
			p.x=11.6;
			p.y=-4.3;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=10.3;
			p.y=-4.7;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=11.1;
			p.y=-6.5;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=12.5;
			p.y=-6.2;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=11.6;
			p.y=-4.3;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
	
		*/	
		
	//________________________
	//                        |
	// 14 cloud ground truth  |
	//________________________|
/*
			p.x=4.4;
			p.y=-1;
			p.z=1.2;
			polygon_points.push_back(p);	
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=4.4;
			p.y=4.8;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=9;
			p.y=5.3;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			//iniciar subida
			p.x=11;
			p.y=-2.3;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			//contornar poste
			p.x=8.05;
			p.y=-2.5;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
				
			p.x=8.05;
			p.y=-2.95;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=8.45;
			p.y=-2.95;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=8.45;
			p.y=-2.55;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=11;
			p.y=-2.35;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			//voltar a subir
			p.x=11.5;
			p.y=-4.8;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=10.8;
			p.y=-4.9;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=6.3;
			p.y=-4.8;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=4.5;
			p.y=-2.8;
			p.z=1.2;
			polygon_points.push_back(p);		
			xy_polygon.push_back(Point2f(p.x,p.y));
	
			p.x=4.4;
			p.y=-1;
			p.z=1.2;
			polygon_points.push_back(p);	
			xy_polygon.push_back(Point2f(p.x,p.y));
			
			//obstáculo
			
			p.x=8.05;
			p.y=-2.5;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
				
			p.x=8.05;
			p.y=-2.95;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=8.45;
			p.y=-2.95;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=8.45;
			p.y=-2.5;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));
			
			p.x=8.05;
			p.y=-2.5;
			p.z=1.2;
			obstacle_points.push_back(p);		
			obstacle_polygon.push_back(Point2f(p.x,p.y));*/

}

void Navigability_Map::getCell_inpolygon(void)
{
	grid_nodePtr node_data;
	double x_cell=0;
	double y_cell=0;
	
	for(uint row=0;row<grid.rows();row++)
	{
		for(uint col=0;col<grid.cols();col++)
		{				
			node_data=grid(row,col);
			
			// ________________________________________________________________________________
			//|                          					                                    |
			//| Cell is occupied (with Z and normal) or has interpolated data (Z and normal)  |
			//|______________________________________________________________________________|
			
			if ((node_data->num_points>0 && node_data->has_normal) || (node_data->interpolate_z_data && node_data->interpolate_normal_data) || (node_data->num_points>0 && node_data->interpolate_normal_data)) 
			{
				//X posistion
				x_cell = row*Sx+Sx/2;
				//Y position
				y_cell = col*Sy-CARaxis_col*Sy;
				
				//_________________
				//                |
				//  polygon test  |
				//________________|
				
				// create a pointer to the data as an array of points (via a conversion to 
				// a Mat() object)
				
				Point2d test_pt;
				test_pt.x = x_cell;
				test_pt.y = y_cell;
				
				if (pointPolygonTest(Mat(xy_polygon), test_pt, false) > 0) //false, dont calc dist ;
				{
					//Cell is in polygon, save row and col
					Matrix_Cell_inpolygon_rowcol.conservativeResize(Matrix_Cell_inpolygon_rowcol.rows()+1,2);
					
					Matrix_Cell_inpolygon_rowcol(Matrix_Cell_inpolygon_rowcol.rows()-1,0)=row;
					Matrix_Cell_inpolygon_rowcol(Matrix_Cell_inpolygon_rowcol.rows()-1,1)=col;				
				}

				if (pointPolygonTest(Mat(obstacle_polygon), test_pt, false) > 0) //false, dont calc dist ;
				{
					//Cell is in polygon, save row and col
					Matrix_obstacle_inpolygon_rowcol.conservativeResize(Matrix_obstacle_inpolygon_rowcol.rows()+1,2);
					
					Matrix_obstacle_inpolygon_rowcol(Matrix_obstacle_inpolygon_rowcol.rows()-1,0)=row;
					Matrix_obstacle_inpolygon_rowcol(Matrix_obstacle_inpolygon_rowcol.rows()-1,1)=col;				
				}
		
			}
			
		}
		
	}
	

}

void Navigability_Map::dataCell_inpolygon(void)
{
	
	grid_nodePtr node_data;
	int row_poly=0;
	int col_poly=0;
	int row_obs=0;
	int col_obs=0;
	
	int total_cells_poly=Matrix_Cell_inpolygon_rowcol.rows();
	int cell_goodacc_poly=0;
	
	int total_cells_obs=Matrix_obstacle_inpolygon_rowcol.rows();
	int cell_badacc_obs=0;
	
	//acess data in the cell
	for (int i=0; i<total_cells_poly; i++)
	{
		row_poly=Matrix_Cell_inpolygon_rowcol(i,0);
		col_poly=Matrix_Cell_inpolygon_rowcol(i,1);
		
		node_data=grid(row_poly,col_poly);
		
		if (node_data->total_accessibility>0.25)
			cell_goodacc_poly++;
	}
	
	//acess data in the cell
	for (int i=0; i<total_cells_obs; i++)
	{
		row_obs=Matrix_obstacle_inpolygon_rowcol(i,0);
		col_obs=Matrix_obstacle_inpolygon_rowcol(i,1);
		
		node_data=grid(row_obs,col_obs);
		
		if (node_data->total_accessibility<=0.25)
			cell_badacc_obs++;
	}
	
	double algorithm_efficiency=0;
	algorithm_efficiency=((double)cell_goodacc_poly/(double)total_cells_poly)*100;
	
	std::cout<<"total cells in polygon: "<<total_cells_poly<<" good cells acc>0.25: "<<cell_goodacc_poly<<std::endl;
	std::cout<<" algorithm_efficiency_polygon: "<<algorithm_efficiency<<"%"<<std::endl;
	std::cout<<std::endl;
	
	//obstacle cells
	double algorithm_efficiency_obstacle=0;
	algorithm_efficiency_obstacle=((double)cell_badacc_obs/(double)total_cells_obs)*100;
	
	std::cout<<"total cells in obstalce: "<<total_cells_obs<<" bad cells acc<=0.25: "<<cell_badacc_obs<<std::endl;
	std::cout<<" algorithm_efficiency_obstalce: "<<algorithm_efficiency_obstacle<<"%"<<std::endl;
	std::cout<<std::endl;
}
