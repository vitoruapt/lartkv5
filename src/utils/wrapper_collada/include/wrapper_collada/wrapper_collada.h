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
 * @addtogroup wrapper_collada 
 *@{
 */
#ifndef _wrapper_collada_H_
#define _wrapper_collada_H_

/**
 * @file wrapper_collada.h
 * @brief The actual wrapper collada class definition
 * @author Miguel Armando Riem de Oliveira
 * @version 0.0
 * @date 2011-09-15
 */

//####################################################################
//// Defines:
////####################################################################


//####################################################################
//// Includes:
////####################################################################
#include <ros/ros.h>  /* Added by V. Santos, 15-Abr-2013,10:12 */
//dom
#include <dae.h>
#include <dom/domConstants.h>
#include <dom/domCOLLADA.h>
#include <dom/domProfile_common.h>
#include <dae/daeSIDResolver.h>
#include <dom/domInstance_controller.h>
#include <dae/domAny.h>
#include <dae/daeErrorHandler.h>
#include <dae/daeUtils.h>
#include <dom/domImage.h>
#include <modules/stdErrPlugin.h>
#include <dom/domEllipsoid.h>
#include <dom/domInput_global.h>
#include <dom/domAsset.h>
//#include <dom/domLimits_sub.h>

//pcl
#include <pcl_ros/point_cloud.h>                                                
#include <pcl/point_types.h>


//####################################################################
//// Class:
////####################################################################

using namespace ColladaDOM150;

class wrapper_collada
{
	public:
		wrapper_collada(std::string fl)
		{
			file_name = fl;
			root = dae.add(file_name);
			daeElement* asset = root->add("asset");
			//daeElement* contributor = asset->add("contributor");
			daeElement* created = asset->add("created");
			daeElement* modified = asset->add("modified");
			const char* date = "2008-04-08T13:07:52-08:00";                                             
			created->setCharData(date);
			modified->setCharData(date);

			//Visual scene
			daeElement* visualSceneLib = root->add("library_visual_scenes");
			visualScene = visualSceneLib->add("visual_scene");
			visualScene->setAttribute("id", "defaultScene");

			// Add a <scene>
			root->add("scene instance_visual_scene")->setAttribute("url", makeUriRef("defaultScene").c_str());
		}

		~wrapper_collada(){};

		void add_polygon_fixed_color(std::string polygon_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr normal, float r, float g, float b, float a); 

		void add_polygon_fixed_color_on_both_sides(std::string polygon_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr normal, float r, float g, float b, float a)
		{
			//call the standard add_polygon_fixed_color
			add_polygon_fixed_color(polygon_name, cloud,  normal, r, g, b, a); 
		
			//now call the same function with the fliped normal
			//First flip the normal
			pcl::PointCloud<pcl::PointXYZ> normal_flip;
			pcl::PointXYZ p;
			p.x = -normal->points[0].x;
			p.y = -normal->points[0].y;
			p.z = -normal->points[0].z;
			normal_flip.points.push_back(p);

			//now reverse the cloud pts orientation
			pcl::PointCloud<pcl::PointXYZ> cloud_flip;

			for (int i=((int)cloud->points.size()-1); i>=0; i--)
			{
				cloud_flip.points.push_back(cloud->points[i]);			
			}

			//Finally, add a new fliped polygon
			add_polygon_fixed_color((polygon_name + "_flip").c_str(), cloud_flip.makeShared(),  normal_flip.makeShared(), r, g, b, a); 
		
		}; 


		void write_file()
		{
			ROS_INFO("Writting collada file %s",file_name.c_str());
			dae.writeAll();
		}


	private:

		DAE dae;
		std::string file_name;
		daeElement* root;
		daeElement* visualScene;

		//private methods
		void my_addInput(daeElement* triangles,
                    const std::string& semantic,
                    const std::string& srcID,
                    int offset);


		std::string makeUriRef(const std::string& id);

		void my_addGeometry(daeElement* root);
		void my_addVisualScene(daeElement* root);
		void my_addEffect(daeElement* root);
		void my_addMaterial(daeElement* root); 
		void my_addImage(daeElement* root);
		void my_addSource(daeElement* mesh,
							 const std::string& srcID,
							 const std::string& paramNames,
							 domFloat values[],
							 int valueCount);
		daeTArray<double> pcl_pointcloud_tp_daearray(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
		template<typename T_wc>
		daeTArray<T_wc> rawArrayToDaeArray(T_wc rawArray[], size_t count);

};

#endif
/**
 *@}
 */
