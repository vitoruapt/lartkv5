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

#ifndef _PCL_REGION_GROWING_HPP_
#define _PCL_REGION_GROWING_HPP_

/**
 * \brief File of function implementation of c++ class
 * \author Tiago Talhada
 * \file
 * \date April 2012
 */

#define PFLN  printf("LINE=%d in FUNCTION=%s\n",__LINE__,__FUNCTION__);

#include <pointcloud_segmentation/region_growing.h>

template<class T, class T1>
void region_growing<T,T1>::set_input_cloud(typename PointCloud<T>::Ptr cloud_in)
{
    cloud=cloud_in;  // sets the private pointer
    flags.cloud_in_set=true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T, class T1>
void region_growing<T,T1>::set_input_seeds(typename PointCloud<T1>::Ptr seeds_in)
{
    seeds=seeds_in; // sets the private pointer
    flags.seeds_in_set=true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T, class T1>
void region_growing<T,T1>::process_flags()
{
   
    if (flags.cloud_in_set && flags.seeds_in_set)
	flags.ok_to_run=true;
    
    // cout to the screen errors on flags
	if (radius<=0)
	    cout << "ERROR: MUST SET PROPERLY A POSITIVE RADIUS THRESHOLD" << endl;
	if (!flags.cloud_in_set)
	    cout << "ERROR: MUST SET PROPERLY A INPUT CLOUD DATA" << endl;
	if (!flags.seeds_in_set)
	    cout << "ERROR: MUST SET PROPERLY A INPUT SEED CLOUD DATA" << endl;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T, class T1>
vector<int> region_growing<T,T1>::get_region_indices(void)
{
    if (flags.run_finished)
	return region_indices;
    else
    {
	vector<int> empty;
	return empty;	
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

template<class T, class T1>
PointIndices region_growing<T,T1>::get_region_point_indices(void)
{
    if (flags.run_finished)
    {
	region_point_indices.indices=region_indices;
	return region_point_indices;
    }
    else
    {
	PointIndices empty;
	empty.indices.push_back(0);
	return empty;
    }
    
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

template<class T, class T1>
void region_growing<T,T1>::filter ()
{
    /// Step 0 check if we are ok to run the algorithm
    process_flags();
    
    if (flags.ok_to_run)
    {
// 	static bool initialize=true;
	
	/// STEP 1 Create an empty queue indice list
	vector<int> queue_list;
	
	/// STEP 2 Create a kdtree representation of cloud input
// 	octree::OctreePointCloud<T> tree (100);
// 	if (initialize)
// 	{
	    static KdTreeFLANN<T> tree;
	    tree.setInputCloud (cloud);
// 	    tree.addPointsFromInputCloud ();
// 	    initialize=false;
// 	}
	
	/// STEP i Create a bool vector with the indice points processed
	vector<bool> processed ((int)cloud->points.size(), false);
	vector<bool> processed_init ((int)cloud->points.size(), false);
	
	/// STEP 3 Let's make a fisrt search on all seed points
	T searchPoint;
	
	for (size_t i=0; i<seeds->points.size(); ++i)
	{
	    std::vector<float> pointRadiusSquaredDistance;
	    std::vector<int> pointIdxRadiusSearch;
	    searchPoint.x=seeds->points[i].x;
	    searchPoint.y=seeds->points[i].y;
	    searchPoint.z=seeds->points[i].z;
	    int neighbor_number=tree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	    if (neighbor_number>0)
	    {
		for (size_t k=0; k<pointIdxRadiusSearch.size(); ++k)
		{
		    if (!processed_init[pointIdxRadiusSearch[k]])
		    {
			region_indices.push_back(pointIdxRadiusSearch[k]);
			processed_init[pointIdxRadiusSearch[k]]=true;
		    }
		}
	    }
	    pointIdxRadiusSearch.clear();
	    pointRadiusSquaredDistance.clear();
	}
	
	/// STEP 4 The queue list will be all the points on region_indices in the begining
	queue_list=region_indices;
	/// STEP 4.1 Make a search on each point and check what points are already processed and if not add it to queue list
	for (size_t i=0; i<queue_list.size(); ++i)
	{
	    std::vector<float> pointRadiusSquaredDistance;
	    std::vector<int> pointIdxRadiusSearch;
	    searchPoint=cloud->points[queue_list[i]];
	    int neighbor_number=tree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	    if (neighbor_number>min_neighbors)
	    {
		/// 4.1.2 For each neighbor found do the following
		for (size_t k=0; k<pointIdxRadiusSearch.size(); ++k)
		{
		    /// 4.1.3 Check if the point is already processed and if not added to queue list
		    if (!processed[pointIdxRadiusSearch[k]])
			{
			    queue_list.push_back(pointIdxRadiusSearch[k]); 	 // Add on queue list 
			    region_indices.push_back(pointIdxRadiusSearch[k]);   // Add on region indices
			    processed[pointIdxRadiusSearch[k]]=true;             // Mark the point as processed
			}
		}
	    }
	    pointIdxRadiusSearch.clear();
	    pointRadiusSquaredDistance.clear();
	}
	/// Setup flags and reset pointers
	
	if(region_indices.size()>0)
	    flags.run_finished=true;
	cloud.reset();
	seeds.reset();
    }
    else
	cout << "COULD NOT PROCCED REGION GROWING. ERRORS OCCURRED!" <<endl;
}

#endif
