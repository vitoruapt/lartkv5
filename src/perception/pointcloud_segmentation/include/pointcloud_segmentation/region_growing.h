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

#ifndef _PCL_REGION_GROWING_H_
#define _PCL_REGION_GROWING_H_

/**
 * \file
 * \brief Class for pcl region growing algorithm
 * \author Tiago Talhada
 * \date April 2012
 */

#include <iostream>
#include <vector>
#include <algorithm>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree.h>

using namespace pcl;
using namespace std;

/** \brief Header file with class declaration for region growing
 *  \author Tiago Talhada
 *  \date April 2012
 *  \file region_growing.h
 */

/** \struct TYPE_flags
 *  \brief some flags to check status
 */
typedef struct
{
    bool cloud_in_set;
    bool seeds_in_set;
    bool ok_to_run;
    bool run_finished;
}TYPE_flags;


/** \class region_growing 
 * \brief region growing based algorithm.
 */
template<class T, class T1>
class region_growing
{
public:
    region_growing()
    {
	// reset the variables
	min_neighbors=5;
	
	// reset flags
	flags.cloud_in_set=false;
	flags.seeds_in_set=false;
	flags.run_finished=false;
	flags.ok_to_run=false;
    };
    
    ~region_growing()
    {

    }
    
    ///////////////////////////////
    //// PUBLIC VARIABLES /////////
    ///////////////////////////////
    /** \brief The radius for search */
    float radius;
        
    /** \brief Number of neighbors to consider the point */
    int min_neighbors;
    
    ///////////////////////////////
    //// PUBLIC FUNCTIONS /////////
    ///////////////////////////////
    /** \brief Sets the input point cloud data */
    void set_input_cloud(typename PointCloud<T>::Ptr cloud_in);
    
    /** \brief Sets the input point cloud seeds */
    void set_input_seeds(typename PointCloud<T1>::Ptr seeds_in);
        
    /** \brief simply runs the algorithm  */
    void filter ();
        
    /** \brief Provide acces to the result indices */
    vector<int> get_region_indices(void);
    
    /** \brief Provide acces to the result indices */
    PointIndices get_region_point_indices(void);
    
private:
    ///////////////////////////////
    //// PRIVATE VARIABLES ////////
    ///////////////////////////////
    /** \brief Flags set */
    TYPE_flags flags;
    
    /** \brief indices from a region */
    std::vector<int> region_indices;
    
    /** \brief Point indices from a region */
    PointIndices region_point_indices;
    
    /** \brief search indices list */
    std::vector<int> search_list;
    
    /** \brief Input cloud data for searching */
    typename PointCloud<T>::Ptr cloud;
    
    /** \brief Input seeds for starting points */
    typename PointCloud<T1>::Ptr seeds;
    
    /** \brief Output region cloud data */
    PointCloud<T> region;
      
//     KdTreeFLANN<T> tree;
    
    ///////////////////////////////
    //// PRIVATE FUNCTIONS ////////
    ///////////////////////////////
    
    /** \brief Process flags to check if it is ok to run */
    void process_flags();
    
    /** \brief extract region_indices from cloud */
    void extract_region_indices();   
};

#include "region_growing.hpp"
#endif
