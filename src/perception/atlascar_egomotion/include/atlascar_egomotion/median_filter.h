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
 * \file
 * \brief Median filter code implementation
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <stdlib.h>
#include <signal.h>
#include "tcp_client/class_tcp.h"
#include <optoelectric/sensor_sharp_msg.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#define PFLN {printf("%s %d\n",__FILE__, __LINE__);}
//include eigen library to vector use
#include <vector>
using namespace std;
using namespace ros;

//MedFilt Function
/** 
 * @brief GetMedian
 * @details Calculates the median value of an array, given an neighborhood size n
*/

float GetMedian(float a[],int n)
{

	float temp;
	int i,j;
	for(i=0;i<n;i++)

	for(j=i+1;j<n;j++)
	{
		if(a[i]>a[j])
		{
	
			temp=a[j];
			a[j]=a[i];
			a[i]=temp;
		}

	}	

	if(n%2==0)
		return (a[n/2]+a[n/2-1])/2;
	else
		return a[n/2];

}
/** 
 * @brief Median_Filter
 * @details Calculates Median distance, subscribes a distance.
*/
float Median_Filter(int countt,float dist,float array[])
{
	float med_dist;
	if(countt<8)
	{
		array[countt]=dist;
		med_dist=GetMedian(array,countt);
	}	
	else 
	{	
		array[7]=dist;
		for (int i=0;i<7;i++)
		{
			array[i]=array[i+1];
		}
		med_dist=GetMedian(array,8);
	}
return med_dist;
}
