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
 * \brief Auxiliary haptic functions implementation
 */

#include <phua_haptic/hd_hl_apis_aux.h>

/*~~~~~~~~~~~~~~~~~ 
|| AUX FUNCTIONS  ||
~~~~~~~~~~~~~~~~~~*/
hduVector3Dd TransformWorldCoordinatesToPHaNToMCoordinates(hduVector3Dd source)
{
  static hduVector3Dd destination;
  static Eigen::Vector3d coord_in_world_ref;
  
  coord_in_world_ref = (rotz(-90.) * roty(-90.)) * (Eigen::Vector3d) source ;
  
  destination[0] = coord_in_world_ref[0];
  destination[1] = coord_in_world_ref[1];
  destination[2] = coord_in_world_ref[2];
  
  return destination;
}

hduVector3Dd TransformPHANToMCoordinatesToWorldCoordinates(hduVector3Dd source)
{
  static hduVector3Dd destination;
  static Eigen::Vector3d coord_in_phantom_ref;
  
  coord_in_phantom_ref = (rotz(90.) * rotx(90.)) * (Eigen::Vector3d) source ;
  
  destination[0] = coord_in_phantom_ref[0];
  destination[1] = coord_in_phantom_ref[1];
  destination[2] = coord_in_phantom_ref[2];
  
  return destination;
}

hduMatrix TransformPHANToMCoordinateMatrixToWorldCoordinateMatrix(hduMatrix source)
{
//   static hduMatrix destination;
  
  static const hduMatrix rotation_x_90(1,0,0,0,
				 0,0,-1,0,
				 0,1,0,0,
				 0,0,0,1);
				 
  static const hduMatrix rotation_z_90(0,-1,0,0,
				 1,0,0,0,
				 0,0,1,0,
				 0,0,0,1);

  return (rotation_z_90 * rotation_x_90) * source;
}
