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
 * @addtogroup mit_logs
 * @file 
 * @brief header file to camera transformations
 *@{
 */
#ifndef __dgc_camtrans_h__
#define __dgc_camtrans_h__

//#ifdef __cplusplus
//extern "C" {
//#endif

//
// Camera representation
//

typedef struct _CamTrans CamTrans;

CamTrans * camtrans_new_spherical (double width, double height, 
        double fx, double fy, 
        double cx, double cy, double skew,
        const double position[3], const double orientation_quat[4],
        const double distortion_cx, const double distortion_cy,
        double distortion_param);


// Destructor
void camtrans_destroy (CamTrans* t);

// Computes internal matrix representations from parameters
void camtrans_compute_matrices (CamTrans *t);


// Gets camera position in world frame (calibration frame)
void camtrans_get_position (const CamTrans* t,
                                    double pos[3]);
void camtrans_get_orientation (const CamTrans *t,
                                       double orientation[4]);

void camtrans_get_world_to_cam_matrix (const CamTrans *t, double matrix[12]);
void camtrans_get_cam_to_world_matrix (const CamTrans *t, double matrix[9]);


double camtrans_get_focal_length_x (const CamTrans *t);
double camtrans_get_focal_length_y (const CamTrans *t);
double camtrans_get_image_width (const CamTrans *t);
double camtrans_get_image_height (const CamTrans *t);
double camtrans_get_principal_x (const CamTrans *t);
double camtrans_get_principal_y (const CamTrans *t);
double camtrans_get_width (const CamTrans *t);
double camtrans_get_height (const CamTrans *t);
void camtrans_get_distortion_center (const CamTrans *t, double *x, double *y);

/**
 * isotropically scale image
 *
 */
void camtrans_scale_image (CamTrans *t, const double scale_factor);


/**
 * rotate camera by q. the rotation is applied to the camera body.
 *
 */
void camtrans_rotate_camera (CamTrans *t, const double q[4]);

/**
 * change camera distortion center
 *
 */
void camtrans_set_distortion_center (CamTrans *t, const double cx,
                                     const double cy);


//
// Camera functions
//

/**
 * camtrans_undistort_pixel_exact:
 *
 * Produce rectified pixel from distorted pixel.
 */
int camtrans_undistort_pixel (const CamTrans* cam,
                                      const double x, const double y,
                                      double* ox, double* oy);

/**
 * camtrans_distort_pixel:
 *
 * Produce distorted pixel from rectified pixel. 
 */
int camtrans_distort_pixel (const CamTrans* cam,
                                    const double x, const double y,
                                    double* ox, double* oy);

// Produce 3D ray from pixel measurement, to body frame coordinates
int camtrans_pixel_to_ray (const CamTrans* cam,
                                   const double x, const double y,
                                   double ray[3]);

/**
 * Project point in the vehicle body frame to image pixel, with optional
 * distortion
 *
 * Returns: 0 on success, -1 if the point lies on the camera's principal plane
 */
int camtrans_project_point (const CamTrans* cam,
                                    const double * p_world,
                                    const int distort,
                                    double* ox, double* oy, double *oz);

/**
 * Project line in the vehicle body frame to undistorted image pixel coords
 *
 * Returns: 0 on success, -1 if the resulting line is at infinity
 */
int camtrans_project_line (const CamTrans *cam,
                           const double *l_world,
                           double *ox, double *oy, double *oz);


//#ifdef __cplusplus
//}
//#endif

#endif
/**
 *@}
 */      
