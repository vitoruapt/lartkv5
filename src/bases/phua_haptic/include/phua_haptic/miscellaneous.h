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
/** @file
 * @brief miscellaneous.h file for this module. Contains prototypes, includes and defines.
 
 This modules contains several functions used overall.
 * @author pedro_cruz
 * @version 2.0
 * @date 3 May 2012
 *@{
 */
#ifndef __MISCELLANEOUS_H_
#define __MISCELLANEOUS_H_

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <armadillo>

#include <phua_haptic/types.h>

//PFLN DEFINITION (CREDITS TO MIKE :-))
#define PFLN {printf("Passing through File [%s] @ Line [%d].\n", __FILE__, __LINE__);}

/** 
* @brief Retrieve a value cropped with the required precision.

* @param[in] original the original value to be cropped.
* @param[in] precision the precision required.
* @return the cropped value.
*/
template <typename Type>
Type RetrievePrecisionFromDouble(Type original, Type precision)
{
  int integer=(int)original;
  Type sub1=(1.0/precision)*(original-(Type)integer);
  return (Type)integer+precision*((Type)round(sub1));
}

/** 
* @brief Templated degree to radian conversion function.

* @param[in] deg angle in degrees.
* @return angle in radians.
*/
template <typename Type>
Type DegToRad(Type deg)
{
#if ROS_VERSION_MINIMUM(1, 8, 0)   //At least FUERTE	, V. Santos, 27-Mai-2013
  return (deg * (Type)M_PI / (Type)180.0);
#else
  return (deg * (Type)arma::datum::pi / (Type)180.0);
#endif
}

/** 
* @brief Templated radian to degree conversion function.

* @param[in] rad angle in radians.
* @return angle in degree.
*/
template <typename Type>
Type RadToDeg(Type rad)
{
#if ROS_VERSION_MINIMUM(1, 8, 0)   //At least FUERTE	, V. Santos, 27-Mai-2013
  return (rad * (Type)180.0 / (Type)M_PI);
#else
  return (rad * (Type)180.0 / (Type)arma::datum::pi);
#endif
}

/** 
* @brief Function acumulate position differences and calculate average speed.

This function stores position diffencial values and calculates mean value over time.
* @param[in] diffX xx axis position differences.
* @param[in] diffY yy axis position differences.
* @param[in] diffZ zz axis position differences.
* @param[in] time_interval time intervals of each given space differences.
* @param[out] return_vector destination return array.
* @return none.
*/
void CalculateAverageCartesianSpeed(double diffX, double diffY, double diffZ, long double time_interval, double *return_vector);

/** 
* @brief Function to calculate mean value of an array [Taken from http://rosettacode.org/].

* @param[in] v pointer to the array to calculate.
* @param[in] len lenght of array elements to use in the calculation.
* @return mean value calculated.
*/
double pc_mean(double *v, int len);

/** 
* @brief Function to calculate rotation matrix over the X axis.
* @param[in] angle_in_degrees angle for rotation.
* @return rotation matrix.
*/
Eigen::Matrix3d rotx(double angle_in_degrees);

/** 
* @brief Function to calculate rotation matrix over the Y axis.
* @param[in] angle_in_degrees angle for rotation.
* @return rotation matrix.
*/
Eigen::Matrix3d roty(double angle_in_degrees);

/** 
* @brief Function to calculate rotation matrix over the Z axis.
* @param[in] angle_in_degrees angle for rotation.
* @return rotation matrix.
*/
Eigen::Matrix3d rotz(double angle_in_degrees);

/** 
* @brief Template function to calculate mean value of an array.

This function calculates the mean value of an a given vector, that can be of any type.
* @param[in] vect vector to calculated mean.
* @return mean value calculated.
*/
template <class T>
T pc_mean(std::vector<T> vect)
{
  T sum = 0;
  
  for(uint i = 0; i < vect.size(); i++)
    sum += vect[i];
  
  return sum / vect.size();
}

/** 
* @brief Template function to determine if a given value is within a give range.
* @param[in] value the value to evaluate.
* @param[in] lower_bound the lower bound of the range.
* @param[in] upper_bound the upper bound of the range.
* @return TRUE if value is within range,FALSE if not.
*/
template <typename Type>
bool IsWithinRange(Type value, Type lower_bound, Type upper_bound)
{
  if(value >= lower_bound && value < upper_bound)
  {
    return TRUE;
  }
  else if(value > lower_bound && value <= upper_bound)
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

#ifdef _cplusplus
extern "C" {
#endif // _cplusplus

/** 
* @brief Function to check if string is numeric.

Returns true (non-zero) if character-string parameter represents a signed or unsigned floating-point number. Otherwise returns false (zero).
[Taken from http://rosettacode.org/] | my edit: strtod() warning supression 
* @param[in] s a const char array to evaluate.
* @return true (non-zero) / otherwise returns false (zero).
*/
int isNumeric (const char * s);

/** 
* @brief Function to determine sign of a given number.

This function determines the sign of a number and return -1 or 1, for negative or positive respectively.
* @param[in] x number to evaluate.
* @return -1 for negative, 1 for positive.
*/
double get_sign(double x);

//FUNCTIONS I CREATED

//phua_haptic related
/**
* @brief Prints to stdout the program help tips.

This function prints to stdout a set of program help tips.
*/
void print_help(void);

/**
* @brief Prints to stdout the program greeting.

This function prints to stdout a greeting message.
* @param[in] RobotVars structure defined in types.h with the joint values.
*/
void print_greeting(shared_vars_t*RobotVars);

//other
/**
* @brief Gets a number from keyboard input.

This function retrieves a number from a keyboard input.
* @return returns an integer from input.
*/
int GetNbrFromKeyboard(void);

/** 
* @brief Gets a character from keyboard input.

This function retrieves a character from a keyboard input.
* @return returns a character from input.
*/
char GetStrFromKeyboard(void);

/**
* @brief Gets a double number from keyboard input.

This function retrieves a double number from a keyboard input.
* @return returns a double number from input.
*/
double GetDblFromKeyboard(void);

/**
 * @brief Sends equivalent shell command "clean" to stdout.

 * @return none.
 */
void ScreenClear(void);

//COLOR FUNCTIONS ATTRIBUTES
/**
* @ Attributes
*/
#define RESET		0
#define BRIGHT 		1
#define DIM		2
#define UNDERLINE 	3
#define BLINK		4
#define REVERSE		7
#define HIDDEN		8

/**
* @ Colors
*/
#define BLACK 		0
#define RED		1
#define GREEN		2
#define YELLOW		3
#define BLUE		4
#define MAGENTA		5
#define CYAN		6
#define	GRAY		7
#define	WHITE		8


///FUNCTIONS CREATED BY VITOR SANTOS(vitor@ua.pt) | ALL CREDITS GO TO HIM... :-)
/**
 * @brief  Set the color of text that follows
 *
 * @param[in]  attr Type of attribute (RESET, BRIGHT, BLINK, etc...
 * @param[in]  fg Color of foreground
 * @param[in]  bg Color of background
 * @return Nothing
 */
void textcolor(int attr, int fg, int bg);

/**
 * @brief  Printf of an horizontal separator red line 
 *
 * @return Nothing
 */
void PrintRedLine(void);

/**
 * @brief  Reset color text
 *
 * @return Nothing
 */
void ResetTextColors(void);

/**
 * @brief  Make somesort of highlight text
 *
 * @return Nothing
 */
void HighLightText(void);

#ifdef _cplusplus
}
#endif // _cplusplus

#endif
/**
 *@}
*/
