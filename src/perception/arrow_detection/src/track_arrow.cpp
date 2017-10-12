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

/*! 
 *  @file   trak_arrow.cpp
 *  @brief  using VISP trak, it will track the point from "/find_arrow_position" topic and publish the distance in pixels for the center of the image for servos.
 *  
 *  @author     César Sousa, cesarsousa@ua.pt
 *  @date       21-10-2014
 *  @version    V0.0
 *  @internal
 * 
 *      Revision    ---
 *      Compiler    gcc
 *      Company     DEM - Universidade de Aveiro
 *      Copyright   Copyright (c) 2014, César Sousa
 * 
 *      Info:       
 *  
 *  command to make doxyfile: "make doxyfile" than "make doc"
 * 
 */

#include <arrow_detection/track_arrow.h>

/*!
canculate the message to send for servo through the blob's position
\return the servo command
\sa calculate_servos_position
*/
geometry_msgs::Point calculate_servos_position(vpImagePoint center,int xcenter,int ycenter)
{
    geometry_msgs::Point ponto;
   
    //distance for the center of the image
    int dif_x = (((int) center.get_i()) - xcenter);
    int dif_y = (((int) center.get_j()) - ycenter);

    ponto.x = dif_x;
    ponto.y = dif_y;
    ponto.z = 0;
    
    return ponto;
}

//point receiver callback
void positionHandler(const geometry_msgs::Point& point)
{
    GLOBAL_POINT = point;
    receive_new_point = true;
}

//main
int main(int argc,char**argv) 
{
    
std::cout << "Starting 'track-arrow'" << std::endl;
    vpImage<unsigned char> I; //create image
    
    vpROSGrabber g; 
    //g.setImageTopic("/camera/image_raw"); //subscribe image
    g.setImageTopic("/camera/image_rect_color"); //subscribe image
    g.open(I); 
    
    vpDisplayX d(I); 
    
std::cout << "I.getHeight():" << I.getHeight() << " I.getWidth(): " << I.getWidth() << std::endl;
    
    int  xcenter = I.getHeight()/2; //center of the image
    int  ycenter = I.getWidth()/2;
    GLOBAL_POINT.x = xcenter;   //initialize the point that will always be updated with the latest point
    GLOBAL_POINT.y = ycenter;
    
    //ROS
    ros::NodeHandle nh_;
    ros::spinOnce();//necessary for catking include ros libraries  
    
    ros::Subscriber sub = nh_.subscribe("/find_arrow_position", 1,positionHandler);//,&head);
    //where and what will be published
    ros::Publisher pub_state= nh_.advertise< geometry_msgs::Point  >( "/find_arrow_state", 1000 );
    
    //Tracking
    vpDot2 blob;
    blob.setGraphics(true);
    blob.setGraphicsThickness(2);
    blob.setEllipsoidShapePrecision(0);
   
    std::cout << "Starting loop to track the arrow..." << std::endl;

    bool lost_arrow = true;
    
    while(ros::ok())
    { 
        try
        {
            //refresh image
            g.acquire(I);
            //vpDisplay::display(I);//?!??!?!!?! eliminate this line to optimize!
            if (receive_new_point == true)
            {
                receive_new_point = false; 
                lost_arrow = false;
                blob.initTracking(I,vpImagePoint(GLOBAL_POINT.y, GLOBAL_POINT.x));
            }
            if(lost_arrow == false)
            {
                blob.track(I);
                vpImagePoint center(blob.getCog());//get center from visp
                pub_state.publish( calculate_servos_position(center,xcenter,ycenter) );
            }
            vpDisplay::flush(I);
        }       
        catch(vpException e) 
        {
            lost_arrow = true;
        }
        if (vpDisplay::getClick(I, false))
        {
            std::cout << "Exiting.. " << std::endl;
            break;
        }
    }
    return 0;
}

//modifications::22_09_2014 
//
//std::cout << "Blob characteristics: " << std::endl;
//std::cout << " width : " << blob.getWidth() << std::endl;
//std::cout << " height: " << blob.getHeight() << std::endl;
//std::cout << " area: " << blob.getArea() << std::endl;
//std::cout << " gray level min: " << blob.getGrayLevelMin() << std::endl;
//std::cout << " gray level max: " << blob.getGrayLevelMax() << std::endl;
//std::cout << " grayLevelPrecision: " << blob.getGrayLevelPrecision() << std::endl;
//std::cout << " sizePrecision: " << blob.getSizePrecision() << std::endl;
//std::cout << " ellipsoidShapePrecision: " << blob.getEllipsoidShapePrecision() << std::endl;
