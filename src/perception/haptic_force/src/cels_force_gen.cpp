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
 * @file cels_force_gen.cpp
 * @author Emílio Estrelinha nº 38637 (emilio.estrelinha@ua.pt)
 * @brief Generates forces from pressure_cells information/data
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <numeric>
#include <assert.h>
#include <sstream>

#include <haptic_force/Force.h>

#include <pressure_cells/Cop.h>


haptic_force::Force force;


void COPCallBk ( const pressure_cells::Cop &COP );

/**
* @fn int main ( int argc, char **argv )
* Main 
*/
int main ( int argc, char **argv )
{
    // Generate a simple force to send a message to phantom_control
    ros::init( argc, argv, "cell_force" );
    
    ros::NodeHandle n("~");

    ros::Publisher pub_force = n.advertise< haptic_force::Force >( "/force", 1000 );

    ros::Subscriber sub_cop = n.subscribe ( "/cop_left_right", 1000, COPCallBk );

    ros::Rate loop_rate( 1.5*1300 );

    force.force[1]=0.49;

    int pid = getpid(), rpid;

    boost::format fmt("sudo renice -10 %d");
    fmt % pid;
    
    rpid = std::system(fmt.str().c_str());
    
    while(ros::ok())
    {
        //force.force[0]=force2send[0];
        //force.force[1]=force2send[1];
        //force.force[2]=force2send[2];
        force.header.stamp = ros::Time::now (  );
        pub_force.publish( force );

        //force2send[0]+=0.01;
        //force2send[2]+=0.01;

        //if(force2send[0]>1.5) force2send[0]=-1.5;
        //if(force2send[2]>1.5) force2send[2]=-1.5;

        ros::spinOnce (  );
        
        loop_rate.sleep (  );
    }


    return 0;
}


void COPCallBk ( const pressure_cells::Cop &COP )
{
    double XX, YY;
    
    XX=COP.copx;
    YY=COP.copy;


    
    
    
    // É necessário acertar os coeficientes da recta depois de calbrar as celulas de carga
    
    // Mostra o desequilibrio => direçao de força igual a posiçao do COP
    force.force[0]=( (2.5 - (-2.5)) / (0.14 - 0.0) )*YY - 2.5;      // limite da força / limite do COP (esperado)
    force.force[2]=( (2.5 - (-2.5)) / (0.046 - (-0.046)) )*XX;       // limite da força / limite do COP (esperado)

    if (abs(force.force[2]) < 0.75)
    {
        force.force[2]=0;
    }
    else if((force.force[2]) <= - 0.75)
    {
        force.force[2]+=  0.75;
    }
    else if((force.force[2]) >= 0.75)
    {
        force.force[2]+=  - 0.75;
    }

    if (abs(force.force[0]) < 0.75)
    {
        force.force[0]=0;
    }
    else if((force.force[0]) <= - 0.75)
    {
        force.force[0]+= 0.75;
    }
    else if((force.force[0]) >= 0.75)
    {
        force.force[0]+=  - 0.75;
    }
    
    //std::cout<<"Força em XX : "<<force.force[0]<<"\tForça em ZZ : "<<force.force[2]<<std::endl;

    //force.force[0]=0.0;
    //force.force[2]=0.0;
    
    //force.force[1]=2.0;
    
}
