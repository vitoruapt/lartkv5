 /**************************************************************************************************
  * Software License Agreement (BSD License)
  *
  * Copyright (c) 2011-2014, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without modification, are permitted
  * provided that the following conditions are met:
  *
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
 *      @file   keyboard.cpp
 *      @brief  read chars from keyboard publish them! 
 *
 *      @author         César Sousa, cesarsousa@ua.pt
 *      @date 29-7-2014
 *      @version V0.0
 *      @internal
 *
 *              Revision        ---
 *              Compiler        gcc
 *              Company         DEM - Universidade de Aveiro
 *              Copyright       Copyright (c) 2014, César Sousa
 *
 *              Info:
 *
 *      command to make doxyfile: "make doxyfile" than "make doc"
 *
 */

#include <arrow_detection/keyboard.h>

int main(int argc, char** argv)
{
    ros::init (argc, argv, "keyboard");
    ros::NodeHandle nh;
    ros::Publisher pub_state;
    
    pub_state= nh.advertise< std_msgs::String  >( "/keyboard", 1000);
    
    std_msgs::String msg;
    
    while (ros::ok())
    {
        std::cout << "input string:   (Press 'q' to quit)" << std::endl;
        std::cin >> msg.data;
        if (msg.data == "q")
        {
            std::cout << "Quiting... " << std::endl;
            return 0;
        }
        pub_state.publish(msg);
        std::cout << "Imputed string:" << msg.data << std::endl;
    }
    // Spin
    ros::spin();
    return 0;
}




