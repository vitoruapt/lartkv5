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
/** \brief Nodelet that recieves ros msg's from a bag file and then prints to a text file
 * 
 *  \file
 *  \author André Oliveira
 *  \date May 2012
 */

#include <driver_recognition/class_recognition.h>



class_recognition::TYPE_msg_partial partial_msg;

class_recognition::TYPE_msg_bags recognition;
FILE *fp;

/**
 * \brief Create a txt file with the information needed  to use in matlab
 * 
 * \return null
 */
int cria_txt(void)
{

   string nane_file;

    nane_file.append("../");
    nane_file.append(recognition.name_file);  
    nane_file.erase(nane_file.end()-4, nane_file.end());
    nane_file.append(".txt");

    fp = fopen(nane_file.c_str(),"a+"); //tenta ler o ficheiro já existente
    /* Verificar se a abertura foi feita com sucesso */

	if (fp==NULL) //Não consegue abir o ficheiro
	{
		cout<<"cannot open file: recognition_data.txt" <<endl;
	    	    return 1;
	}
	
	fprintf(fp,"%s \t %f \t %d \t %d \t %d \t %d \t %d \t %d \t %f \t %f \t %f\n",
	      recognition.name_file.c_str(),
		recognition.bag_time,
	      partial_msg.ignition,
	      partial_msg.lights_left,
	      partial_msg.lights_right,
	      partial_msg.throttle,
	      partial_msg.brake,
	      partial_msg.clutch,
	      partial_msg.velocity,
	      partial_msg.steering_wheel,
	      partial_msg.rpm);

		
     
     fclose(fp);

  return 0;
};




/**
 * \brief Callback to save the msg from driver monitoring Arduino
 * 
 * \return void
 */
// void topic_callback_partial(atlascar_base::AtlascarPartialStatus AtlascarPartialStatus_msg)
// 		{
// 				partial_msg.lights_high=AtlascarPartialStatus_msg.lights_high; 
// 				partial_msg.lights_medium=AtlascarPartialStatus_msg.lights_medium; 
// 				partial_msg.ignition=AtlascarPartialStatus_msg.ignition; 
// 				partial_msg.lights_left=AtlascarPartialStatus_msg.lights_left; 
// 				partial_msg.lights_right=AtlascarPartialStatus_msg.lights_right; 
// 				partial_msg.danger_lights=AtlascarPartialStatus_msg.danger_lights; 
// 				partial_msg.horn=AtlascarPartialStatus_msg.horn; 
// 				partial_msg.throttle=AtlascarPartialStatus_msg.throttle; 
// 				partial_msg.brake=AtlascarPartialStatus_msg.brake; 
// 				partial_msg.clutch=AtlascarPartialStatus_msg.clutch;
// 				recognition.bag_time=ros::Time::now().toSec();
// 		};
/**
 * \brief Callback to save the msg from Velocity Arduino
 * 
 * \return void
 */

// void topic_callback_velocity(atlascar_base::AtlascarVelocityStatus AtlascarVelocityStatus_msg)
// 		{
// 				partial_msg.velocity=AtlascarVelocityStatus_msg.velocity;
// 				//uma vez que é a mensagem que chag mais tarde a ser actualizada achei por bem recolher os dqados na mesma frequencia, isto é 9hz, paso queira confirmar faça rostopic hz /vhc/driver/status
// 				
// 				cria_txt();
// 		};
/**
 * \brief Callback to save the msg from atlascar PLC
 * 
 * \return void
 */	
// void topic_callback_plc_status(atlascar_base::AtlascarStatus AtlascarStatus_msg)
// 		{
// 				partial_msg.steering_wheel=AtlascarStatus_msg.steering_wheel;
// 				partial_msg.rpm=AtlascarStatus_msg.rpm;
// 
// 				
// 		};
		
		
/**
 * \brief Callback to save the msg from the name of bag file
 * 
 * \return void
 */
void topic_callback_rosout(rosgraph_msgs::Log rosout_msg)
		{
			
			if (rosout_msg.msg[0]=='O' && rosout_msg.msg[1]=='p'&& rosout_msg.msg[2]=='e'&& rosout_msg.msg[6]=='g')
			{
				char name_file[1024];
 				
				string names;
				
				sscanf(rosout_msg.msg.c_str(),"%*s %s",name_file); 
				
 				
				recognition.name_file=name_file;
				

			}
		};

/**
 * \brief Function that organizes the variables in the txt file.
 * 
 * \return void
 */
void   create_head_of_file(void)
{
  
  fp = fopen("../recognition_data.txt","a+"); //tenta ler o ficheiro já existente
				 
    fprintf(fp,"%s \t %s \t %s 	\t %s \t %s \t %s 	\t %s \t %s \t %s 	\t %s \t %s\n",
	      "name of file" ,
	      "bag time",
	      "ignition",	     
	      "lights_left",
	      "lights_right",
	      "throttle",	     
	      "brake",
	      "clutch",
	      "velocity",	     
	      "steering_wheel",
	      "rpm");
  fclose(fp);
}


/**
 * \brief main of program
 * 
 * \return void
 */
int main (int argc, char **argv)
{
  ROS_INFO("Starting recognition_data node.");
  
  create_head_of_file();
  
  ros::init(argc,argv,"recognition"); 
  ros::NodeHandle n;

  ROS_ERROR("AtlascarStatus and AtlascarVelocityStatus no longer avaliable, please correct!!");
//   ros::Subscriber sub = n.subscribe("/mtt/targets", 1, topic_callback);
//   ros::Subscriber sub_status_plc = n.subscribe("/vhc/plc/status", 1, topic_callback_plc_status);
//   ros::Subscriber sub_partial_status = n.subscribe("/vhc/driver/status", 1, topic_callback_partial);
//   ros::Subscriber sub_velocity_status = n.subscribe("/vhc/velocity/status", 1, topic_callback_velocity);
  ros::Subscriber sub_rosout =n.subscribe("/rosout_agg", 1, topic_callback_rosout);
//   ros::Subscriber sub_status_plc = n.subscribe("/atc/base/status/plc", 1, topic_callback_plc_status);
//   ros::Subscriber sub_partial_status = n.subscribe("/partial_status", 1, topic_callback_partial);
//   ros::Subscriber sub_velocity_status = n.subscribe("/velocity_status", 1, topic_callback_velocity);
 

  ros::spin();
  
  return 1;
}
