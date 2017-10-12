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



/** \brief Nodelet to detect and display information about danger situation.
 *  \file colision_time.cpp
 *  \author André Oliveira
 *  \date May 2012
 */
#include <driver_monitoring/class_colision_time.h>



visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b);

visualization_msgs::Marker mark_side_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b);

visualization_msgs::Marker generic_text(float x, float y, float z , float r, float g, float b, float scale, std::string txt, std::string frm_id, std::string ns);


ros::Publisher points_pub;
// front_zone of the car
ros::Publisher marker_fronts_publisher;
ros::Publisher marker_front_text_publisher;
//ros::Publisher marker_generic_text_publisher;

//side zone of the car
ros::Publisher marker_side_publisher;
ros::Publisher marker_side_text_publisher;
ros::Publisher marker_side_text_blink_publisher;
ros::Publisher marker_generic_text_blink_left_publisher;			
// class_colision_time colision;
//ros::Publisher marker_front_grid_publisher;


tf::TransformListener *listener_center_bumper_ptr;
class_colision_time::TYPE_msg_mtt obstacle_zone_front; 
class_colision_time::TYPE_msg_mtt obstacle_left; 
class_colision_time::TYPE_msg_partial partial_msg;
class_colision_time::TYPE_msg_velocity velocity_msg;
class_colision_time::TYPE_msg_plc plc_msg;
class_colision_time::TYPE_msg_bags colision_time;


/**
 * \brief Function responsable to write the text file with data relevant to
each risk situation
 *
 * \param[in] int that have the number situation
 * \return null.
 */
int cria_txt(int x)
{
   FILE *fp;
   string nane_file;
  
   nane_file.append("../");
   nane_file.append(colision_time.name_file);  
   nane_file.erase(nane_file.end()-4, nane_file.end());
   nane_file.append(".txt");  
   cout<< "msg_save:"<< colision_time.name_file<<endl;
   fp = fopen(nane_file.c_str(),"a+"); //tenta ler o ficheiro já existente

    /* Verificar se a abertura foi feita com sucesso */

	if (fp==NULL) //Não consegue abir o ficheiro
	{
		
		cout<<"cannot open file: " << nane_file.c_str() <<endl;
	    	   // return 1;
	}
		if (x==1){
			fprintf(fp,"%s \t %s \t %f \t %f \t %f \n", colision_time.name_file.c_str(), colision_time.sit_type_1.c_str(),colision_time.sit_begin_1, colision_time.duration_1,  colision_time.sit_end_1);
			cout<<"cria txt 1"<<endl; 
		}
		if (x==2){
			fprintf(fp,"%s \t %s \t %f \t %f \t %f \n", colision_time.name_file.c_str(), colision_time.sit_type_2.c_str(),colision_time.sit_begin_2, colision_time.duration_2,  colision_time.sit_end_2);
			cout<<"cria txt 2"<<endl; 
		}
		if (x==3){
			fprintf(fp,"%s \t %s \t %f \t %f \t %f \n", colision_time.name_file.c_str(), colision_time.sit_type_3.c_str(),colision_time.sit_begin_3, colision_time.duration_3, colision_time.sit_end_3);
			cout<<"cria txt 3"<<endl; 
		}
		if (x==4){
			fprintf(fp,"%s \t %s \t %f \t %f \t %f \n", colision_time.name_file.c_str(), colision_time.sit_type_4.c_str(),colision_time.sit_begin_4, colision_time.duration_4,  colision_time.sit_end_4);
			cout<<"cria txt 4"<<endl; 
		}
		
//      cout <<"antes de sair."<<endl;
     fclose(fp);

  return 0;
};


/**
 * \brief inicialization of each situation to zero
 * 
 * \return void
 */
void initialization_variables(void)
{
	colision_time.internal_1=0;
	colision_time.internal_2=0;
	colision_time.internal_3=0;
	colision_time.internal_4=0;
}

/**
 * \brief Verification of each situation,
 * 
 * \return void
 */
void begin_situation(void)
{
	//cout<<"entrou no begin_situation "<<endl; 
	//cout<<"internal_1: "<< colision_time.internal_1<<" sit_1: "<< colision_time.sit_1<<endl; 
	//cout<< "data do ficheiro bag"<<colision_time.bag_time<<endl;
// 	 		verifica se ainda está em situação de perigo numero 1-> Objecto detectado na parte lateral do carro

	if(colision_time.sit_1==1 && colision_time.internal_1==0) //só entra aqui umavez, pois a segunda o valor internal é 1, logo faz else
		{
			//sacar o tempo da primeira vez que é apanhado a situação tempo inicial
			colision_time.lst_obj_sit_1=ros::Time::now().toSec(); // colision_time.bag_time;
			colision_time.sit_begin_1=ros::Time::now().toSec();
			colision_time.internal_1=1;
		}
	if (colision_time.sit_1==1 && colision_time.internal_1==1) //para todas as vezes que seja chamado a verificar a data
		{colision_time.lst_obj_sit_1=ros::Time::now().toSec(); }

	if(ros::Time::now().toSec()-colision_time.lst_obj_sit_1<1 )  //se o tempo que passou for menor que 300 milisegundos assumimos que tem objecto durente esse tempo, para evitar ruido no ficheiro a gravar.
		{
			colision_time.sit_1=1;
			//variavel interna que só é usada uma vez
		}else 
		{
			if(colision_time.sit_1==0 && colision_time.internal_1==1)
			{
				colision_time.sit_type_1="object detected--> vehicle behind the car";
				//colision_time.sit_begin=colision_time.lst_obj_sit_1; // inicio da situação
				colision_time.duration_1=colision_time.bag_time-colision_time.sit_begin_1; //duração da situação
				colision_time.sit_end_1=colision_time.bag_time; // final da situação
				
				//colision_time.sit_1=0;
				colision_time.internal_1=0;
				cria_txt(1);
				
			}
		}


	if(colision_time.sit_2==1 && colision_time.internal_2==0) //só entra aqui umavez, pois a segunda o valor internal é 1, logo faz else
		{
			//sacar o tempo da primeira vez que é apanhado a situação tempo inicial
			colision_time.lst_obj_sit_2=ros::Time::now().toSec(); // colision_time.bag_time;
			colision_time.sit_begin_2=ros::Time::now().toSec();
			colision_time.internal_2=1;
		}
	if (colision_time.sit_2==1 && colision_time.internal_2==1) //para todas as vezes que seja chamado a verificar a data
		{colision_time.lst_obj_sit_2=ros::Time::now().toSec(); }

	if(ros::Time::now().toSec()-colision_time.lst_obj_sit_2<1 )  //se o tempo que passou for menor que 300 milisegundos assumimos que tem objecto durente esse tempo, para evitar ruido no ficheiro a gravar.
		{
			colision_time.sit_2=1;
			//variavel interna que só é usada uma vez
		}else 
		{
			if(colision_time.sit_2==0 && colision_time.internal_2==1)
			{
				colision_time.sit_type_2="object detected--> Colision Risk";
				//colision_time.sit_begin=colision_time.lst_obj_sit_1; // inicio da situação
				colision_time.duration_2=colision_time.bag_time-colision_time.sit_begin_2; //duração da situação
				colision_time.sit_end_2=colision_time.bag_time; // final da situação
				
				//colision_time.sit_1=0;
				colision_time.internal_2=0;
				cria_txt(2);
				cout<<"cria txt 2"<<endl; 
			}
		}

if(colision_time.sit_3==1 && colision_time.internal_3==0) //só entra aqui umavez, pois a segunda o valor internal é 1, logo faz else
		{
			//sacar o tempo da primeira vez que é apanhado a situação tempo inicial
			colision_time.lst_obj_sit_3=ros::Time::now().toSec(); // colision_time.bag_time;
			colision_time.sit_begin_3=ros::Time::now().toSec();
			colision_time.internal_3=1;
		}
	if (colision_time.sit_3==1 && colision_time.internal_3==1) //para todas as vezes que seja chamado a verificar a data
		{colision_time.lst_obj_sit_3=ros::Time::now().toSec(); }

	if(ros::Time::now().toSec()-colision_time.lst_obj_sit_3<1 )  //se o tempo que passou for menor que 300 milisegundos assumimos que tem objecto durente esse tempo, para evitar ruido no ficheiro a gravar.
		{
			colision_time.sit_3=1;
			//variavel interna que só é usada uma vez
		}else 
		{
			if(colision_time.sit_3==0 && colision_time.internal_3==1)
			{
				colision_time.sit_type_3="object detected--> Forgot to blink left!";
				//colision_time.sit_begin=colision_time.lst_obj_sit_1; // inicio da situação
				colision_time.duration_3=colision_time.bag_time-colision_time.sit_begin_3; //duração da situação
				colision_time.sit_end_3=colision_time.bag_time; // final da situação
				
				//colision_time.sit_1=0;
				colision_time.internal_3=0;
				cria_txt(3);
				cout<<"cria txt 3"<<endl; 
			}
		}

if(colision_time.sit_4==1 && colision_time.internal_4==0) //só entra aqui umavez, pois a segunda o valor internal é 1, logo faz else
		{
			//sacar o tempo da primeira vez que é apanhado a situação tempo inicial
			colision_time.lst_obj_sit_4=ros::Time::now().toSec(); // colision_time.bag_time;
			colision_time.sit_begin_4=ros::Time::now().toSec();
			colision_time.internal_4=1;
		}
	if (colision_time.sit_4==1 && colision_time.internal_4==1) //para todas as vezes que seja chamado a verificar a data
		{colision_time.lst_obj_sit_4=ros::Time::now().toSec(); }

	if(ros::Time::now().toSec()-colision_time.lst_obj_sit_4<1 )  //se o tempo que passou for menor que 300 milisegundos assumimos que tem objecto durente esse tempo, para evitar ruido no ficheiro a gravar.
		{
			colision_time.sit_4=1;
			//variavel interna que só é usada uma vez
		}else 
		{
			if(colision_time.sit_4==0 && colision_time.internal_4==1)
			{
				colision_time.sit_type_4="object detected--> Forgot to blink right";
				//colision_time.sit_begin=colision_time.lst_obj_sit_1; // inicio da situação
				colision_time.duration_4=colision_time.bag_time-colision_time.sit_begin_4; //duração da situação
				colision_time.sit_end_4=colision_time.bag_time; // final da situação
				
				//colision_time.sit_1=0;
				colision_time.internal_4=0;
				cria_txt(4);
				cout<<"cria txt 4 "<<endl; 
			}
		}

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
// 		};
// 	

/**
 * \brief Callback to save the msg from Velocity Arduino
 * 
 * \return void
 */
// void topic_callback_velocity(atlascar_base::AtlascarVelocityStatus AtlascarVelocityStatus_msg)
// 		{
// 				velocity_msg.counting=AtlascarVelocityStatus_msg.counting;
// 				velocity_msg.pulses_sec=AtlascarVelocityStatus_msg.pulses_sec;
// 				velocity_msg.revolutions_sec=AtlascarVelocityStatus_msg.revolutions_sec;
// 				velocity_msg.velocity=AtlascarVelocityStatus_msg.velocity;
// 		};
/**
 * \brief Callback to save the msg from atlascar PLC
 * 
 * \return void
 */		
// void topic_callback_plc_status(atlascar_base::AtlascarStatus AtlascarStatus_msg)
// 		{
// 				plc_msg.steering_wheel=AtlascarStatus_msg.steering_wheel;
// 				plc_msg.rpm=AtlascarStatus_msg.rpm;
// 				cout<<plc_msg <<endl;
// 		};
			
/**
 * \brief Callback to save the msg from the name of bag file
 * 
 * \return void
 */	
void topic_callback_rosout_cool(rosgraph_msgs::Log rosout_msg)
		{
			
			if (rosout_msg.msg[0]=='O' && rosout_msg.msg[1]=='p'&& rosout_msg.msg[2]=='e'&& rosout_msg.msg[6]=='g')
			{
				char name_file[1024];
				sscanf(rosout_msg.msg.c_str(),"%*s %s",name_file); 
				colision_time.name_file= name_file;
				
				//cout<< "msg_txt:"<<name_file <<endl;
				//cria_txt();
			}
		};

		
/**
 * \brief Callback to read the msg from mtt pacagde.
 * 
 * This is one of the principal functions in this code, because this function
identifies, if an obstacle is in or out of the search zones. And calculates the
colision_time from the car to the object.
*
*
 * \return void
 */
void topic_callback(const mtt::TargetListPC::Ptr& msg)
{
	colision_time.bag_time=ros::Time::now().toSec();
	colision_time.sit_1=0;
	colision_time.sit_2=0;
	colision_time.sit_3=0;
	colision_time.sit_4=0;
	
	
	//cout<<colision_time.bag_time<<" isto foi a data" <<endl;
	
	PointCloud<PointXYZ> velocity;
	PointCloud<PointXYZ> velocity_1;
	PointCloud<PointXYZ> position_1; 
	PointCloud<PointXYZ> position;
  
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(msg->velocity, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, velocity_1);
    
    pcl_conversions::toPCL(msg->position, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, position_1);
    
// 	fromROSMsg(msg->velocity,velocity_1);
// 	fromROSMsg(msg->position,position_1);
	
	tf::StampedTransform transform;
    try
      {
		listener_center_bumper_ptr->lookupTransform("/atc/vehicle/center_bumper", msg->header.frame_id, ros::Time(0), transform);
      }
    catch (tf::TransformException ex)
      {
		ROS_ERROR("%s",ex.what());
      }
		
			pcl_ros::transformPointCloud (position_1,position,transform); 
			pcl_ros::transformPointCloud (velocity_1,velocity,transform);
			position.header.frame_id="/atc/vehicle/center_bumper";
			velocity.header.frame_id="/atc/vehicle/center_bumper";
			
			PointCloud<PointXYZ>::Ptr front_zone (new PointCloud<PointXYZ> ());
			front_zone->header.frame_id="/atc/vehicle/center_bumper";
			
			
			PointCloud<PointXYZ>::Ptr left_zone (new PointCloud<PointXYZ> ());
			left_zone->header.frame_id="/atc/vehicle/center_bumper";
	
	
	PointXYZ point;
	//PointXYZ point_side_left;
	for (int i=0; i<(int)position.size(); ++i)
	{
	  point=position.points[i];
	      //Front of the car!!
	  if (point.x>0 && point.x<40 && point.y>-1 && point.y<1)
	      {
		front_zone->points.push_back(point);
		obstacle_zone_front.id=msg->id[i];
		obstacle_zone_front.x=point.x;
		obstacle_zone_front.y=point.y;
		obstacle_zone_front.z=point.z;
		obstacle_zone_front.v_x=velocity[i].x;
		obstacle_zone_front.v_y=velocity[i].y;
		obstacle_zone_front.v_z=velocity[i].z;
	      }  

	  if (point.x>-40 && point.x<-0.5 && point.y>1.2 && point.y<3.5)
	      {
		left_zone->points.push_back(point);
		obstacle_left.id=msg->id[i];
		obstacle_left.x=point.x;
		obstacle_left.y=point.y;
		obstacle_left.z=point.z;
		obstacle_left.v_x=velocity[i].x;
		obstacle_left.v_y=velocity[i].y;
		obstacle_left.v_z=velocity[i].z;
	      }  
	  
		}

      sensor_msgs::PointCloud2 ptc_front;
      toROSMsg(*front_zone,ptc_front);
      ptc_front.header.frame_id="/atc/vehicle/center_bumper";
      points_pub.publish(ptc_front);
      
      
      sensor_msgs::PointCloud2 ptc_side;
      toROSMsg(*left_zone,ptc_side);
      ptc_side.header.frame_id="/atc/vehicle/center_bumper";
      points_pub.publish(ptc_side);
      
      
      
      ////////////////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////PARTE FRONTAL DO CARRO//////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////
      
      //reference of points with a cube
      visualization_msgs::Marker marker_front=mark_cluster(front_zone, "front_cube" ,0, 0, 0, 1);
      marker_fronts_publisher.publish(marker_front);

      //cout <<obstacle_zone_front <<endl;
      //condição para entrar na situação de risco!!
      float velocity_car_front;
      float time_car_front;
      velocity_car_front=obstacle_zone_front.v_x+velocity_msg.velocity;
      time_car_front=obstacle_zone_front.x/obstacle_zone_front.v_x;
      
      cout<<"V Atlascar ->"<< velocity_msg.velocity<<endl;
      cout<<"V objecto->"<< obstacle_zone_front.v_x<< endl;
      cout<<"V total ->"<< velocity_car_front <<endl;
      cout<<"eime total ->"<<time_car_front <<endl;
      
	  if(time_car_front>-4 && time_car_front< 0)//obstacle_zone_front.x < 10 && obstacle_zone_front.v_x < -2)
	      {
		colision_time.sit_2=1;
		string msg_text="Colision Risk!!";
		visualization_msgs::Marker marker_front_text_front=generic_text( obstacle_zone_front.x , obstacle_zone_front.y, obstacle_zone_front.z , 1, 0, 0, 1, msg_text , "/atc/vehicle/center_bumper", "front_warning");
			
		marker_front_text_publisher.publish(marker_front_text_front);    
		      
		  //inserir condição para caso entre aqui, contar o tempo que aqui passou, 
		  //terá um intervalo superior a 0,2segundos, para verificar que ainda tem alguma.
		
	      }
      //tratamento de dados, tentar visualizar a velocidade em x do ponto.
     
      
      ////////////////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////PARTE LATERAL DO CARRO//////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////
       
      
      //reference of points with a cube
      visualization_msgs::Marker marker_side=mark_side_cluster(left_zone, "side_cube" ,0, 0, 0, 1);
      marker_side_publisher.publish(marker_side);
      
      
	  ///////////////////////////////////////////////////////////////////////////////////////
	  ////////////////////////////////Objecto lateral detectado//////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////////      
      //reference to text and id of the point more 

      //cout<<"nuvem de pontos esquerda"<<left_zone->points.size()<<endl;
      long int exist_points_side=left_zone->points.size();
      if(exist_points_side>0)
	  {
		colision_time.sit_1=1;
		
	    string msg_text_side="OBJECT DETECTED";
	    visualization_msgs::Marker marker_side_text =generic_text( 0 , obstacle_left.y, 2 , 0, 0, 0.487, 1, msg_text_side ,"/atc/vehicle/center_bumper","cannot_pass");
	
	    marker_side_text_publisher.publish(marker_side_text);    	    
	  }/*else{colision_time.sit_1=0;}*/
	  
	  ///////////////////////////////////////////////////////////////////////////////////////
	  ////////////////////////////////Falha de Pisca esquerdo////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////////
      
	  if( partial_msg.lights_left==0 &&  plc_msg.steering_wheel<(-0.315)  ) //falta inserir se existe alguem a frente do carro!!
	{
	  colision_time.sit_3=1;
	  string msg_blink_side="Forgot to blink left!";
	    
	  visualization_msgs::Marker marker_side_text_left_blink= generic_text( 3 , 6, 2 , 0, 0, 1, 1, msg_blink_side ,"/atc/vehicle/center_bumper","Blink_left");
	  
	  marker_generic_text_blink_left_publisher.publish(marker_side_text_left_blink);
	  //marker_side_text_blink_publisher.publish(marker_side_text_blink);
	}
	 
	 ///////////////////////////////////////////////////////////////////////////////////////
	  ////////////////////////////////Falha de Pisca direito/////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////////
      
	 //long int exist_points_front=front_zone->points.size();
	 if(  partial_msg.lights_right==0 && plc_msg.steering_wheel>(-0.275) && plc_msg.steering_wheel!=0 ) //falta inserir se existe alguem a frente do carro!!
	{
	  colision_time.sit_4=1;
	  string msg_blink_side="Forgot to blink right!";
	    
	  visualization_msgs::Marker marker_side_text_right_blink= generic_text(3 , -6, 2 , 0, 0, 1, 1, msg_blink_side ,"/atc/vehicle/center_bumper","Blink_right");
	  
	  marker_side_text_blink_publisher.publish(marker_side_text_right_blink);
	}
	
	begin_situation();
 
//  plc_msg={0};
// obstacle_zone_front={0};
	obstacle_zone_front.v_x =0;
};


		
/**
 * \brief main of the program, includes the callback funtions associated
 * 
 * \return 1 
 */
int main (int argc, char **argv)
{
  ROS_INFO("Starting colision time node.");
  
  ros::init(argc,argv,"colision_time"); 
  ros::NodeHandle n;
  
  initialization_variables();
  
  //class_colision_time cenas_fixes;
  
  tf::TransformListener listener_center_bumper;   // atc/vehicle/center_bumper
  listener_center_bumper_ptr=&listener_center_bumper;
  
  //   Subscribe mtt message
  ros::Subscriber sub = n.subscribe("/mtt/trck/targets", 1, topic_callback);
//   ros::Subscriber sub_status_plc = n.subscribe("/atc/base/status/plc", 1, topic_callback_plc_status);
//   ros::Subscriber sub_partial_status = n.subscribe("/partial_status", 1, topic_callback_partial);
//   ros::Subscriber sub_velocity_status = n.subscribe("/velocity_status", 1, topic_callback_velocity);
  ros::Subscriber sub_rosout =n.subscribe("/rosout_agg", 1, topic_callback_rosout_cool);
  
  
  ROS_ERROR("AtlascarPartialStatus and AtlascarVelocityStatus no longer avaliable, please correct!!");
//   ros::Subscriber sub_status_plc = n.subscribe("/vhc/plc/status", 1, topic_callback_plc_status);
//   ros::Subscriber sub_partial_status = n.subscribe("/vhc/driver/status", 1, topic_callback_partial);
//   ros::Subscriber sub_velocity_status = n.subscribe("/vhc/velocity/status", 1, topic_callback_velocity);
  
  
  
  //  Publicador de pontos dentro da região
  points_pub = n.advertise<sensor_msgs::PointCloud2>("/atc/monitoring/danger_zone", 1);
 
//   cubo frontal
  marker_fronts_publisher= n.advertise<visualization_msgs::Marker>("/prcp/front_mrk", 1000);
    
  //cubo lateral
  marker_side_publisher= n.advertise<visualization_msgs::Marker>("/prcp/side_mrk", 1000);
  
  //colision Risk
  marker_front_text_publisher= n.advertise<visualization_msgs::Marker>("/prcp/front_text_mrk", 1000);
  
  //objecto detectado zona na lateral
  marker_side_text_publisher= n.advertise<visualization_msgs::Marker>("/prcp/side_text_mrk", 1000);

  
  //lado direito
  marker_side_text_blink_publisher= n.advertise<visualization_msgs::Marker>("/prcp/side_text_blink_right_mrk", 1000);
  
  //lado esquerdo
  marker_generic_text_blink_left_publisher= n.advertise<visualization_msgs::Marker>("/prcp/side_text_blink_left_mrk", 1000);
  
  
 
  ros::spin();
  
  return 1;
}


		
/**
 * \brief Function responsable for defining generic text in rviz.
 * \param[in] x position X of text
 * \param[in] y position Y of text
 * \param[in] z position Z of text
 * \param[in] r color on channel red of the text
 * \param[in] g color on channel green of the text
 * \param[in] b color on channel blue of the text
 * \param[in] scale text scale
 * \param[in] txt letters to be written on rviz
 * \param[in] frm_id frame id of the marker
 * \param[in] ns id number of the marker
 * \return marker_front the marker_front publish
 */
visualization_msgs::Marker generic_text(float x, float y, float z , float r,
float g, float b, float scale, std::string txt, std::string frm_id, std::string
ns)
{
  uint32_t shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
  visualization_msgs::Marker marker_front;
  
  marker_front.header.frame_id = frm_id;
  marker_front.header.stamp = ros::Time::now();
  
  
  marker_front.ns = ns;
  marker_front.id = 0;
  marker_front.type = shape;
  marker_front.action = visualization_msgs::Marker::ADD;
  
  marker_front.pose.position.x = x;
  marker_front.pose.position.y = y;
  marker_front.pose.position.z = z;
  marker_front.scale.z = scale;
  
   
  //message to send in the msg.
  
  marker_front.text=txt;
 
  marker_front.color.r = r;
  marker_front.color.g = g;
  marker_front.color.b = b;
  marker_front.color.a = 0.8;
  
  marker_front.lifetime = ros::Duration(0.4);
  return marker_front;
}






/** 
 * \brief function to estimate a marker_front arround the cluster to show on rviz
 * \param[in] cloud_cluster pointer to the cluster
 * \param[in] ns string with the classification name
 * \param[in] id the marker_front ident
 * \param[in] r the r channel color
 * \param[in] g the g channel color
 * \param[in] b the b channel color
 * \return marker_front the marker_front to publish
 */
visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b)
{
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  
  pcl::compute3DCentroid (*cloud_cluster, centroid);
  pcl::getMinMax3D (*cloud_cluster, min, max);
  
  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker_front;
  marker_front.header.frame_id = cloud_cluster->header.frame_id;
  marker_front.header.stamp = ros::Time::now();
  
  marker_front.ns = ns;
  marker_front.id = id;
  marker_front.type = shape;
  marker_front.action = visualization_msgs::Marker::ADD;
  
  
  //market's position is in the minimum x cordinate of the point
  marker_front.pose.position.x = min[0];
  marker_front.pose.position.y = centroid[1];
  marker_front.pose.position.z = centroid[2];
  marker_front.pose.orientation.x = 0.0;
  marker_front.pose.orientation.y = 0.0;
  marker_front.pose.orientation.z = 0.0;
  marker_front.pose.orientation.w = 1.0;
  
  marker_front.scale.x = 1;//(max[0]-min[0]);
  marker_front.scale.y = 1;//(max[1]-min[1]);
  marker_front.scale.z = 1;//(max[2]-min[2]);

  marker_front.color.r = r;
  marker_front.color.g = g;
  marker_front.color.b = b;
  marker_front.color.a = 0.5;
  
//   marker_front.lifetime = ros::Duration();
    marker_front.lifetime = ros::Duration(0.3);
  return marker_front;
}





/** 
 * \brief function to estimate a marker_front arround the cluster to show on rviz
 * \param[in] cloud_cluster pointer to the cluster
 * \param[in] ns string with the classification name
 * \param[in] id the marker_front ident
 * \param[in] r the r channel color
 * \param[in] g the g channel color
 * \param[in] b the b channel color
 * \return marker_front the marker_front to publish
 */
visualization_msgs::Marker mark_side_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b)
{
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  
  pcl::compute3DCentroid (*cloud_cluster, centroid);
  pcl::getMinMax3D (*cloud_cluster, min, max);
  
  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker_side;
  marker_side.header.frame_id = cloud_cluster->header.frame_id;
  marker_side.header.stamp = ros::Time::now();
  
  marker_side.ns = ns;
  marker_side.id = id;
  marker_side.type = shape;
  marker_side.action = visualization_msgs::Marker::ADD;
  
  
  //market's position is in the minimum x cordinate of the point
  marker_side.pose.position.x = max[0];
  marker_side.pose.position.y = centroid[1];
  marker_side.pose.position.z = centroid[2];
  marker_side.pose.orientation.x = 0.0;
  marker_side.pose.orientation.y = 0.0;
  marker_side.pose.orientation.z = 0.0;
  marker_side.pose.orientation.w = 1.0;
  
  marker_side.scale.x = 1;//(max[0]-min[0]);
  marker_side.scale.y = 1;//(max[1]-min[1]);
  marker_side.scale.z = 1;//(max[2]-min[2]);

  marker_side.color.r = r;
  marker_side.color.g = g;
  marker_side.color.b = b;
  marker_side.color.a = 1;
  
//   marker_front.lifetime = ros::Duration();
    marker_side.lifetime = ros::Duration(0.3);
  return marker_side;
}

