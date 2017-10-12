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
  \brief The node recive the masseges from manager and NavSatFix, pick the car that is needed and them send them to the
* remote DB. When the program is stopped a remote backup is created and after that the DataBase is cleared.
 \file remote_monitor.cpp
 \author José Viana
 **/


#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>

#include <stdio.h>
#include <time.h>


//icludes packages

#include <database_interface/db_class.h>
#include <boost/shared_ptr.hpp>
#include <database_interface/postgresql_database.h>
#include <libpq-fe.h>

//includes of the messages
#include <atlascar_base/ManagerStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>

#include <boost/thread/thread.hpp>

using namespace std;


int argc_g;
char** argv_g;

//string for time and date 
const string currentDateTimedate();
const string currentDateTime();
/** 
 \brief Function for database query execute
  \details This class makes a new two commands for copy and deleting a table from data base
  \return void
 */
class Database: public database_interface::PostgresqlDatabase
{
	public:
		Database(std::string server,std::string port,std::string username,std::string pass,std::string table):
		PostgresqlDatabase("lars.mec.ua.pt", "5432","atlas", "atlas2013", "atlas_monitoring")
		{
			touch();
		}
		
		class PGresultAutoPtr
		{
			private:
				PGresult* result_;
			public:
				PGresultAutoPtr(PGresult *ptr):
				result_(ptr)
				{}
				
				~PGresultAutoPtr()
				{
					PQclear(result_);
				}
				
				void reset(PGresult *ptr)
				{
					PQclear(result_);
					result_=ptr;
				}
				
				PGresult* operator * ()
				{
					return result_;
				}
		};
		//Função nova criada que apaga a tabela existente
		bool cleanTable()
		{
			std::string query("DELETE FROM data_table_atlas;");
			PGresultAutoPtr result(PQexec(connection_, query.c_str()));  
			return true;
		}
		
		
		bool copyTable()
		{
			std::string query("COPY data_table_atlas TO '/opt/atlascar/atlascar_" + currentDateTimedate()+"_"+currentDateTime()+".csv' DELIMITER ',' CSV HEADER;");
			PGresultAutoPtr result(PQexec(connection_, query.c_str()));  			
			return true;
		}
		
		void touch()
		{
			tic_=ros::Time::now();
		}
		
		ros::Time tic_;
};
/** 
 \brief Function for database structure criation
 \details This class makes creates the structure of database
 \return void
 */
class DatabaseStructure : public database_interface::DBClass
{

//fields are made public in this toy example for easy access, but you
//can treat them as you would any member data of your C++ classes
public:

  //key requirement: all fields that are to be stored in the 
  //database must be wrapped as database_interface::DBField<>, 
  //templated on the type of data they hold
  
	database_interface::DBField<int> id_;
	database_interface::DBField<int> rpm_;
	database_interface::DBField<double> speed_;
	database_interface::DBField<std::string> db_time_;
	database_interface::DBField<std::string> atlas_time_;//ttime send by the car to the db to compare with
	database_interface::DBField<int> throttle_;
	database_interface::DBField<int> brake_;
	database_interface::DBField<int> clutch_;
	database_interface::DBField<int> gear_;
	database_interface::DBField<double> steering_;
	database_interface::DBField<double> throttle_press_;
	database_interface::DBField<double> brake_press_;
	database_interface::DBField<double> clutch_press_;
	database_interface::DBField<int> auto_throttle_;
	database_interface::DBField<int> auto_brake_;
	database_interface::DBField<int> auto_clutch_;
	database_interface::DBField<int> auto_steering_;
	database_interface::DBField<int> auto_ignition_;
	database_interface::DBField<int> handbrake_;
	database_interface::DBField<int> emergency_;
	database_interface::DBField<int> lights_minimum_;
	database_interface::DBField<int> lights_medium_;
	database_interface::DBField<int> lights_high_;
	database_interface::DBField<int> lights_left_;
	database_interface::DBField<int> lights_right_;
	database_interface::DBField<int> lights_brake_;
	database_interface::DBField<int> lights_reverse_;
	database_interface::DBField<int> lights_warning_;
	database_interface::DBField<int> horn_;
	database_interface::DBField<double> lat_gps_;
	database_interface::DBField<double> long_gps_;
	database_interface::DBField<double> alt_gps_;
	database_interface::DBField<double> track_gps_;
// 	This are some empty fiels for future data
	
	
	database_interface::DBField<int> empty_field3_;
	database_interface::DBField<int> empty_field4_;
	
  
  
  
  /***
   This defines the type of fiels and the name of them in the table   
   ***/
  DatabaseStructure() : 
		id_(database_interface::DBFieldBase::TEXT,
			this, "id", "data_table_atlas", true),
		rpm_(database_interface::DBFieldBase::TEXT, 
			this, "rpm", "data_table_atlas", true),
		speed_(database_interface::DBFieldBase::TEXT, 
			this, "speed", "data_table_atlas", true),
		db_time_(database_interface::DBFieldBase::TEXT, 
			this, "db_time", "data_table_atlas", true),
		atlas_time_(database_interface::DBFieldBase::TEXT, 
			this, "atlas_time", "data_table_atlas", true),
		throttle_(database_interface::DBFieldBase::TEXT, 
			this, "throttle", "data_table_atlas", true),
		brake_(database_interface::DBFieldBase::TEXT, 
			this, "brake", "data_table_atlas", true),
		clutch_(database_interface::DBFieldBase::TEXT, 
			this, "clutch", "data_table_atlas", true),
		gear_(database_interface::DBFieldBase::TEXT, 
			this, "gear", "data_table_atlas", true),
		steering_(database_interface::DBFieldBase::TEXT, 
			this, "steering", "data_table_atlas", true),
		throttle_press_(database_interface::DBFieldBase::TEXT, 
			this, "throttle_press", "data_table_atlas", true),
		brake_press_(database_interface::DBFieldBase::TEXT, 
			this, "brake_press", "data_table_atlas", true),
		clutch_press_(database_interface::DBFieldBase::TEXT, 
			this, "clutch_press", "data_table_atlas", true),
		auto_throttle_(database_interface::DBFieldBase::TEXT, 
			this, "auto_throttle", "data_table_atlas", true),
		auto_brake_(database_interface::DBFieldBase::TEXT, 
			this, "auto_brake", "data_table_atlas", true),		
		auto_clutch_(database_interface::DBFieldBase::TEXT, 
			this, "auto_clutch", "data_table_atlas", true),	
		auto_steering_(database_interface::DBFieldBase::TEXT, 
			this, "auto_steering", "data_table_atlas", true),	
		auto_ignition_(database_interface::DBFieldBase::TEXT, 
			this, "auto_ignition", "data_table_atlas", true),
		handbrake_(database_interface::DBFieldBase::TEXT, 
			this, "handbrake", "data_table_atlas", true),
		emergency_(database_interface::DBFieldBase::TEXT, 
			this, "emergency", "data_table_atlas", true),
		lights_minimum_(database_interface::DBFieldBase::TEXT, 
			this, "lights_minimum", "data_table_atlas", true),
		lights_medium_(database_interface::DBFieldBase::TEXT, 
			this, "lights_medium", "data_table_atlas", true),
		lights_high_(database_interface::DBFieldBase::TEXT, 
			this, "lights_high", "data_table_atlas", true),	
		lights_left_(database_interface::DBFieldBase::TEXT, 
			this, "lights_left", "data_table_atlas", true),	
		lights_right_(database_interface::DBFieldBase::TEXT, 
			this, "lights_right", "data_table_atlas", true),	
		lights_brake_(database_interface::DBFieldBase::TEXT, 
			this, "lights_brake", "data_table_atlas", true),
		lights_reverse_(database_interface::DBFieldBase::TEXT, 
			this, "lights_reverse", "data_table_atlas", true),
		lights_warning_(database_interface::DBFieldBase::TEXT, 
			this, "lights_warning", "data_table_atlas", true),
		horn_(database_interface::DBFieldBase::TEXT, 
			this, "horn", "data_table_atlas", true),
		lat_gps_(database_interface::DBFieldBase::TEXT, 
			this, "lat_gps", "data_table_atlas", true),	
		long_gps_(database_interface::DBFieldBase::TEXT, 
			this, "long_gps", "data_table_atlas", true),
		alt_gps_(database_interface::DBFieldBase::TEXT, 
			this, "alt_gps", "data_table_atlas", true),	
		track_gps_(database_interface::DBFieldBase::TEXT, 
			this, "track_gps", "data_table_atlas", true),
		empty_field3_(database_interface::DBFieldBase::TEXT, 
			this, "empty_field3", "data_table_atlas", true),
		empty_field4_(database_interface::DBFieldBase::TEXT, 
			this, "empty_field4", "data_table_atlas", true)
		
  {	
			

  
    //finally, all fields must be registered with the DBClass itself
    //one field MUST be a primary key
    //all instances of DBClass have a primary_key_field_ pointer, 
    //which must be set on construction
    
    
    /***
     Defenition of all fiels, one of them must be the PRIMARY KEY, and definesd as it. 
     ***/
    primary_key_field_ = &id_;
	//all other fields go into the fields_ array of the DBClass
	fields_.push_back(&rpm_);
	fields_.push_back(&speed_);
	
	fields_.push_back(&db_time_);
	fields_.push_back(&atlas_time_);
	fields_.push_back(&throttle_);
	fields_.push_back(&brake_);
	fields_.push_back(&clutch_);
	fields_.push_back(&gear_);
	fields_.push_back(&steering_);
	fields_.push_back(&throttle_press_);
	fields_.push_back(&brake_press_);
	fields_.push_back(&clutch_press_);
	fields_.push_back(&auto_throttle_);
	fields_.push_back(&auto_brake_);
	fields_.push_back(&auto_clutch_);
	fields_.push_back(&auto_steering_);
	fields_.push_back(&auto_ignition_);
	fields_.push_back(&handbrake_);
	fields_.push_back(&emergency_);
	fields_.push_back(&lights_minimum_);
	fields_.push_back(&lights_medium_);
	fields_.push_back(&lights_high_);
	fields_.push_back(&lights_left_);
	fields_.push_back(&lights_right_);
	fields_.push_back(&lights_brake_);
	fields_.push_back(&lights_reverse_);
	fields_.push_back(&lights_warning_);
	fields_.push_back(&horn_);
	fields_.push_back(&lat_gps_);
	fields_.push_back(&long_gps_);
	fields_.push_back(&alt_gps_);
	fields_.push_back(&track_gps_);
	
	//empty fields for future
	
	fields_.push_back(&empty_field3_);
	fields_.push_back(&empty_field4_);
	
	db_time_.data()=currentDateTimedate();
	throttle_press_.data()=0;
	brake_press_.data()=0;
	clutch_press_.data()=0;
	lights_left_.data()=0;
	lights_right_.data()=0;
	horn_.data()=0;
	lat_gps_.data()=0;
	long_gps_.data()=0;
	alt_gps_.data()=0;
	track_gps_.data()=0;
	empty_field3_.data()=0;
	empty_field4_.data()=0;
	
	rpm_.data()=0;
	
	throttle_.data()=0;
	brake_.data()=0;
	clutch_.data()=0;
	gear_.data()=0;
	
	steering_.data()=0;
	
	auto_throttle_.data()=0;
	auto_brake_.data()=0;
	auto_clutch_.data()=0;
	auto_steering_.data()=0;
	auto_ignition_.data()=0;
	
	lights_minimum_.data()=0;
	lights_medium_.data()=0;
	lights_high_.data()=0;
	handbrake_.data()=0;
	
	lights_brake_.data()=0;
	lights_reverse_.data()=0;
	lights_warning_.data()=0;
	emergency_.data()=0;
	
    //optional: let all fields be read automatically when an instance 
    // is retrieved from the database
    setAllFieldsReadFromDatabase(true);
    //optional: let all fields be written automatically when an instance 
    //is saved the database
    setAllFieldsWriteToDatabase(true);
    //(these options are usful if you have a very large field (e.g. a 
    // binary bitmap with the picture of the student) which you do not 
    //want retrieved automatically whenever you get a student info 
    //from the database
  }
  

  /*** 
  Creation of the vector containing all the fiels of the database
 ***/
  std::vector<database_interface::DBFieldBase*> getFields()
  {
	 return fields_; 
  }
  
};

typedef boost::shared_ptr<DatabaseStructure> DatabaseStructurePtr;



/** 
 \brief Function for geting a line from table with a specific id
 \details This fuction 
 \param DatabaseStructPtr
 \param id
 \return void
 */
DatabaseStructurePtr getDatabaseStructureById(vector<DatabaseStructurePtr>& atlas_status,int id)
{
// 	for(uint i=0;i<atlas_status.size();i++)
// 		if(atlas_status[i]->id_.data()==id)
// 			return atlas_status[i];
// 	if (!database.getList(atlas_status))
// 		{
// 		  ROS_ERROR("Failed to get list of test objects");
// 		  return -1;
// 		}
// 
// 		ROS_INFO("Retrieved %zd objects", objects.size());
// 		if ( objects.size() != NUM_OBJECTS)
// 		{
// 		  ROS_ERROR("Expected %d objects", (int)NUM_OBJECTS);
// 		  return -1;
// 		}
	DatabaseStructurePtr null;
	return null;
}
/** 
 \brief Function for print all the recieved data
\details Print's all the recived data
 \return void
 */
ostream& operator<<(ostream& o, vector<DatabaseStructurePtr>& v)
{
	o << "Atlas Status size: " << v.size() <<endl;

	for (size_t i=0; i<v.size(); i++)
	{
		o << "Variaveis do AtlasCar: \n";
		
		o << "  id: " << v[i]->id_.data() << "\n";
		o << "  rpm: " << v[i]->rpm_.data() << "\n";
		o << "  speed: " << v[i]->speed_.data() << "\n";
		o << "  db_time: " << v[i]->db_time_.data() << "\n";
		o << "  atlas_time: " << v[i]->atlas_time_.data() << "\n";
		o << "  throttle: " << v[i]->throttle_.data() << "\n";
		o << "  brake: " << v[i]->brake_.data() << "\n";
		o << "  clutch: " << v[i]->clutch_.data() << "\n";
		o << "  gear: " << v[i]->gear_.data() << "\n";
		o << "  steering: " << v[i]->steering_.data() << "\n";
		o << "  throttle press: " << v[i]->throttle_press_.data() << "\n";
		o << "  brake press: " << v[i]->brake_press_.data() << "\n";
		o << "  clutch press: " << v[i]->clutch_press_.data() << "\n";
		o << "  auto throttle: " << v[i]->auto_throttle_.data() << "\n";
		o << "  auto brake: " << v[i]->auto_brake_.data() << "\n";
		o << "  auto clutch: " << v[i]->auto_clutch_.data() << "\n";
		o << "  auto steering: " << v[i]->auto_steering_.data() << "\n";
		o << "  auto ignition: " << v[i]->auto_ignition_.data() << "\n";
		o << "  handbrake: " << v[i]->handbrake_.data() << "\n";
		o << "  emergency: " << v[i]->emergency_.data() << "\n";
		o << "  lights minimum: " << v[i]->lights_minimum_.data() << "\n";
		o << "  lights medium: " << v[i]->lights_medium_.data() << "\n";
		o << "  lights high: " << v[i]->lights_high_.data() << "\n";
		o << "  lights left: " << v[i]->lights_left_.data() << "\n";
		o << "  lights right: " << v[i]->lights_right_.data() << "\n";
		o << "  lights brake: " << v[i]->lights_brake_.data() << "\n";
		o << "  lights reverse: " << v[i]->lights_reverse_.data() << "\n";
		o << "  lights warning: " << v[i]->lights_warning_.data() << "\n";
		o << "  horn: " << v[i]->horn_.data() << "\n";
		o << "  lat gps: " << v[i]->lat_gps_.data() << "\n";
		o << "  long gps: " << v[i]->long_gps_.data() << "\n";
	}
	
	return o;
}

using namespace std;
DatabaseStructure atlas_insert;//variavel global para introduzir os dados na DB

/** 
\brief Function for current time
 \details This function gets the instant time from the pc
 \return time
 */
// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const string currentDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://www.cplusplus.com/reference/clibrary/ctime/strftime/
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%X", &tstruct);
    return buf;
}

/** 
 \brief Function for current date
 \details This function gets the instant date from the pc
\return date
 */
const string currentDateTimedate()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://www.cplusplus.com/reference/clibrary/ctime/strftime/
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);
    return buf;
}

/** 
 \brief Function for current time
 \details This function gets the instant time from the pc
\return date
 */
 void ManagerStatusCallback(const atlascar_base::ManagerStatusPtr& msg)
{
	atlas_insert.db_time_.data()=currentDateTimedate();
// 	cout<<"data: "<<currentDateTimedate()<<endl;
	atlas_insert.rpm_.data()=msg->rpm;
	double velocidade;
	velocidade=((msg->velocity)*3600)/1000;

	atlas_insert.speed_.data()=velocidade;
	atlas_insert.throttle_.data()=msg->throttle;
	atlas_insert.brake_.data()=msg->brake;
	atlas_insert.clutch_.data()=msg->clutch;
	atlas_insert.gear_.data()=msg->gear;
	
	atlas_insert.steering_.data()=msg->steering_wheel;
	
	atlas_insert.auto_throttle_.data()=msg->auto_throttle;
	atlas_insert.auto_brake_.data()=msg->auto_brake;
	atlas_insert.auto_clutch_.data()=msg->auto_clutch;
	atlas_insert.auto_steering_.data()=msg->auto_direction;
	atlas_insert.auto_ignition_.data()=msg->auto_ignition;
	
	atlas_insert.lights_medium_.data()=msg->lights_medium;
	atlas_insert.lights_high_.data()=msg->lights_high;
	
	
	atlas_insert.lights_brake_.data()=msg->lights_brake;
	atlas_insert.lights_reverse_.data()=msg->lights_reverse;
	atlas_insert.lights_warning_.data()=msg->lights_warning;
	atlas_insert.emergency_.data()=msg->emergency;
	atlas_insert.throttle_press_.data()=msg->throttle;
	atlas_insert.brake_press_.data()=msg->brake;
	atlas_insert.clutch_press_.data()=msg->clutch;

	atlas_insert.lights_left_.data()=msg->lights_left;
	atlas_insert.lights_right_.data()=msg->lights_right;
	
	atlas_insert.horn_.data()=msg->horn;
	
	//messages for future handbrake and lights min non existing in manager
	atlas_insert.handbrake_.data()=0;
	atlas_insert.lights_minimum_.data()=0;
	atlas_insert.empty_field3_.data()=0;
	atlas_insert.empty_field4_.data()=0;
}


/** 
 \brief Function for GPS callback 
\details when a new message is recieved from the gps, store them in the variables
 \param pointer with the message
 \return
 */
void NavSatFixCallback(const sensor_msgs::NavSatFixPtr& nav_sat)
{
// 	atlas_insert.lat_gps_.data()=nav_sat->latitude;
// 	atlas_insert.long_gps_.data()=nav_sat->longitude;
// 	atlas_insert.alt_gps_.data()=nav_sat->altitude;
}

void GpsFixCallback(const gps_common::GPSFixPtr& gps)
{
	atlas_insert.lat_gps_.data()=gps->latitude;
	atlas_insert.long_gps_.data()=gps->longitude;
	atlas_insert.alt_gps_.data()=gps->altitude;
	atlas_insert.track_gps_.data()=gps->track;
}

void watchdog(Database**database)
{
	ros::Rate r(50);
	
	while(ros::ok())
	{
		r.sleep();
		
		if( (ros::Time::now() - (*database)->tic_).toSec() > 5. )
		{
			cout<<"disconnected"<<endl;
			cout<<"restarting process"<<endl;
			if(execvp(argv_g[0],argv_g)<0)
				perror("Cannot launch new process");
		}
		
	}
}

/** 
 \brief main function
 \details This function gets the instant time from the pc
\return date
 */
int main(int argc,char**argv)
{
	argc_g=argc;
	argv_g=argv;
	
	ros::init(argc, argv, "listener");
	ros::NodeHandle n("~");

	ros::Subscriber sub_driver = n.subscribe("manager_status", 1000, ManagerStatusCallback);
	ros::Subscriber sub_gps_fix = n.subscribe("gps_fix", 1000, GpsFixCallback);
	ros::Subscriber sub_nav_sat = n.subscribe("nav_sat_fix", 1000, NavSatFixCallback);

	//Connect to the database
	Database* database = new Database("lars.mec.ua.pt", "5432","atlas", "atlas2013", "atlas_monitoring");
	if (!database->isConnected())
	{
		std::cerr << "Database failed to connect \n";
	}else
	{
		std::cerr << "Database connected successfully \n";
		//check if database is empty, and if isn't make a copy
		std::vector< boost::shared_ptr<DatabaseStructure> > objects;
		
		if (!database->getList(objects))
		{
		  ROS_ERROR("Failed to get list of test objects");
		}else
		{
			if(objects.size()>0)
			{
				cout<<" The database isn't empty "<<endl;
				ROS_INFO("Retrieved %zd objects", objects.size());
				database->copyTable();
				cout<<" Database were copied "<<endl;	
			}
		}
		//
		database->cleanTable();
		cout<<"Deleted database done! "<<endl;
	}
	
	//Launch the database watchdog
	boost::thread thread(boost::bind(watchdog,&database));
	
	int i=0;

	ros::Rate r(2);
	while(ros::ok())
	{
		r.sleep();
		ros::spinOnce();
		//cada vez que corre vai ter de se atualizar o id que vai na mensagem para escrever na base de dados

		if(!database->isConnected())
		{
			delete database;
			
			//Connect to the database
			database = new Database("lars.mec.ua.pt", "5432","atlas", "atlas2013", "atlas_monitoring");
			if (!database->isConnected())
			{
				std::cerr << "Database failed to connect \n";
				continue;
			}else
			{
				std::cerr << "Database connected successfully \n";
				//check if database is empty, and if isn't make a copy
				std::vector< boost::shared_ptr<DatabaseStructure> > objects;
				
				if (!database->getList(objects))
				{
				  ROS_ERROR("Failed to get list of test objects");
				  continue;
				}
				
				if(objects.size()>0)
				{
				  cout<<" The database isn't empty "<<endl;
				  ROS_INFO("Retrieved %zd objects", objects.size());
				  database->copyTable();
				  cout<<" Database were copied "<<endl;  
				}
				
				database->cleanTable();
				
				cout<<"Deleted database done! "<<endl;
				if (!database->getList(objects))
				{
				  ROS_ERROR("Failed to get list of test objects");
				  continue;
				}
				ROS_INFO("Retrieved %zd objects", objects.size());
			}
		}
		
		//time send by the car to the db to compare with
		atlas_insert.atlas_time_.data()=currentDateTime();
		atlas_insert.id_.data()=i++;
		
		//Remove the line from the database, and add it (it only deletes if the line is repeated, this sould not be needed)
		database->deleteFromDatabase((database_interface::DBClass*)&atlas_insert);
		database->insertIntoDatabase((database_interface::DBClass*)&atlas_insert);
		
		database->touch();
	}

	//Join the thread
	thread.join();

	database->copyTable();
	cout<<"Backup database done! "<<endl;
	
	database->cleanTable();
	cout<<"Deleted database done! "<<endl;

	return 0;
}
