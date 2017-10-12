#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "mtt/TargetList.h"
#include "sensor_msgs/LaserScan.h"
#include "cmath"
#include <iostream>
#include <vector>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

using namespace std;

typedef enum
{
    PEDESTRIAN,
    UNKNOWN
} TypeClassification;

ros::Publisher marker_publisher;

/**
 * \brief Dynamic markers support class
 * 
 * This class allows to easily publish a variable number of markers without paying attention to the delete action requests.
 */
class Markers
{
    public:
        /**
         * \brief Update a internal marker
         * 
         * The updated marker will be marked for publishing, this marker may or may not already be present in the class.
         * 
         * \param marker a marker to update
         */
        void update(visualization_msgs::Marker& marker)
        {
            for(uint i=0;i<markers.size();++i)
                if(markers[i].ns==marker.ns && markers[i].id==marker.id)//Marker found
                {
                    markers.erase(markers.begin()+i);
                    break;
                }
                
            markers.push_back(marker);
        }
        
        /**
         * \brief Mark existing markers for deletion
         */
        void decrement(void)
        {
            for(uint i=0;i<markers.size();++i)
            {
                switch(markers[i].action)
                {
                    case visualization_msgs::Marker::ADD:
                        markers[i].action = visualization_msgs::Marker::DELETE;
                        break;
                    case visualization_msgs::Marker::DELETE:
                        markers[i].action = -1;
                        break;
                }
            }
        }

        
        /**
         * \brief Remove markers that should not be transmitted
         */
        void clean(void)
        {
            vector<visualization_msgs::Marker> new_markers;
            
            for(uint i=0;i<markers.size();++i)
                if(markers[i].action!=-1)
                    new_markers.push_back(markers[i]);
                
            markers=new_markers;
        }
        
        /**
         * \brief Obtain the list of outgoing markers
         * 
         * \return a vector of visualization_msgs::Marker
         */
        vector<visualization_msgs::Marker> getOutgoingMarkers(void)
        {
            vector<visualization_msgs::Marker> m_out(markers);
            return markers;
        }
    private:
        ///Internal storing vector of markers
        vector<visualization_msgs::Marker> markers;
};

/**
 * \brief Candidate validation
 * 
 * This class allows, potencial pedestrian images, to be analysed
 */

class Candidate
{
    public:
        
        typedef boost::shared_ptr<Candidate> Ptr;
        
        Candidate(double input_image, uint priority_, uint id_)
        {
            id = id_;
            image = input_image;
            priority = priority_;
            
            classification = UNKNOWN;
        }
        
        
        void classify()
        {
            t = boost::thread(boost::bind(&Candidate::do_classfication,this));
        }
        
        string getClassification()
        {
            switch(classification)
            {
                case PEDESTRIAN:
                    return "pedestrian";
                case UNKNOWN:
                    return "unknown";
            }
            
            return "unknown";
        }
    
        void do_classfication()
        {
            //Run the heady classifcation code from pedro
            int c = 0;
            cout<<"Processing image"<<endl;
            while(c<100)
            {
                c++;
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            }
            
            classification = PEDESTRIAN;
            
            cout<<"done"<<endl;
        }
    
        uint id;
        uint priority;
        double distance_to_target_from_center_bumper;
        double theta;
        double image;
        TypeClassification classification;
        boost::thread t;
    
};
        
Markers markers;

void  laserGather(const mtt::TargetList& msg)
{
    markers.decrement();
    
    vector<Candidate::Ptr> candidate_list;
    
	//trying to draw markers
    visualization_msgs::MarkerArray allmarkers;
		
//      cout << "Target message size: " << msg.Targets.size() << endl;
//      cout << msg.Targets[1] << endl;

// 	cout << "inicio" << endl;
	
	for(int i=0; i<msg.Targets.size();i++)
	{
        
			if (msg.Targets[i].size > 0.01/* && msg.Targets[i].size < 0.6*/ ) // && msg.Targets[i].pose.position.x >= 0 ) //&& msg.Targets[i].pose.position.y >= 0)
			{
                Candidate::Ptr candidate = Candidate::Ptr(new Candidate(0.0,i,i));
                
// 				cout << "target id" << msg.Targets[i].id << endl;
// 				cout << "target size:" << msg.Targets[i].size << endl;
// 				cout << "target position x:" << msg.Targets[i].pose.position.x << "    y:" << msg.Targets[i].pose.position.y << endl;
             
                candidate->distance_to_target_from_center_bumper = sqrt(msg.Targets[i].pose.position.x * msg.Targets[i].pose.position.x + 
                                    msg.Targets[i].pose.position.y * msg.Targets[i].pose.position.y);
                
// 				cout << "distancetotarget:" << candidate->distance_to_target_from_center_bumper  << endl;
                
                candidate->theta = acos( msg.Targets[i].pose.position.x / candidate->distance_to_target_from_center_bumper);
				
//                 cout << "theta:" << msg.Targets[i].velocity << endl;
                
				if(msg.Targets[i].velocity.linear.x > 0.01 /*candidate->distance_to_target_from_center_bumper < 20 && candidate->theta < (3.1415926/3)*/ )
				{
					visualization_msgs::Marker marker;
					
					marker.header.frame_id = "laser_1";
					marker.header.stamp = ros::Time();
					marker.ns = "my_namespace";
					marker.id = msg.Targets[i].id;
					marker.type = visualization_msgs::Marker::CUBE;
					marker.action = visualization_msgs::Marker::ADD;
					marker.pose.position.x = msg.Targets[i].pose.position.x;
					marker.pose.position.y = msg.Targets[i].pose.position.y;
					marker.pose.position.z = msg.Targets[i].pose.position.z;
					marker.pose.orientation.x = 0.0;
					marker.pose.orientation.y = 0.0;
					marker.pose.orientation.z = 0.0;
					marker.pose.orientation.w = 1.0;
					marker.scale.x = 0.1;
					marker.scale.y = msg.Targets[i].size * exp(-candidate->distance_to_target_from_center_bumper /100) * 2;
					marker.scale.z = exp(-(candidate->distance_to_target_from_center_bumper /100)*2);
					marker.color.a = 1.0;
					marker.color.r = 0.0;
					marker.color.g = 1.0;
					marker.color.b = 0.0;
					
					markers.update(marker);
                   candidate_list.push_back(candidate);

				}
				
                
			}
	}
	
	sort(candidate_list.begin(), candidate_list.end(),
         [&](const Candidate::Ptr c1,const Candidate::Ptr c2){ 
            return c1->distance_to_target_from_center_bumper > c2->distance_to_target_from_center_bumper; 
            });
	
	markers.clean();
	allmarkers.markers = markers.getOutgoingMarkers();
    
	marker_publisher.publish(allmarkers);
}

// - fazer a lista de peões (PCU)
// - actualizar o blog

// bool compareByPriority(const Candidate::Ptr& c1,const Candidate::Ptr& c2)
// {
//     return c1->priority > c2->priority; 
// }

int main (int argc, char **argv)
{
     ros::init(argc, argv, "laser_gather");
     
     ros::NodeHandle nh("~");  
	      
     marker_publisher = nh.advertise<visualization_msgs::MarkerArray>( "/ped_markers", 10);
     ros::Subscriber sub = nh.subscribe ("/targets", 1000, laserGather);

     ros::spin();
     
     return 0;
}

// void  test(const sensor_msgs::LaserScan& msg)
// {
//     cout << "msg:" << msg << endl;
// }
// 
// int main (int argc, char **argv)
// {
//      ros::init(argc, argv, "lascan");
//      
// //      cout << "msg:" << /*msg <<*/ endl;
//      
//      ros::NodeHandle nh("~");  
//           
//      ros::Subscriber sub = nh.subscribe ("/laser_1/scan", 1000, test);
// 
//      ros::spin();
//      
//      return 0;
// }