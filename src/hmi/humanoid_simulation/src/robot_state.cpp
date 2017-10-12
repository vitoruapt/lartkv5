/**
 * @file robot_state.cpp
 * @brief Node in charge of receiving data from the V-REP simulation, monitoring robot and simulation states.
 *        This module basically defines a C++ class responsible for mmonitoring and register the main variables
 *        involved in the robot simulation.   
 *        Receives the sensor positions and the force values measured by them, and calculates the CoP. 
 *        Also defines the robot's ground base support, when one or both feet are in contact with the ground.
 *        These calculations are performed relatively to the robot coordinate frame, making them independent
 *        of the model's position in the V-REP scene.
 *        Inertial measurement units data is also received.  
 * @version v1.1
 * @author João O. Barros
 * @date February 2015
 */


#include <humanoid_simulation/humanoid_simulation.h>

using namespace std;

double realTime_start;
int simulator_state(4);


class Humanoid
{
    private:
        ros::NodeHandle nh_;
        
        ros::Subscriber simulation_state_sub;
        ros::Subscriber phua_pose_sub;
        ros::Subscriber left_foot_pose_sub;
        ros::Subscriber right_foot_pose_sub;
        ros::Subscriber sensors_pose_sub; 
        ros::Subscriber sensors_data_sub; 
        ros::Subscriber dummies_pose_sub;
        ros::Subscriber joints_state_sub;
        ros::Subscriber imu1_data_sub;
        ros::Subscriber imu2_data_sub;
        ros::Subscriber imu3_data_sub;
        ros::Subscriber imu4_data_sub;
        ros::Subscriber imu5_data_sub;
        ros::Subscriber imu6_data_sub;
        ros::Subscriber imu7_data_sub;
        ros::Subscriber imu8_data_sub;
        ros::Subscriber imu9_data_sub;
        
        ros::Publisher cop_global_position_pub;
        ros::Publisher support_base_position_pub;
        ros::Publisher phua_position_pub;
        ros::Publisher reaction_force_pub;
        ros::Publisher joints_state_pub;
        ros::Publisher imu_network_data_pub;
      
        vector<geometry_msgs::PoseStamped> sensor_pos;
        vector<geometry_msgs::Vector3Stamped> sensor_value;
        vector<geometry_msgs::PoseStamped> dummy_pos;
     
        geometry_msgs::Pose left_foot_pose;
        geometry_msgs::Pose right_foot_pose;
        geometry_msgs::Pose feet_pose;
        
        tf::TransformBroadcaster WP_broadcaster, WLf_broadcaster, WRf_broadcaster, WF_broadcaster;
        tf::TransformListener WP_listener, WLf_listener, WRf_listener, WF_listener;
        tf::Transform WP_T, WLf_T, WRf_T, WF_T;
       
        string referential;
        
        double simulationTime;
        double realTime;
        
        geometry_msgs::PointStamped cop_global;
        geometry_msgs::PoseStamped phua_pos;     
       
        vrep_common::ImuData imu_network;
        sensor_msgs::Imu imu_data_prev;
       
        
    public:
        Humanoid(ros::NodeHandle nh) : nh_(nh) // sensor_abs.pose(0), dummy_abs.pose(0), sensor_value.vector(0), cop_global.point(0)
        {
	  simulation_state_sub = nh_.subscribe("/vrep/info", 1000, &Humanoid::simulationState, this);
	    
	  phua_pose_sub = nh_.subscribe("/vrep/phua_pose", 1000, &Humanoid::phuaPosition, this);
	  left_foot_pose_sub = nh_.subscribe("/vrep/left_foot_pose", 1000, &Humanoid::leftFootPosition, this);
	  right_foot_pose_sub = nh_.subscribe("/vrep/right_foot_pose", 1000, &Humanoid::rightFootPosition, this);
	  sensors_data_sub = nh_.subscribe("/vrep/sensors_data", 1000, &Humanoid::sensorsUpdate, this);
	  sensors_pose_sub = nh_.subscribe("/vrep/sensors_pose", 1000, &Humanoid::sensorsPosition, this);
	  dummies_pose_sub = nh_.subscribe("/vrep/dummies_pose", 1000, &Humanoid::dummiesPosition, this);
	  joints_state_sub = nh_.subscribe("/vrep/joints_state", 1000, &Humanoid::jointsUpdate, this);
	    
	  imu1_data_sub = nh.subscribe("/vrep/imu1_data", 1000, &Humanoid::imusUpdate, this);
	  imu2_data_sub = nh.subscribe("/vrep/imu2_data", 1000, &Humanoid::imusUpdate, this);
	  imu3_data_sub = nh.subscribe("/vrep/imu3_data", 1000, &Humanoid::imusUpdate, this);
	  imu4_data_sub = nh.subscribe("/vrep/imu4_data", 1000, &Humanoid::imusUpdate, this);
	  imu5_data_sub = nh.subscribe("/vrep/imu5_data", 1000, &Humanoid::imusUpdate, this);
	  imu6_data_sub = nh.subscribe("/vrep/imu6_data", 1000, &Humanoid::imusUpdate, this);
	  imu7_data_sub = nh.subscribe("/vrep/imu7_data", 1000, &Humanoid::imusUpdate, this);
	  imu8_data_sub = nh.subscribe("/vrep/imu8_data", 1000, &Humanoid::imusUpdate, this);
	  imu9_data_sub = nh.subscribe("/vrep/imu9_data", 1000, &Humanoid::imusUpdate, this);
	  
	  cop_global_position_pub = nh_.advertise<geometry_msgs::PointStamped>("/cop_global_position", 1000);
	  support_base_position_pub = nh_.advertise<geometry_msgs::PolygonStamped>("/support_base_position", 1000);
	  phua_position_pub = nh_.advertise<geometry_msgs::PoseStamped>("/phua_position", 1000);
	  reaction_force_pub = nh_.advertise<geometry_msgs::Vector3Stamped>("/reaction_force", 1000);
	  joints_state_pub = nh_.advertise<sensor_msgs::JointState>("/phua_joints_state", 1000);
	  imu_network_data_pub = nh_.advertise<vrep_common::ImuData>("/imu_network_data", 1000);
        }
        
        
        void simulationState(const vrep_common::VrepInfo& msg)
        {
	  if (simulator_state == 5 && msg.simulatorState.data == 4) /* Simulation stopped. */
	      ros::shutdown();
	    
	  simulator_state = msg.simulatorState.data;
	  simulationTime = msg.simulationTime.data;
        }
        
        
        void phuaPosition(const geometry_msgs::PoseStamped& msg)
        {
	  geometry_msgs::PoseStamped phua_abs;
		
	  phua_abs.header.frame_id = "World";
	  phua_abs.header.stamp = ros::Time();
		
	  phua_abs.pose.position.x = msg.pose.position.x;
	  phua_abs.pose.position.y = msg.pose.position.y;
	  phua_abs.pose.position.z = msg.pose.position.z;
	  phua_abs.pose.orientation.x = msg.pose.orientation.x;
	  phua_abs.pose.orientation.y = msg.pose.orientation.y;
	  phua_abs.pose.orientation.z = msg.pose.orientation.z;
	  phua_abs.pose.orientation.w = msg.pose.orientation.w;
		
	  geometry_msgs::PoseStamped phua_local;
		
	  try
	  {	
	      WP_listener.transformPose(referential, phua_abs, phua_local);
	        
	      /*ROS_INFO("World: (%.2f, %.2f. %.2f) -----> PHUA: (%.2f, %.2f, %.2f) at time %.2f", 
  		   phua_abs.pose.position.x, phua_abs.pose.position.y, phua_abs.pose.position.z,
  		   phua_local.pose.position.x, phua_local.pose.position.y, phua_local.pose.position.z, phua_local.header.stamp.toSec());*/
	  }
	  catch(tf::TransformException& ex)
	  {
	      ROS_ERROR("Received an exception trying to transform PHUA from \"%s\" to \"%s\": %s", phua_abs.header.frame_id.c_str(), referential.c_str(), ex.what());
	  }
	    
	  phua_local.header.stamp.sec = simulationTime*100000;
	  phua_position_pub.publish(phua_local);      	   
        }
        
        
        /* Publishes the transformation from world to left foot frame. */
        void leftFootPosition(const geometry_msgs::PoseStamped& msg)
        {
	  left_foot_pose.position.x = msg.pose.position.x;
	  left_foot_pose.position.y = msg.pose.position.y;
	  left_foot_pose.position.z = 0; //msg.pose.position.z;
	  left_foot_pose.orientation.x = msg.pose.orientation.x;
	  left_foot_pose.orientation.y = msg.pose.orientation.y;
	  left_foot_pose.orientation.z = msg.pose.orientation.z;
	  left_foot_pose.orientation.w = msg.pose.orientation.w;

	  WLf_T.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
	  WLf_T.setRotation(tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w));

	  WLf_broadcaster.sendTransform(tf::StampedTransform(WLf_T, ros::Time::now(), "World", "LeftFoot"));
	    
	  feetPosition(left_foot_pose, right_foot_pose);
        }
        
        
        /* Publishes the transformation from world to right foot frame. */
        void rightFootPosition(const geometry_msgs::PoseStamped& msg)
        {
	  right_foot_pose.position.x = msg.pose.position.x;
	  right_foot_pose.position.y = msg.pose.position.y;
	  right_foot_pose.position.z = 0; //msg.pose.position.z;
	  right_foot_pose.orientation.x = msg.pose.orientation.x;
	  right_foot_pose.orientation.y = msg.pose.orientation.y;
	  right_foot_pose.orientation.z = msg.pose.orientation.z;
	  right_foot_pose.orientation.w = msg.pose.orientation.w;
	    
	  WRf_T.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
	  WRf_T.setRotation(tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w));

	  WRf_broadcaster.sendTransform(tf::StampedTransform(WRf_T, ros::Time::now(), "World", "RightFoot"));
	    
	  feetPosition(left_foot_pose, right_foot_pose);
        }
        
        
        /* Publishes the transformation from world to feet frame. */
        void feetPosition(geometry_msgs::Pose& left_foot_pose, geometry_msgs::Pose& right_foot_pose)
        {
	  feet_pose.position.x = (left_foot_pose.position.x + right_foot_pose.position.x)/2;
	  feet_pose.position.y = (left_foot_pose.position.y + right_foot_pose.position.y)/2;
	  feet_pose.position.z = (left_foot_pose.position.z + right_foot_pose.position.z)/2;
	  feet_pose.orientation.x = (left_foot_pose.orientation.x + right_foot_pose.orientation.x)/2;
	  feet_pose.orientation.y = (left_foot_pose.orientation.y + right_foot_pose.orientation.y)/2;
	  feet_pose.orientation.z = (left_foot_pose.orientation.z + right_foot_pose.orientation.z)/2;
	  feet_pose.orientation.w = (left_foot_pose.orientation.w + right_foot_pose.orientation.w)/2;
	    
	  WF_T.setOrigin(tf::Vector3(feet_pose.position.x, feet_pose.position.y, feet_pose.position.z));
	  WF_T.setRotation(tf::Quaternion(feet_pose.orientation.x, feet_pose.orientation.y, feet_pose.orientation.z, feet_pose.orientation.w));

	  WF_broadcaster.sendTransform(tf::StampedTransform(WF_T, ros::Time::now(), "World", "Feet"));
        }
        
            
        void sensorsUpdate(const vrep_common::ObjectGroupData& msg)
        {
	  /* Retrieves force sensor data (in intData (1 values): force sensor state; in floatData (6 values): force (fx,fy,fz) and torque (tx,ty,tz)). */
	    
	  /* Force in z direction for each sensor. */
	  sensor_value.clear();
	    
	  for (uint i=0; i<8; i++)
	  {
	      geometry_msgs::Vector3Stamped sensor;
		
	      sensor.vector.z = msg.floatData.data[(i*6)+2];
	  
	      sensor_value.push_back(sensor);
	  }
	  
	  /* Feet sensor values scheme */
	  /*cout << sensor_value[1].vector.z << "  ";                                    // Left sensor 2 value
	  cout << sensor_value[0].vector.z << "       ";                               // Left sensor 1 value
	  cout << sensor_value[4].vector.z << "  ";                                    // Right sensor 1 value
  	  cout << sensor_value[5].vector.z << '\n' << '\n' << '\n';                    // Right sensor 2 value
  	  cout << sensor_value[2].vector.z << "  ";                                    // Left sensor 3 value
  	  cout << sensor_value[3].vector.z << "       ";                               // Left sensor 4 value
  	  cout << sensor_value[7].vector.z << "  ";                                    // Right sensor 4 value
  	  cout << sensor_value[6].vector.z << endl;*/                                    // Right sensor 3 value
  
	  /* Choose referential based on sensors state. */  
	  int left_sensors_state, right_sensors_state;    

	  left_sensors_state = msg.intData.data[0] + msg.intData.data[1] + msg.intData.data[2] + msg.intData.data[3];
	  right_sensors_state = msg.intData.data[4] + msg.intData.data[5] + msg.intData.data[6] + msg.intData.data[7];
	    
	  if (left_sensors_state == 4 && right_sensors_state != 4)
	      referential = "LeftFoot";
	  else if (right_sensors_state == 4 && left_sensors_state != 4)
	      referential = "RightFoot";
	  else if (left_sensors_state == 4 && right_sensors_state == 4)
	      referential = "Feet"; 
	  else
	      ROS_ERROR("Received an exception trying to choose referential. Unknown state!");
        }
        
        
        void normalize(geometry_msgs::Quaternion& quaternion)
        {
	  double norm = sqrt(quaternion.x*quaternion.x + quaternion.y*quaternion.y + quaternion.z*quaternion.z);
	  quaternion.x/=norm;
	  quaternion.y/=norm;
	  quaternion.z/=norm;
        }
        
        
        /* Transforms sensor absolute coordinates to foot/feet frame. */
        void sensorsPosition(const vrep_common::ObjectGroupData& msg)
        {   
	  sensor_pos.clear(); 
	    
	  for (int i=0; i<8; i++)
	  {
	      geometry_msgs::PoseStamped sensor_abs;
		
	      sensor_abs.header.frame_id = "World";
	      sensor_abs.header.stamp = ros::Time();
		
	      sensor_abs.pose.position.x = msg.floatData.data[i*7];
	      sensor_abs.pose.position.y = msg.floatData.data[(i*7)+1];
	      sensor_abs.pose.position.z = msg.floatData.data[(i*7)+2];
	      sensor_abs.pose.orientation.x = msg.floatData.data[(i*7)+3];
	      sensor_abs.pose.orientation.y = msg.floatData.data[(i*7)+4];
	      sensor_abs.pose.orientation.z = msg.floatData.data[(i*7)+5];
	      sensor_abs.pose.orientation.w = msg.floatData.data[(i*7)+6];
		  
	      normalize(sensor_abs.pose.orientation);
	    
	      geometry_msgs::PoseStamped sensor_local;
		
	      try
	      {	
		WP_listener.transformPose(referential, sensor_abs, sensor_local);
		  
		/*ROS_INFO("World: (%.2f, %.2f. %.2f) -----> PHUA: (%.2f, %.2f, %.2f) at time %.2f", 
  		       sensor_abs.pose.position.x, sensor_abs.pose.position.y, sensor_abs.pose.position.z,
  		       sensor_local.pose.position.x, sensor_local.pose.position.y, sensor_local.pose.position.z, sensor_local.header.stamp.toSec());*/
		
	      }
	      catch(tf::TransformException& ex)
	      {
		ROS_ERROR("Received an exception trying to transform a sensor from \"%s\" to \"%s\": %s", sensor_abs.header.frame_id.c_str(), referential.c_str(), ex.what());
		
	      }
		    
	      sensor_pos.push_back(sensor_local);
	      
	  } 
	  
  	  /*cout << "Sensor local pos: (" << sensor_pos[0].pose.position.x << ", " << sensor_pos[0].pose.position.y << ", " << sensor_pos[0].pose.position.z << ")" << endl;*/

	  calculateGlobalCOP(sensor_pos, sensor_value);
        }
        
        
        /* Calculates the two-dimensional point location of CoP. */
        void calculateGlobalCOP(vector<geometry_msgs::PoseStamped>& sensor_pos, vector<geometry_msgs::Vector3Stamped>& sensor_value)
        {  
	  double left_resultant_force, right_resultant_force, resultant_force;
	    
	  /* Calculate resultant_force for each foot. */
	  for (uint i=0; i<sensor_value.size()/2; i++)
	  {
	      left_resultant_force += sensor_value[i].vector.z;
	      right_resultant_force += sensor_value[i+4].vector.z;
	  }
	    
	  /* Calculate resultant_force for both foot. */
	  resultant_force = left_resultant_force + right_resultant_force;
	    
	  geometry_msgs::Vector3Stamped reaction_force;
	  reaction_force.vector.z = resultant_force;
	  reaction_force.header.stamp.sec = simulationTime*100000;
	    
	  /* CoP position: */
	  for (uint i=0; i<sensor_pos.size(); i++)
	  {
	      cop_global.point.x += (sensor_pos[i].pose.position.x * sensor_value[i].vector.z);
	      cop_global.point.y += (sensor_pos[i].pose.position.y * sensor_value[i].vector.z);
	  }
	    
	  cop_global.point.x = cop_global.point.x / resultant_force;
	  cop_global.point.y = cop_global.point.y / resultant_force;
	  cop_global.header.stamp.sec = simulationTime*100000;
	    
	  cout.precision(5);
	  cout << "Global CoP = " << '(' << fixed << cop_global.point.x << ", " << fixed << cop_global.point.y << ')' << endl; 
	  cout << "--------------------------------------" << endl;
	    
	  cop_global_position_pub.publish(cop_global);
	  reaction_force_pub.publish(reaction_force);
        }
        
        
        /* Transforms dummy absolute coordinates to foot/feet frame. */
        void dummiesPosition(const vrep_common::ObjectGroupData& msg)
        {   
	  dummy_pos.clear();
	    
	  for (int i=0; i<8; i++)
	  {
	      geometry_msgs::PoseStamped dummy_abs;
		
	      dummy_abs.header.frame_id = "World";
	      dummy_abs.header.stamp = ros::Time();
		
	      dummy_abs.pose.position.x = msg.floatData.data[(i*7)+7];
	      dummy_abs.pose.position.y = msg.floatData.data[(i*7)+8];
	      dummy_abs.pose.position.z = msg.floatData.data[(i*7)+9];
	      dummy_abs.pose.orientation.x = msg.floatData.data[(i*7)+10];
	      dummy_abs.pose.orientation.y = msg.floatData.data[(i*7)+11];
	      dummy_abs.pose.orientation.z = msg.floatData.data[(i*7)+12];
	      dummy_abs.pose.orientation.w = msg.floatData.data[(i*7)+13];
		
	      normalize(dummy_abs.pose.orientation);
		
	      geometry_msgs::PoseStamped dummy_local;
		
	      try
	      {	
		WP_listener.transformPose(referential, dummy_abs, dummy_local);
		  
  		/*ROS_INFO("World: (%.2f, %.2f. %.2f) -----> PHUA: (%.2f, %.2f, %.2f) at time %.2f", 
  		         dummy_abs.pose.position.x, dummy_abs.pose.position.y, dummy_abs.pose.position.z,
  		         dummy_local.pose.position.x, dummy_local.pose.position.y, dummy_local.pose.position.z, dummy_local.header.stamp.toSec());*/
		
	      }
	      catch(tf::TransformException& ex)
	      {
		ROS_ERROR("Received an exception trying to transform a dummy from \"%s\" to \"%s\": %s", dummy_abs.header.frame_id.c_str(), referential.c_str(), ex.what());
		
	      }
		
	      dummy_pos.push_back(dummy_local);
	      
	  }
	    
  	  /*cout << "Dummy local pos: (" << dummy_pos[0].pose.position.x << ", " << dummy_pos[0].pose.position.y << ", " << dummy_pos[0].pose.position.z << ")" << endl;*/
	    
	  convexHull(dummy_pos);
        }

        
        static bool compare_pos(const geometry_msgs::PoseStamped p1, const geometry_msgs::PoseStamped p2)
        {
	  return p1.pose.position.x < p2.pose.position.x || (p1.pose.position.x == p2.pose.position.x && p1.pose.position.y < p2.pose.position.y);
        }
        
        
        /* 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
         *Returns a positive value, if OAB makes a counter-clockwise turn, negative for clockwise turn, and zero if the points are collinear. */
        double cross(const geometry_msgs::PoseStamped& O, const geometry_msgs::PoseStamped& A, const geometry_msgs::PoseStamped& B)
        {
	  return (A.pose.position.x - O.pose.position.x) * (B.pose.position.y - O.pose.position.y) - (A.pose.position.y - O.pose.position.y) * (B.pose.position.x - O.pose.position.x);
        }
        
        
        /* Returns a list of points on the convex hull in counter-clockwise order.
         * Note: the last point in the returned list is the same as the first one. */
        void convexHull(vector<geometry_msgs::PoseStamped> dummy_pos)
        {
	  int n = dummy_pos.size(), k = 0;
	  vector<geometry_msgs::PoseStamped> convex_hull_point(2*n); //H
	    
	  /* Sort points lexicographically. */
	  sort(dummy_pos.begin(), dummy_pos.end(), compare_pos); //P
	    
	  /* Build lower hull. */
	  for (int i = 0; i < n; ++i) 
	  {
	      while (k >= 2 && cross(convex_hull_point[k-2], convex_hull_point[k-1], dummy_pos[i]) <= 0) k--;
	      convex_hull_point[k++] = dummy_pos[i];
	  }
	    
	  /* Build upper hull. */
	  for (int i = n-2, t = k+1; i >= 0; i--) 
	  {
	      while (k >= t && cross(convex_hull_point[k-2], convex_hull_point[k-1], dummy_pos[i]) <= 0) k--;
	      convex_hull_point[k++] = dummy_pos[i];
	  }
	    
	  convex_hull_point.resize(k);
	  /* End */
	    
  	  /*cout << "Convex hull size: " << convex_hull_point.size() << endl;*/
	    
	  geometry_msgs::PolygonStamped support_base;
	    
	  for (uint i=0; i<convex_hull_point.size(); i++)
	  {
	      geometry_msgs::Point32 p;
		
	      p.x = convex_hull_point[i].pose.position.x;
	      p.y = convex_hull_point[i].pose.position.y;
	      p.z = 0;
		
	      support_base.header.stamp.sec = simulationTime*100000;
	      support_base.polygon.points.push_back(p);
 	  }
	  
	  
	  /* Algorithm test */
	  /*geometry_msgs::Point32 p;
	  p.x=-0.089*1000;
	  p.y=0.093*1000;
	  support_base.polygon.points.push_back(p);
	  p.x=0.029*1000;
	  p.y=0.093*1000;
	  support_base.polygon.points.push_back(p);
	  p.x=0.089*1000;
	  p.y=0.093*1000;
	  support_base.polygon.points.push_back(p);
	  p.x=0.089*1000;
	  p.y=-0.056*1000;
	  support_base.polygon.points.push_back(p);
	  p.x=-0.029*1000;
	  p.y=-0.056*1000;
	  support_base.polygon.points.push_back(p);
	  p.x=-0.089*1000;
	  p.y=-0.056*1000;
	  support_base.polygon.points.push_back(p);
	  p.x=-0.089*1000;
	  p.y=0.093*1000;
	  support_base.polygon.points.push_back(p);*/
	  
	  
	  support_base_position_pub.publish(support_base);
        }
        
        
        void jointsUpdate(const sensor_msgs::JointState& msg)
        {
	  sensor_msgs::JointState state;
	    
	  for (uint i=0; i<14; i++)
	  {     
	      state.position.push_back(msg.position[i]);
	      state.velocity.push_back(msg.velocity[i]);
	      state.effort.push_back(msg.effort[i]);
	  }
	    
	  state.header.stamp.sec = simulationTime*100000;
	    
	  joints_state_pub.publish(state);
        }
        
        
        struct compare_id
        {
	  inline bool operator() (const sensor_msgs::Imu& unit1, const sensor_msgs::Imu& unit2)
	  {
	      return (atoi(unit1.header.frame_id.c_str()) < atoi(unit2.header.frame_id.c_str())); 
	  }
        };
        
        
        /* Sorts the IMU data according to the numbering in V-REP (9 units in the network). */
        void imusUpdate(const sensor_msgs::Imu& msg)
        {
	  sensor_msgs::Imu imu_data;
	  
	  imu_data.header.stamp.sec = simulationTime*100000;
	  imu_data.header.frame_id = msg.header.frame_id;
	  imu_data.angular_velocity = msg.angular_velocity;
	  imu_data.linear_acceleration = msg.linear_acceleration;  
	  
	  if (imu_data.header.stamp.sec != imu_data_prev.header.stamp.sec)
	  {
	      imu_network.unitData.clear();
	      imu_network.unitData.push_back(imu_data);	      
	  }
	  else
	      imu_network.unitData.push_back(imu_data);
	  
	  if (imu_network.unitData.size() == 9)
	  {
	      sort(imu_network.unitData.begin(), imu_network.unitData.end(), compare_id());
	      imu_network_data_pub.publish(imu_network);
	  }
	   
	  imu_data_prev = imu_data;
        }
};


int main(int argc, char **argv)
{   
    ros::init(argc, argv, "robot_state");
    ros::NodeHandle nh;
    
    Humanoid phua(nh);
    
    ros::spin();
    
    return 0;
}
