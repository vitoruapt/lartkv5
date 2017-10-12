/**
 * @file haptic_feedback.cpp
 * @brief Node that translates sensory information into a force vector to be rendered by the haptic joystick.
 *        Calculates the feedback force sent to the joystick based on the CoP location and on its distance
 *        to the support polygon boundaries. 
 *        Defines a C++ class responsible for subscribing to CoP and support polygon information, which then evaluates 
 *        if the CoP is within the support polygon or not. If so, an algorithm calculates the force to be rendered on the user.      
 * @version v1.0
 * @author João O. Barros
 * @date February 2015
 */


#include <humanoid_simulation/humanoid_simulation.h>

using namespace std;

int simulator_state(4);


geometry_msgs::Vector3 operator-(const geometry_msgs::Point p0, const geometry_msgs::Point p1)
{
    geometry_msgs::Vector3 v;
	  
    v.x = p0.x - p1.x;
    v.y = p0.y - p1.y;
    v.z = p0.z - p1.z;
     
    return v;
}


geometry_msgs::Vector3 operator*(const double s, const geometry_msgs::Vector3 v)
{
    geometry_msgs::Vector3 u;
	  
    u.x = s*v.x;
    u.y = s*v.y;
    u.z = s*v.z;
     
    return u;
}


geometry_msgs::Point operator+(const geometry_msgs::Point p, const geometry_msgs::Vector3 v)
{
    geometry_msgs::Point pb;
    
    pb.x = p.x + v.x;
    pb.y = p.y + v.y;
    pb.z = p.z + v.z;
    
    return pb;
}


geometry_msgs::Point FIR_cop(const geometry_msgs::Point cop_prev, const geometry_msgs::Point cop_new)
{
    geometry_msgs::Point cop_filtered;
    double beta = 0.2;
    
    cop_filtered.x = (1-beta)*cop_prev.x + beta*cop_new.x;
    cop_filtered.y = (1-beta)*cop_prev.y + beta*cop_new.y;
    
    return cop_filtered;
}


/* Float-point filter implemented "on the fly" to compute the final force value,
 * based on the current and the previous calculated values. */ 
geometry_msgs::Vector3 FIR_force(const geometry_msgs::Vector3 force_prev, const geometry_msgs::Vector3 force_new)
{
    geometry_msgs::Vector3 force_filtered;
    double beta = 0.4;
    
    force_filtered.x = (1-beta)*force_prev.x + beta*force_new.x;
    force_filtered.y = (1-beta)*force_prev.y + beta*force_new.y;
    
    return force_filtered;
}


class Feedback
{
    private:
        ros::NodeHandle nh_;
        
        ros::Subscriber simulation_state_sub;
        ros::Subscriber cop_global_position_sub;
        ros::Subscriber support_base_position_sub; 
        ros::Subscriber phua_position_sub;
        
        ros::Publisher cop_global_filtered_pub;
        ros::Publisher feedback_force_instability_pub;
        ros::Publisher feedback_force_stability_pub;
        ros::Publisher feedback_force_pub;
        ros::Publisher feedback_force_filtered_pub;
        
        geometry_msgs::PoseStamped phua_position;
        geometry_msgs::PointStamped cop_global;
        geometry_msgs::PolygonStamped support_base;
       
        geometry_msgs::PointStamped cop_filtered;
        geometry_msgs::Vector3Stamped force;
         
        double distance_min;
        double simulationTime;
    
        geometry_msgs::Vector3Stamped force_stability_prev, force_instability_prev;
        
        
    public:
        Feedback(ros::NodeHandle nh) : nh_(nh) 
        {
	  simulation_state_sub = nh_.subscribe("/vrep/info", 1000, &Feedback::simulationState, this);
	  
	  phua_position_sub = nh_.subscribe("/phua_position", 1000, &Feedback::phuaPosition, this);
	  cop_global_position_sub = nh_.subscribe("/cop_global_position", 1000, &Feedback::copPosition, this);
	  support_base_position_sub = nh_.subscribe("support_base_position", 1000, &Feedback::supportBaseDefinition, this);
	  
	  cop_global_filtered_pub = nh_.advertise<geometry_msgs::PointStamped>("/cop_global_filtered",1000);
	  feedback_force_stability_pub = nh_.advertise<geometry_msgs::Vector3Stamped>("/feedback_force_stability",1000);
	  feedback_force_instability_pub = nh_.advertise<geometry_msgs::Vector3Stamped>("/feedback_force_instability",1000);
	  feedback_force_pub = nh_.advertise<geometry_msgs::Vector3Stamped>("/feedback_force",1000);
	  feedback_force_filtered_pub = nh_.advertise<geometry_msgs::Vector3Stamped>("/feedback_force_filtered",1000);
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
	  phua_position.pose.position.x = msg.pose.position.x;
	  phua_position.pose.position.y = msg.pose.position.y;
	  phua_position.pose.position.z = msg.pose.position.z;
        }
        
        
        void copPosition(const geometry_msgs::PointStamped& msg)
        {
	  geometry_msgs::PointStamped cop_global_new;
	  
	  cop_global_new.point.x = msg.point.x*1000;
	  cop_global_new.point.y = msg.point.y*1000;
	  
	  /* Chooese here if CoP position is filtered. */
	  /*cop_global.point = FIR_cop(cop_global.point, cop_global_new.point);*/
	  cop_global.point = cop_global_new.point;

	  cop_global.header.stamp.sec = simulationTime*100000;      	   
	  cop_global_filtered_pub.publish(cop_global);    
        }
        
        
        /* Tests if CoP is within the support polygon. If so, the feedback force is calculated. */
        void supportBaseDefinition(const geometry_msgs::PolygonStamped& msg)
        {
	  support_base.polygon.points.clear();
	
	  for (uint i=0; i<msg.polygon.points.size(); i++)
	  {
	      geometry_msgs::Point32 p;
	      
	      p.x = msg.polygon.points[i].x*1000;
	      p.y = msg.polygon.points[i].y*1000;
	      p.z = msg.polygon.points[i].z*1000;
	      
	      support_base.polygon.points.push_back(p);
	  }
	  
	  if (wn_PnPoly(cop_global.point, support_base.polygon) != 0)
	  {
	      vector<double> distance;
	      
	      for (uint i=0; i<support_base.polygon.points.size()-1; i++)
	      {
		vector<geometry_msgs::Point> segment(2);
		
		segment[0].x = support_base.polygon.points[i].x;
		segment[0].y = support_base.polygon.points[i].y;
		
		segment[1].x = support_base.polygon.points[i+1].x;
		segment[1].y = support_base.polygon.points[i+1].y;
		
		distance.push_back(dist_PointSegment(cop_global.point, segment));
	      }
	      
	      distance_min = *min_element(distance.begin(), distance.end()); 
    
	      /*cout << "Minimum distance to boundary: " << distance_min << endl;*/
	      
 	      calculateFeedback(cop_global, distance_min);     
	  }
	  else
	      ROS_ERROR("Center of Pressure is beyond support polygon boundary. System unstable!");
        }
        
        
        /* Implementation of the force rendering algorithm that compares the CoP position with the real support polygon size
         * and establishes a metric of stability/instability. The force vector synthesized is determined by a function that expresses 
         * simultaneously the CoP approximation to the robot's feet edges (F2), and its separation from the most stable position (F1). */ 
        void calculateFeedback(geometry_msgs::PointStamped cop, double distance)
        {     
	  geometry_msgs::Vector3Stamped force_stability, force_instability, force_new;
	  
	  double cop_s = sqrt(cop.point.x*cop.point.x + cop.point.y*cop.point.y); 
	  
	  /*cout << "Minimum distance from origin: " << cop_s << endl;*/
	  
	  double a1 = 0.35, b1 = 40, c1=0.2;
	  double a2 = 0.35, b2 = 16.89733, c2=0.2;

	  double force_f1, force_f2;
	  
	  if (cop_s < 5)
	      force_f1 = 0;
	  else if (cop_s >= 5 && cop_s < 28.4490)
	      force_f1 = (0.025*cop_s - 0.125);
	  else if (cop_s >= 28.4490 && cop_s < 40)
	      force_f1 = (1/(c1*(1+exp(-a1*(cop_s - b1)))) + 0.5);
	  else if (cop_s >= 40)
	      force_f1 = 3;
	      
	  if (distance > 51.8972)
	      force_f2 = 0;
	  else if (distance <= 51.8972 && distance > 28.4490)
	      force_f2 = (-(0.025)*(distance) + (1.29743));
	  else if (distance <= 28.4490 && distance > 16.89733)
	      force_f2 = (1/(c2*(1+exp(a2*((distance) - b2)))) + 0.5);
	  else if (distance <= 16.89733)
	      force_f2 = 3;    
	      
	  double r1 = (1*force_f1)/cop_s;
	  double r2 = (1*force_f2)/cop_s;
	  
	  force_stability.vector.x = (cop.point.x * r1);
	  force_stability.vector.y = cop.point.y * r1;
	  force_stability.vector.z = 0;
	  
	  force_instability.vector.x = cop.point.x * r2;
	  force_instability.vector.y = cop.point.y * r2;
	  force_instability.vector.z = 0;
	  
	  /*if (force_stability.vector.x < 0 && phua_position.pose.position.x < 0)
	      force_stability.vector.x = force_stability.vector.x;
	  else if (force_stability.vector.x < 0 && phua_position.pose.position.x > 0)
	      force_stability.vector.x = -force_stability.vector.x;
	  else if (force_stability.vector.x > 0 && phua_position.pose.position.x > 0)
	      force_stability.vector.x = force_stability.vector.x;
	  else if (force_stability.vector.x > 0 && phua_position.pose.position.x < 0)
	      force_stability.vector.x = -force_stability.vector.x;
	  
	  if (force_stability.vector.y < 0 && phua_position.pose.position.y < 0)
	      force_stability.vector.y = force_stability.vector.y;
	  else if (force_stability.vector.y < 0 && phua_position.pose.position.y > 0)
	      force_stability.vector.y = -force_stability.vector.y;
	  else if (force_stability.vector.y > 0 && phua_position.pose.position.y > 0)
	      force_stability.vector.y = force_stability.vector.y;
	  else if (force_stability.vector.y > 0 && phua_position.pose.position.y < 0)
	      force_stability.vector.y = -force_stability.vector.y;*/
	  
	  force_stability.header.stamp.sec = simulationTime*100000;      	   
	  feedback_force_stability_pub.publish(force_stability);
	  
	  force_instability.header.stamp.sec = simulationTime*100000;      	   
	  feedback_force_instability_pub.publish(force_instability); 
	  
	  double vm_stability_x, vm_stability_y;
	  double vm_instability_x, vm_instability_y;
	  
	  if (force_stability.vector.x - force_stability_prev.vector.x != 0)
	      vm_stability_x = (force_stability.vector.x - force_stability_prev.vector.x) / 0.05;
	  else
	      vm_stability_x = (force_stability.vector.x) / 0.05;
	      
	  if (force_instability.vector.x - force_instability_prev.vector.x != 0)
	      vm_instability_x = (force_instability.vector.x - force_instability_prev.vector.x) / 0.05;
	  else
	      vm_instability_x = (force_instability.vector.x) / 0.05;
	  
	  if (force_stability.vector.y - force_stability_prev.vector.y != 0)
	      vm_stability_y = (force_stability.vector.y - force_stability_prev.vector.y) / 0.05;
	  else
	      vm_stability_y = (force_stability.vector.y) / 0.05;
	  
	  if (force_instability.vector.y - force_instability_prev.vector.y != 0)
	      vm_instability_y = (force_instability.vector.y - force_instability_prev.vector.y) / 0.05;
	  else
	      vm_instability_y = (force_instability.vector.y) / 0.05;
	 
	  double rvm_x, rvm_y;
	  
	  rvm_x = abs(vm_instability_x / vm_stability_x); 
	  rvm_y = abs(vm_instability_y / vm_stability_y); 
	  
	  force_new.vector.x = (rvm_x/(rvm_x+1)) * force_instability.vector.x + (1/(rvm_x+1)) * force_stability.vector.x; 
	  force_new.vector.y = (rvm_y/(rvm_y+1)) * force_instability.vector.y + (1/(rvm_y+1)) * force_stability.vector.y; 
	  force_new.vector.z = 0;
	  
	  if (force_instability.vector.x == 0 || force_stability.vector.x == 0)
	     force_new.vector.x = force_instability.vector.x + force_stability.vector.x;
	     
	  if (force_instability.vector.y == 0 || force_stability.vector.y == 0)
	     force_new.vector.y = force_instability.vector.y + force_stability.vector.y; 
	     
	  force_new.header.stamp.sec = simulationTime*100000;      	   
	  feedback_force_pub.publish(force_new); 
	  
	  /* Chooese here if force is filtered */
 	  force.vector = FIR_force(force.vector, force_new.vector);
	  /* force.vector = force_new.vector; */
	  
	  cout << "Force vector filtered = " << '(' << force.vector.x << ", " << force.vector.y << ')' << endl;
	  cout << "--------------------------------------" << endl;
	  
	  force.header.stamp.sec = simulationTime*100000;      	   
	  feedback_force_filtered_pub.publish(force);   
	  
	  force_stability_prev = force_stability;
	  force_instability_prev = force_instability;
        }  
        
        
        /* Tests if a point is Left|On|Right of an infinite line. */
        int isLeft(geometry_msgs::Point32 p0, geometry_msgs::Point32 p1, geometry_msgs::Point p2)  
        {
	  return ((p1.x - p0.x)*(p2.y - p0.y) - (p2.x - p0.x)*(p1.y - p0.y));
        }
	      
	  
        /* Winding number test for a point in a polygon. */
        int wn_PnPoly(geometry_msgs::Point p, geometry_msgs::Polygon poly)
        {
	  int n = poly.points.size()-1; /* polygon size */
	 
	  int wn = 0; /* the winding number counter */
	  
	  /* Loop through all edges of the polygon. */
	  for (int i=0; i<n; i++) /* edge from poly[i] to poly[i+1] */
	  {
	      if (poly.points[i].y <= p.y)
	      {
		if (poly.points[i+1].y > p.y)
		{
		    if (isLeft(poly.points[i], poly.points[i+1], p) > 0) 
		        ++wn;
		}
	      }
	      else
	      {
		if (poly.points[i+1].y <= p.y)
		{
		    if (isLeft(poly.points[i], poly.points[i+1], p) < 0)
		        --wn;
		}
	      }
	  }
	  
	  return wn;
        }
        
           
        double dot(geometry_msgs::Vector3 u, geometry_msgs::Vector3 v)
        {
	  return (u.x*v.x + u.y*v.y + u.z*v.z);
        }
        
        
        double norm(geometry_msgs::Vector3 v)     
        {
	  return (sqrt(dot(v,v)));
        }
	     
	     
        double d(geometry_msgs::Point p0, geometry_msgs::Point p1)
        {
	  return (norm(p0-p1)); 
        }
		    
	
        /* Calculates the distance from a 2D-point to a line segment. */  
        double dist_PointSegment(geometry_msgs::Point p, vector<geometry_msgs::Point> seg_point)
        {
	  geometry_msgs::Vector3 v = seg_point[1] - seg_point[0];
	 
	  geometry_msgs::Vector3 w = p - seg_point[0];

	  double c1 = dot(w,v); 
	  if (c1 <= 0)
	      return d(p, seg_point[0]);
	  
	  double c2 = dot(v,v);
	  if (c2 <= c1)
	      return d(p, seg_point[1]);
	  
	  double b = c1/c2;
	  geometry_msgs::Point pb = seg_point[0] + b*v;
	  return d(p, pb);
        }
         
};


int main(int argc, char **argv)
{   
    ros::init(argc, argv, "haptic_feedback");
    ros::NodeHandle n;
    
    Feedback phantom(n);
    
    ros::spin();
    
    return 0;
}
