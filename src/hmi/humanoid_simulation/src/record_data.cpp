/**
 * @file record_data.cpp
 * @brief Node that subscribes to different topics recording general information about a certain teleoperation task. 
 *        Its function is to record the data transmitted among the different ROS nodes, including sensory information 
 *        and control commands that can later be used for visualization and analyses.
 *        The data is saved in several dedicated .csv files.
 * @version v1.0
 * @author João O. Barros
 * @date February 2015
 */


#include <humanoid_simulation/humanoid_simulation.h>
#include <sys/stat.h>

using namespace std;

string dir_path;
string plane, amplitude, frequency, user;

int simulator_state(4);
double simulationTime (0);


template <typename T> string NumberToString ( T Number )
{
    ostringstream ss;
    ss << setprecision(1) << Number;
    return ss.str();
}


void simulationState(const vrep_common::VrepInfo& msg)
{
    if (simulator_state == 5 && msg.simulatorState.data == 4) /* Simulation stopped. */
	      ros::shutdown();
	  
    simulator_state = msg.simulatorState.data;
    simulationTime = msg.simulationTime.data;
    
    cout << "Simulations time [s]: " << simulationTime << endl;
    cout << "--------------------------------------" << endl;
}


void recordCOP(const geometry_msgs::PointStamped& msg)
{
    /*string location ("/home/phua3-lar/Experiments/Results/");
    string path = location + date_time + '_' + "recorded_cop" + ".csv";*/
    string path = dir_path + "recorded_cop" + ".csv";
    
    ofstream recorded_cop(path.c_str(), ios::app);
    
    if (recorded_cop.is_open())
    {
        recorded_cop << msg.header.stamp.sec << "," << msg.point.x <<  "," << msg.point.y << "," << msg.point.z << "\n";
        recorded_cop.close();
    }
    else
        cout << "Unable to open file." << endl;      
}


void recordCOP_filtered(const geometry_msgs::PointStamped& msg)
{
    string path = dir_path + "recorded_cop_filtered" + ".csv";

    ofstream recorded_cop_filtered(path.c_str(), ios::app);
    
    if (recorded_cop_filtered.is_open())
    {
        recorded_cop_filtered << msg.header.stamp.sec << "," << msg.point.x <<  "," << msg.point.y << "," << msg.point.z << "\n";
        recorded_cop_filtered.close();
    }
    else
        cout << "Unable to open file." << endl;      
}


void recordPHUA(const geometry_msgs::PoseStamped& msg)
{     
    string path = dir_path + "recorded_phua" + ".csv";

    ofstream recorded_phua(path.c_str(), ios::app);
    
    if (recorded_phua.is_open())
    {
        recorded_phua << msg.header.stamp.sec << "," << msg.pose.position.x <<  "," << msg.pose.position.y << "," << msg.pose.position.z - 0.075 << "\n";
        recorded_phua.close();
    }
    else
        cout << "Unable to open file." << endl;      
}


void recordPHUA_theoretical(const geometry_msgs::PointStamped& msg)
{
    string path = dir_path + "recorded_phua_theoretical" + ".csv";

    ofstream recorded_phua_theoretical(path.c_str(), ios::app);
    
    if (recorded_phua_theoretical.is_open())
    {
        recorded_phua_theoretical << msg.header.stamp.sec << "," << -msg.point.x <<  "," << -msg.point.y << "," << -msg.point.z << "\n";
        recorded_phua_theoretical.close();
    }
    else
        cout << "Unable to open file." << endl;      
}


void recordReaction(const geometry_msgs::Vector3Stamped& msg)
{
    string path = dir_path + "recorded_reaction" + ".csv";

    ofstream recorded_reaction(path.c_str(), ios::app);
    
    if (recorded_reaction.is_open())
    {
        recorded_reaction << msg.header.stamp.sec << "," << msg.vector.x <<  "," << msg.vector.y << "," << msg.vector.z << "\n";
        recorded_reaction.close();
    }
    else
        cout << "Unable to open file." << endl;      
}


void recordForce_stab(const geometry_msgs::Vector3Stamped& msg)
{
    string path = dir_path + "recorded_force_stab" + ".csv";
    
    ofstream recorded_force_stab(path.c_str(), ios::app);
    
    if (recorded_force_stab.is_open())
    {
        recorded_force_stab << msg.header.stamp.sec << "," << msg.vector.x <<  "," << msg.vector.y << "," << msg.vector.z << "\n";
        recorded_force_stab.close();
    }
    else
        cout << "Unable to open file." << endl;      
}


void recordForce_instab(const geometry_msgs::Vector3Stamped& msg)
{
    string path = dir_path + "recorded_force_instab" + ".csv";

    ofstream recorded_force_instab(path.c_str(), ios::app);
    
    if (recorded_force_instab.is_open())
    {
        recorded_force_instab << msg.header.stamp.sec << "," << msg.vector.x <<  "," << msg.vector.y << "," << msg.vector.z << "\n";
        recorded_force_instab.close();
    }
    else
        cout << "Unable to open file." << endl;      
}


void recordForce(const geometry_msgs::Vector3Stamped& msg)
{
    string path = dir_path + "recorded_force" + ".csv";
    
    ofstream recorded_force(path.c_str(), ios::app);
    
    if (recorded_force.is_open())
    {
        recorded_force << msg.header.stamp.sec << "," << msg.vector.x <<  "," << msg.vector.y << "," << msg.vector.z << "\n";
        recorded_force.close();
    }
    else
        cout << "Unable to open file." << endl;      
}


void recordForce_filtered(const geometry_msgs::Vector3Stamped& msg)
{
    string path = dir_path + "recorded_force_filtered" + ".csv";
    
    ofstream recorded_force_filtered(path.c_str(), ios::app);
    
    if (recorded_force_filtered.is_open())
    {
        recorded_force_filtered << msg.header.stamp.sec << "," << msg.vector.x <<  "," << msg.vector.y << "," << msg.vector.z << "\n";
        recorded_force_filtered.close();
    }
    else
        cout << "Unable to open file." << endl;      
}


void recordState(const sensor_msgs::JointState& msg)
{
    string path_t = dir_path + "recorded_joints_torque" + ".csv";
    string path_p = dir_path + "recorded_joints_position" + ".csv";
    string path_v = dir_path + "recorded_joints_velocity" + ".csv";
   
    ofstream recorded_joints_torque(path_t.c_str(), ios::app);
    ofstream recorded_joints_position(path_p.c_str(), ios::app);
    ofstream recorded_joints_velocity(path_v.c_str(), ios::app);
    
    if (recorded_joints_torque.is_open())
    {
        recorded_joints_torque << msg.header.stamp.sec << "," << msg.effort[0] << "," << msg.effort[1] <<  "," << msg.effort[2] << "," << msg.effort[3] << "," << msg.effort[4] <<  "," << msg.effort[5] << "," << msg.effort[7] <<  "," << msg.effort[8] << "," << msg.effort[9] << "," << msg.effort[10] <<  "," << msg.effort[11] << "," << msg.effort[12] << "\n";
        recorded_joints_torque.close();
    }
    else
        cout << "Unable to open file." << endl; 
    
    
    if (recorded_joints_position.is_open())
    {
        recorded_joints_position << msg.header.stamp.sec << "," << msg.position[0] << "," << msg.position[1] <<  "," << msg.position[2] << "," << msg.position[3] << "," << msg.position[4] <<  "," << msg.position[5] << "," << msg.position[7] <<  "," << msg.position[8] << "," << msg.position[9] << "," << msg.position[10] <<  "," << msg.position[11] << "," << msg.position[12] << "\n";
        recorded_joints_position.close();
    }
    else
        cout << "Unable to open file." << endl; 
    
    
    if (recorded_joints_velocity.is_open())
    {
        recorded_joints_velocity << msg.header.stamp.sec << "," << msg.velocity[0] << "," << msg.velocity[1] <<  "," << msg.velocity[2] << "," << msg.velocity[3] << "," << msg.velocity[4] <<  "," << msg.velocity[5] << "," << msg.velocity[7] <<  "," << msg.velocity[8] << "," << msg.velocity[9] << "," << msg.velocity[10] <<  "," << msg.velocity[11] << "," << msg.velocity[12] << "\n";
        recorded_joints_velocity.close();
    }
    else
        cout << "Unable to open file." << endl; 
    
}


void recordBOS(const geometry_msgs::PolygonStamped& msg)
{
    string path = dir_path + "recorded_bos" + ".csv";
    
    ofstream recorded_bos(path.c_str(), ios::app);
    
    if (recorded_bos.is_open())
    {
        recorded_bos << msg.header.stamp.sec << ",";
        
        for (uint i=0; i<msg.polygon.points.size()-1; i++)
        {
	  recorded_bos << msg.polygon.points[i].x << "," << msg.polygon.points[i].y << ",";
        }
        
        recorded_bos << msg.polygon.points[msg.polygon.points.size()-1].x << "," << msg.polygon.points[msg.polygon.points.size()-1].y << "\n";
        
        recorded_bos.close();
    }
    else
        cout << "Unable to open file." << endl;      
}


void recordIMU(const vrep_common::ImuData& msg)
{
    string path = dir_path + "recorded_imu_data" + ".csv";
    
    ofstream recorded_imu_data(path.c_str(), ios::app);
    
    if (recorded_imu_data.is_open())
    {
        recorded_imu_data << msg.unitData[0].header.stamp.sec << ",";
        
        for (uint i=0; i<msg.unitData.size()-1; i++)
        {
	  recorded_imu_data << msg.unitData[i].angular_velocity.x << "," << msg.unitData[i].angular_velocity.y << "," << msg.unitData[i].angular_velocity.z << "," << msg.unitData[i].linear_acceleration.x << "," << msg.unitData[i].linear_acceleration.y << "," << msg.unitData[i].linear_acceleration.z << ",";
        }
        
        recorded_imu_data << msg.unitData[msg.unitData.size()-1].angular_velocity.x << "," << msg.unitData[msg.unitData.size()-1].angular_velocity.y << "," << msg.unitData[msg.unitData.size()-1].angular_velocity.z << "," << msg.unitData[msg.unitData.size()-1].linear_acceleration.x << "," << msg.unitData[msg.unitData.size()-1].linear_acceleration.y << "," << msg.unitData[msg.unitData.size()-1].linear_acceleration.z << "\n";
        
        recorded_imu_data.close();
    }
    else
        cout << "Unable to open file." << endl;  
}


void recordPlatform_left(const geometry_msgs::PoseStamped& msg)
{     
    string path = dir_path + "recorded_platform_left" + ".csv";

    ofstream recorded_platform_left(path.c_str(), ios::app);
    
    if (recorded_platform_left.is_open())
    {
        recorded_platform_left << msg.header.stamp.sec << "," << msg.pose.position.x << "," << msg.pose.position.y << "," << msg.pose.position.z << "," << msg.pose.orientation.x << "," << msg.pose.orientation.y << "," << msg.pose.orientation.z << "," << msg.pose.orientation.w << "\n";
        recorded_platform_left.close();
    }
    else
        cout << "Unable to open file." << endl;      
}


void recordPlatform_right(const geometry_msgs::PoseStamped& msg)
{     
    string path = dir_path + "recorded_platform_right" + ".csv";

    ofstream recorded_platform_right(path.c_str(), ios::app);
    
    if (recorded_platform_right.is_open())
    {
        recorded_platform_right << msg.header.stamp.sec << "," << msg.pose.position.x << "," << msg.pose.position.y << "," << msg.pose.position.z << "," << msg.pose.orientation.x << "," << msg.pose.orientation.y << "," << msg.pose.orientation.z << "," << msg.pose.orientation.w << "\n";
        recorded_platform_right.close();
    }
    else
        cout << "Unable to open file." << endl;      
}


int main(int argc, char **argv)
{      
    time_t rawtime;
    time (&rawtime);
    
    struct tm * timeinfo;
    timeinfo = localtime (&rawtime);
    
    char date_time[16];
    
    strftime(date_time, sizeof(date_time), "%Y%m%d_%H%M%S", timeinfo);
    
    ros::init(argc, argv, "record_data");
    ros::NodeHandle nh;
    
    if (!ros::param::get("plane", plane))
        ROS_ERROR("Failed to read symbols on parameter server");
    else if (!ros::param::get("amplitude", amplitude))
        ROS_ERROR("Failed to read symbols on parameter server");
    else if (!ros::param::get("frequency", frequency))
        ROS_ERROR("Failed to read symbols on parameter server");
    else if (!ros::param::get("user", user))
        ROS_ERROR("Failed to read symbols on parameter server");
    
    string dir_location ("/home/phua3-lar/Experiments/Results_");
    dir_path = dir_location + date_time + '_' + plane + "_amp" + amplitude + "_freq" + frequency + "_user" + user + '/';
    mkdir(dir_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    
    ros::Subscriber simulation_state_sub = nh.subscribe("/vrep/info", 1000, simulationState);
    ros::Subscriber phua_position_sub = nh.subscribe("/phua_position", 1000, recordPHUA);
    ros::Subscriber joints_state_sub = nh.subscribe("/phua_joints_state", 1000, recordState);
    ros::Subscriber cop_global_position_sub = nh.subscribe("/cop_global_position", 1000, recordCOP);
    ros::Subscriber support_base_position_sub = nh.subscribe("/support_base_position", 1000, recordBOS);
    /* ros::Subscriber cop_global_filtered_sub = nh.subscribe("/cop_global_filtered", 1000, recordCOP_filtered); */
    ros::Subscriber phua_theoretical_position_sub = nh.subscribe("/command_pelvis_position_stamped", 1000, recordPHUA_theoretical);
    ros::Subscriber feedback_force_stability_sub = nh.subscribe("/feedback_force_stability", 1000, recordForce_stab);
    ros::Subscriber feedback_force_instability_sub = nh.subscribe("/feedback_force_instability", 1000, recordForce_instab);
    ros::Subscriber feedback_force_sub = nh.subscribe("/feedback_force", 1000, recordForce);
    ros::Subscriber feedback_force_filtered_sub = nh.subscribe("/feedback_force_filtered", 1000, recordForce_filtered);
    ros::Subscriber reaction_force_sub = nh.subscribe("/reaction_force", 1000, recordReaction);
    ros::Subscriber imu_network_data_sub = nh.subscribe("/imu_network_data", 1000, recordIMU);
    ros::Subscriber left_platform_sub = nh.subscribe("/vrep/command_left_platform", 1000, recordPlatform_left);
    ros::Subscriber right_platform_sub = nh.subscribe("/vrep/command_right_platform", 1000, recordPlatform_right);
    
    ros::spin();

    return 0;
}
