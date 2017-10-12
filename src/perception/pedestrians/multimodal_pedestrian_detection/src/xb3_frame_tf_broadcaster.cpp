#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_tf_broadcaster");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(10.0);
    
    while (node.ok())
    {
        transform.setOrigin( tf::Vector3( -1.8375, 0.2646, 1.1118) );

        
        tf::Matrix3x3 tf3d;
        tf3d.setValue(  -0.0184,   -0.1384,    0.9902,
                        -0.9998,    0.0052,   -0.0179,
                        -0.0027,   -0.9904,   -0.1384);

        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);
        
//         transform.setRotation( tf::Quaternion(1.597001304715086, -0.8908563685714233, -1.581197539311177) );
        
        transform.setRotation(tfqt);
        
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "atc/vehicle/center_bumper", "atc/vehicle/carrot1"));
        rate.sleep();
    }
    
    return 0;
}; 