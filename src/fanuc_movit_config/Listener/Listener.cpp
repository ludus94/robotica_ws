#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using namespace std;


int main(int argc,char** argv){
    std::string joint_name[7]={"base_link", "link1", "link2", "link3", "link4", "link5", "flange"};
    ros::init(argc,argv,"tf_listener");
    ros::NodeHandle node;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(1);
    while (node.ok()){
      try{
      for(int i=0;i<6;i++){
        geometry_msgs::TransformStamped transformStamped;
        std::ostringstream ss;
        transformStamped=tfBuffer.lookupTransform(joint_name[i],joint_name[6],ros::Time(0));
        ss<<"\n************ Transformation from "<<joint_name[i]<<" to "<<joint_name[6]<<" ************"<<std::endl;
        ss<<"-----Translation-----"<<std::endl;
        ss<<transformStamped.transform.translation;
        
        tf2::Quaternion quaternion;
        tf2::fromMsg(transformStamped.transform.rotation,quaternion);
        tf2::Vector3 rotation_axis=quaternion.getAxis();
        
        ss<<"-----Axsis/angle-----"<<std::endl; 
        ss<<"Axsis=["<<rotation_axis.getX()<<" "<<rotation_axis.getY()<<" "<<rotation_axis.getZ()<<"]"<<std::endl;
        ss<<"Angle= "<<quaternion.getAngle()<<" "<<std::endl;

        tf2::Matrix3x3 matrix=tf2::Matrix3x3(quaternion);

        
        //Print of matrix rotation
        ss<<"-----Rotation-----"<<endl;
        ss<<"["<<matrix[0][0]<<" "<<matrix[0][1]<<" "<<matrix[0][2]<<"]"<<std::endl;
        ss<<"["<<matrix[1][0]<<" "<<matrix[1][1]<<" "<<matrix[1][2]<<"]"<<std::endl;
        ss<<"["<<matrix[2][0]<<" "<<matrix[2][1]<<" "<<matrix[2][2]<<"]"<<std::endl;
        ss<<"----Eulero Angle RPY-----"<<std::endl;

        tf2Scalar roll,pitch,yaw;
        
        matrix.getRPY(roll,pitch,yaw);        
        ss<<"["<<roll<<" "<<pitch<<" "<<yaw<<"]"<<endl;
        ROS_INFO_STREAM(ss.str());
        rate.sleep();
      }
      }catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
    }
}