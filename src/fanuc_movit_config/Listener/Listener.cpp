#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
using namespace std;


int main(int argc,char** argv){
    std::string joint_name[8]={"base_link", "link1", "link2", "link3", "link4", "link5", "flange"};
    ros::init(argc,argv,"tf_listener");
    ros::NodeHandle node;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(1);
    while (node.ok()){
      try{
      for(int i=1;i<=7;i++){
        geometry_msgs::TransformStamped transformStamped;
        std_msgs::String outstring;
        std::stringstream ss;
        transformStamped=tfBuffer.lookupTransform(joint_name[i],joint_name[i-1],ros::Time(0));
        ss<<"\n************ Transformation from "<<joint_name[i-1]<<" to "<<joint_name[i]<<" ************"<<endl;
        float translationx=transformStamped.transform.translation.x;
        float translationy=transformStamped.transform.translation.y;
        float translationz=transformStamped.transform.translation.z;
        ss<<"-----Translation-----"<<endl;
        ss<<"x:"<<translationx<<"\n"<<endl;
        ss<<"y:"<<translationy<<"\n"<<endl;
        ss<<"z:"<<translationz<<"\n"<<endl;
        float x=transformStamped.transform.rotation.x;
        float y=transformStamped.transform.rotation.y;
        float z=transformStamped.transform.rotation.z;
        float w=transformStamped.transform.rotation.w;
        //Print Quaternion at matlab
        ss<<"-----Quaternion----"<<endl;
        ss<<"x:"<<x<<endl;  
        ss<<"y:"<<y<<endl;
        ss<<"z:"<<z<<endl;
        ss<<"w:"<<w<<endl;
        //Define elements of rotation matrix
        float r11=(2*(pow(w,2)+pow(x,2)))-1;
        float r12=2*(x*y-w*z);
        float r13=2*(x*z+w*y);
        float r21=2*(x*y+w*z);
        float r22=(2*(pow(w,2)+pow(y,2)))-1;
        float r23=2*(y*z-w*x);
        float r31=2*(x*z-w*y);
        float r32=2*(y*z+w*x);
        float r33=(2*(pow(w,2)+pow(z,2)))-1;
        float theta=acos((r11+r22+r33-1)/2);
        float raa1=(1/(2*sin(theta)))*(r32-r23);
        float raa2=(1/(2*sin(theta)))*(r13-r31);
        float raa3=(1/(2*sin(theta)))*(r21-r12);
        //If theta is 0, choose the parameter equals of last row of matrix rotations
        if(theta==0){
          raa1=r31;
          raa2=r32;
          raa3=r33;      
        }
        ss<<"-----Axsis/angle-----"<<endl;     
        ss<<"Axsis=["<<raa1<<" "<<raa2<<" "<<raa3<<"]"<<endl;
        ss<<"Angle= "<<theta<<" "<<endl;
        //Print of matrix rotation
        ss<<"-----Rotation-----"<<endl;
        ss<<"["<<r11<<" "<<r12<<" "<<r13<<"]"<<endl;
        ss<<"["<<r21<<" "<<r22<<" "<<r23<<"]"<<endl;
        ss<<"["<<r31<<" "<<r32<<" "<<r33<<"]"<<endl;
        ss<<"----Eulero Angle RPY-----"<<endl;
        float roll=0;
        float pitch=0;
        float yaw=0;
        //if(theta>-1.57 && theta<1.57){
          roll=atan2(r21,r11);
          pitch=atan2(-r31,sqrt(pow(r32,2)+pow(r33,2)));
          yaw=atan2(r32,r33);
        //}else if (theta>1.57 && theta<4.71){
          //roll=atan2(-r21,-r11);
          //pitch=atan2(-r31,-sqrt(pow(r32,2)+pow(r33,2)));
          //yaw=atan2(-r32,-r33);
        //}
        ss<<"["<<roll<<" "<<pitch<<" "<<yaw<<"]"<<endl;
        outstring.data=ss.str();
        ROS_INFO("%s",outstring.data.c_str());
        ros::Duration(2).sleep();
      }
      }catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
    }
}