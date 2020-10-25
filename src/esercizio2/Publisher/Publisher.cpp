#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string.h>
using namespace std;
class Encoder{
public:
    bool type_joint;
    int encoder_id;
    string encoder_name;
    int position;
    int max_length;
   Encoder(){
     encoder_id=0;
     encoder_name="Enconder :";
     position=0;	
    }
    void set_type_joint(bool type,int maxlength){
    	type_joint=type;
    	if (type_joint==false){
            max_length=maxlength;
    	}
    }
    void set_encoder_id(int encoderid){
         encoder_id=encoderid;
    }
    void set_encoder_name(string encodername){
	encoder_name=encodername;
    }
    int  get_encoder_id(void){
	return encoder_id;
    }
    string get_encoder_name(void){
    	return encoder_name;
    }
    int get_encoder_misuration(void){
 	if (type_joint==true){
           position=rand()%360+1;
           return position;
    }else{
        position=rand()%max_length+1;
        return position;
    }
    }
    string to_string(void){
    if(type_joint==true){
        return "Encoder id: "+ std::to_string(encoder_id)+" Name:"+encoder_name+std::to_string(encoder_id)+" Misuration: "+std::to_string(get_encoder_misuration())+"grad ";  
    	}else{
        return "Encoder id: "+ std::to_string(encoder_id)+" Name: "+encoder_name+std::to_string(encoder_id)+" Misuration: "+std::to_string(get_encoder_misuration())+"m ";
        }
    }
};


int main(int argc, char** argv) {
    Encoder enc[6];
    ros::init(argc, argv, "encoderPubliesher");
    ros::NodeHandle nh;
    ros::Publisher encoderPublisher=nh.advertise<std_msgs::String>("/echo", 1000);
    ros::Rate loopRate(10);
    int number;
    bool type;
    int max_length;
     for(int i=0;i<6;i++){
        number==rand()%2;
        type=number==1;
         if(type==true)
            enc[i].set_type_joint(type,0);
        else{
            max_length=rand()%2000+1;
            enc[i].set_type_joint(type,max_length);
        }
	enc[i].set_encoder_id(i+1);
     }
     while(ros::ok()) {
         int enc_number=rand()%6;
         std_msgs::String message;
         message.data=enc[enc_number].to_string();
         ROS_INFO_STREAM(message.data);
         encoderPublisher.publish(message);
         ros::spinOnce();
         loopRate.sleep();
	}
 return 0;
}
