#include<ros/ros.h>
#include<kinematic_service_msgs/GetForwardKinematicSolution.h>
#include<moveit/robot_state/robot_state.h>
#include<moveit/robot_state/conversions.h>
#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit_msgs/GetPositionFK.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc,char **argv){
    ros::init(argc,argv,"kinematics_service_client");
    ros::NodeHandle nh;

    ros::ServiceClient client=nh.serviceClient<kinematics_service_msgs::GetForwardKinematicsSolution>("compute_forward_kinematics");
    
    kinematics_service_msgs::GetForwardKinematicSolution fk_service;
    
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));
    
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

    moveit::core::RobotState robot_state(kinematic_model);

    const std::vector<double> join_positions={0,0.341793,0.00991429,0,-1.9225,-3.14159};

    std::string planning_group_name;
    
    if(!nh.getParam("planning_group_name",planning_group_name)){
        ROS_ERROR("planning_group_name is undefined on the parameter server");
        return false;
    }

   const robot_state::JointModelGroup *joint_model_group=kinematic_model->getLinkModelGroup(planning_group_name);
   
   robot_state.setJointGroupPositions(joint_model_group,joint_positions);
   
   moveit::core::roborStateToRobotStateMsg(robot_state,fk_service.request.robot_state);
   
   if(!client.call(fk_service)){
       ROS_ERROR("Could not compute forword Kinematics");
   }
   
   tf2::Quaterion quaternion;
   tf2::fromMsg(fk_service.respose.end_effector_pose.orientation,quaternion);

    tf2::Matrix3x3 matrix(quaternion);
    tf2Scalar roll,pitch,yaw;
    matrix.getRPY(roll,pitch,yaw);

    std::ostringstream output_msgs;

    output_msgs<<"Computed forward kinematic solution with custom solver:"<<std::endl;
    output_msgs<<"Prosition (XYZ): ["<<std::endl;
    output_msgs<<fk_service.response,end_effector_pose.position.x<<",";
    output_msgs<<fk_service.response,end_effector_pose.position.y<<",";
    output_msgs<<fk_service.response,end_effector_pose.position.z<<"]"<<std::endl;
    output_msgs<<"Orientation (RPY): ["<<roll<<", "<<pitch<<", "<<yaw<<"]";
    
    ROS_INFO_STREAM(output_msgs.str());
    
    client=nh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
    std::vector<std::sting> link_names = joint_model_group->getLinkNames();

    move_group_fk_service.request.handler.frame_id=link_names[0];
    move_group_fk_service.request.fk_link_names.push_back(link_names.back());
    moveit::core::robotStateToRobotStateMsg(robot_state,move_group_fk_service.request.robot_state);

    if(!client.call(move_group_fk_service)){
        ROS_ERROR("Could not compute forward kineematics");
    }
    tf2::formMsg(move_group_fk_service.respose.pose_stamped[0].pose.orientation,quaterion);
    tf2::Matrix3x3 rotation_matrix(quaternion);
    rotation_matrix.getRPY(roll,pitch,yaw);
    std::ostringstream msg;
    msg<<"Computed forward kinematic solution with move_group solver:"<<std::endl;
    msg<<"Position (XYZ): [";
    msg<<move_group_fk_service.response.pose_stamped[0].pose.position.x<<", ";
    msg<<move_group_fk_service.response.pose_stamped[0].pose.position.y<<", ";
    msg<<move_group_fk_service.response.pose_stamped[0].pose.position.z<<"] "<<std::endl;
    msg<<"Orentiation (RPY): ["<<roll<<","<<pitch<<", "<<yaw<<"]";

    ROS_INFO_STREAM(msg.str());
    ros::shutdown();
    return 0;
    }