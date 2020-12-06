#include <ros/ros.h>
#include <kinematic_service_msgs/GetForwardKinematicSolution.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <eigen_conversions/eigen_msgs.h>

bool computeForwardKinematicSolutions(kinematics_service_msgs::GetForwardKinematicSolutionRequest &req,kinematics_service_msgs::GetForwardKinematicSolutionResponse &res);

int main(int argc,char **argv){
    ros::init(argc,argv,"kinematics_service_server");
    ros::NodeHandle nh;
    ros::ServiceServer service_server=nh.advertiseService("compute_forward_kinematics",computeForwardKinematicSolutions);
    ROS_INFO("Started forward kinematics service");
    ros::spin();
    ros::shutdown();
    return 0;
}
bool computeForwardKinematicSolutions(kinematics_service_msgs::GetForwardKinematicSolutionRequest &req,kinematics_service_msgs::GetForwardKinematicSolutionResponse &res){

    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));    
    
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();
    
    moveit::core::RobotState robot_state(kinematic_model);
    
    moveit::core::robotStateMsgToRobotState(req.robot_state,robot_state);

    robot_state.updateLinkTransforms();

    ros::NodeHandle nh;
    std::string planning_group_name;

    if(!nh.getParam("planning_group_name",planning_group_name)){
        ROS_ERROR("planning_group_name is undefined on the parameter server");
        return false;
    }
    
    const robot_state::JointModelGroup *joint_model_group=kinematic_model->getLinkModelGroup(planning_group_name);
    
    std::vector<std::string> link_names=joint_model_group->getLinkModelNames();
    
    Eigen::Isometry3d forword_kinematics=robot_state.getGlobalLinkTransform(link_names.back());
    
    tf::poseEigenToMsg(forword_kinematis,res.end_effector_pose);
    
    return true;
}
