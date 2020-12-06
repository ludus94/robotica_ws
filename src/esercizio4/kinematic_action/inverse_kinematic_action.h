#include <actionlib/server/simple_action_server.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace kinematics_action{

class  InverseKinematicsAction{



public:
     InverseKinematicsAction();
    ~ InverseKinematicsAction();
private:
    void ik_callback_(const kinematics_action_msgs::GetInverseKinematicSolutionsGoalConstPtr &goal);
    bool isSolutionNew_(const std::vector<double> &solution) const;
    std::vector<double>generateSeedState_() const;
    void normalizeJointPositions_(std::vector<double> &solution) const;

    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<kinematics_action_msgs::GetInverseKinematicSolutionsAction> action_server_;
    
    std::vector<std::vector<double>> ik_solutions_;
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader_;
    robot_model::RobotModelConstPtr kinematic_model_;
    const robot_state::JointModelGroup *jmg_;
};

}