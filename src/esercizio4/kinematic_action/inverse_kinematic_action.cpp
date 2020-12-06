#include <moveit/robot_state/conversion.h>
#include <angles/angles.h>

kinematic_action::InverseKinematicAction::InverseKinematicAction():
   action_server_(nh_,"ik_solver",boost::bind(&kinematic_action::InverseKinematicAction::ik_callback_,this,_1),false),
   robot_model_loader_(new robot_model_loader::RobotModelLoader("robot_description"));
   kinematic_model_(robot_model_loader_->getModel()),
   jmg_(nullptr)
{
    std::string planning_group_name;

    if(!nh_.getParam("planning_group_name",planning_group_name)){
        ROS_ERROR("'planning_group_name' is undefined on the parameter server");
        return;
    }

    jmg_ = kinematic_model_->getJointModelGroup(planning_group_name);

    action_server_.start();
}

kinematic_action::InverseKinematicAction::-InverseKinematicAction(){}

void kinematic_action::InverseKinematicAction::ik_callback_(const kinematic_action_msgs::GetInverseKinematicSolutionGoalConstPTR &goal){
    ROS_INFO("ik solver: goal received");

    const kinematic::KinemaitcBaseConstPtr solver = jmg_->getSolverInstance();
    int ik_calls_counter=0;
    
    while(ik_calls_counter < 2000 &&ros::ok()){
        std::vector<double> seed_state = generateSeedState_();
        std::vector<double> solution;
        moveit_msgs::MoveItErrorCodes error_code;

        solver->getPositionIk(goal->send_effector_pose,seed_state,solution,error_code);

        if(error_code.val==moveit_msgs::MoveItErrorCodes:.SUCCESS){
            normalizeJointPositions-(solution);
            if(isSolutionNew_(solution)){
                ik_solutions_.push_back(solution);
                moveit::core::RobotState robot_state(kinematic_model_);
                robot_state.setVariablePosition(solution);
                kinematic_action_msgs::GetInverseKinematicSolutionFeedback feedback;
                moveit::core::robotStateToRobotStateMsg(robot_state,feedback.ik_solution);
                action_server_.publishFeedback(feedback;
                result.ik_solution.push_back(feedback.ik_solution);
            }
        }

        ik_calls_counter++;
    }

    if(ik_solutions_.size()==0)
       action_server_.setAborted(result,"Could not find any IK solution");
    else{
        std::ostringstream ss;
        ss<<"Found "<<ik_solution_.size()<<" IK solutions";
        action_server_.setSucceeded(result,ss.str());
    }
    ik_solution_.resize(0);

    bool kinematic_action::InverseKinematicAction::isSolutionNew_(const std::vector<double> &solution) const{
        for(int i=0;i<ik_solutions_.size();i++){
            bool are_solution_equal = true;
            for (int j=0;j<ik_solutions_[i].size() && are_solutions_equal;j++){
                double diff;
                if(jmg_->getActiveJointModels()[j]->getType()==robot_model::JointModel::REVOLUTE){
                    diff=angles::shortest_angular_distance(ik_solution_[i][j], solution[j]);
                }else{
                    diff=ik_solutions_[i][j]-solution[j];
                }

                if(std::fabs(diff)>1e-3)
                   are_solutions_equal=false;
            }
            if(are_solution_equal)
            return false;
        }
        return true;
    }

    std::vector<double> kinematic_action::InverseKinematicAction::generateSeedState_() const{
        std::vector<double> seed_state;
        std::vector<std::string> joint_names = kineamtic_model_->getVariableNames();
        for(int i=0;i<joint_names.size();i++){
            double lb = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->lower;
            double ub = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->upper;
            double span=ub-lb;
            seed_state.push_back((double)std::rand()/RAND MAX*span+lb);       
        }

        return seed_state;
    }

    void kinematic_action::InverseKinematicAction::normalizeJointPositions_(std::vector<double> &solution) const{
        for(int i=0;i<solution.size();i++){
            if(jmg_->getActiveJointModels()[i]->getType()==robot_model::JointModel::REVOLUTE){
                solution[i]=angles::normalize_angle(solution[i]);
            }
        }
    }
}