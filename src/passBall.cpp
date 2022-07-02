DPoint r2b = ball_pos_ - robot_pos_;
if (last_dribble != world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState())
    ROS_INFO("change::");
last_dribble = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState();

if (!last_dribble) // catchball
{
    action_cmd_.handle_enable = 1;
    if (mv2orif_e1(r2b.angle().radian_, robot_ori_.radian_))
        move2target(ball_pos_, robot_pos_);
    action_cmd_.move_action = CatchBall;
    action_cmd_.rotate_acton = CatchBall;
    action_cmd_.rotate_mode = 0;
}

else // passball
{
    int pass__mode = RUN;
    auto pass_vec_ = world_model_info_.RobotInfo_[1].getLocation() - robot_pos_;

    if (pass_vec_.length() > 650) // dis > 650 fly
        pass__mode = FLY;

    shoot_flag = m_plan_.PassBall_Action(2, pass__mode);
}

bool Plan::PassBall_Action(int catch_ID, int pass_mode_)
{
    world_model_->pass_ID_ = world_model_->AgentID_;
    world_model_->catch_ID_ = catch_ID;
    world_model_->catch_pt_ = world_model_->RobotInfo_[catch_ID - 1].getLocation();
    bool shoot_flag = false;
    bool kick_off = false;

    world_model_->is_passed_out_ = true;
    world_model_->pass_cmds_.isvalid = true;

    DPoint pass_target_ = world_model_->RobotInfo_[catch_ID - 1].getLocation() - robot_pos_;
    if (m_behaviour_.move2orif(pass_target_.angle().radian_, robot_ori_.radian_, 0.087))
    {
        world_model_->is_static_pass_ = 1;
        action->shootPos = pass_mode_;

        if (pass_mode_ == -1) // FLY
            action->strength = sqrt((pass_target_.length() / 100.0 - 2.0) * 9.8);
        else
        {
            action->strength = action->strength = pass_target_.length() / 30;
            if (action->strength < 5.0)
                action->strength = 5.0;
        }
        shoot_flag = true;
        std::cout << "pass out" << std::endl;
        kick_off = true;
    }
    return shoot_flag;
}
