if (world_model_info_.pass_state_.catch_id_ == world_model_info_.AgentID_)
{
    //收到信号后首先朝向球
    action_cmd_.rotate_acton = CatchBall;
    action_cmd_.move_action = No_Action;
    action_cmd_.rotate_mode = 0;
    m_plan_.m_behaviour_.move2orif(r2b.angle().radian_, robot_ori_.radian_, 0.087);

    DPoint headoffPoint;

    //垂直运动到球路径上
    if (ball_vel_.length())
    {
        // action_cmd_.move_action = CatchBall;
        double k1 = ball_vel_.y_ / ball_vel_.x_;
        double b1 = ball_pos_.y_ - k1 * ball_pos_.x_;
        double k2 = -1.0 / k1;
        double b2 = robot_pos_.y_ - k2 * robot_pos_.x_;
        //球路径上的点
        headoffPoint = DPoint((b2 - b1) / (k1 - k2), (k1 * b2 - k2 * b1) / (k1 - k2));
    }

    if ((action_cmd_.move_action = CatchBall) /*解除移动限制*/ && move2target(headoffPoint, robot_pos_))
    {
        // catchball
        if (robot_pos_.distance(ball_pos_) <= 250.0 && !world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState() && !world_model_info_.RobotInfo_[world_model_info_.pass_state_.pass_id_ - 1].getDribbleState())
        {
            action_cmd_.handle_enable = 1;
            action_cmd_.move_action = CatchBall;
            action_cmd_.rotate_acton = CatchBall;
            action_cmd_.rotate_mode = 0;
            if (mv2orif_e1(r2b.angle().radian_, robot_ori_.radian_))
                move2target(robot_pos_, robot_pos_);
        }
    }

    //   ?
    if (whichopp_dribble() >= 0)
        world_model_info_.pass_state_.reset();
}

/*...其他动作...*/