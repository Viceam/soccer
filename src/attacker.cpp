isactive = true;
if (!shoot_flag)
{
    DPoint b2r = ball_pos_ - robot_pos_;
    DPoint t2r = tmp - robot_pos_;
    // DPoint shoot_line = world_model_info_.field_info_.ourGoal_[GOAL_MIDDLE] - robot_pos_;
    if (last_dribble != world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState())
        ROS_INFO("change::");
    last_dribble = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState();
    if (!world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState())
    {
        action_cmd_.handle_enable = 1;
        if (move2ori(b2r.angle().radian_, robot_ori_.radian_))
            move2target(ball_pos_, robot_pos_);
        action_cmd_.move_action = CatchBall;
        action_cmd_.rotate_acton = CatchBall;
        action_cmd_.rotate_mode = 0;
    }
    else if (robot_pos_.distance(tmp) > 25.0)
    {
        action_cmd_.move_action = MoveWithBall;
        action_cmd_.rotate_acton = MoveWithBall;
        action_cmd_.rotate_mode = 0;
        if (move2ori(t2r.angle().radian_, robot_ori_.radian_))
            move2target(tmp, robot_pos_);
    }
    else
    {
        action_cmd_.move_action = TurnForShoot;
        action_cmd_.rotate_acton = TurnForShoot;
        action_cmd_.rotate_mode = 0;

        auto pre_for_shoot = [this]()
        {
            if (world_model_info_.RobotInfo_[0].getLocation().y_ > 0)
                return world_model_info_.field_info_.ourGoal_[GOAL_MIDLOWER];
            else
                return world_model_info_.field_info_.ourGoal_[GOAL_MIDLOWER];
        };

        static DPoint shoot_target = pre_for_shoot();
        // DPoint mid((d1.x_ + d2.x_) / 2.0, (d1.y_ + d2.y_) / 2.0);
        // auto shoot_line = mid - robot_pos_;
        auto shoot_line = shoot_target - robot_pos_;
        move2target(tmp, robot_pos_);
        mv2orif_e1(shoot_line.angle().radian_, robot_ori_.radian_);

        {
            // double low_radian_ = (d1 - robot_pos_).angle().radian_;
            // double up_radian_ = (d2 - robot_pos_).angle().radian_;
            // if (low_radian_ > up_radian_)
            //     std::swap(low_radian_, up_radian_);

            //求出球的运动轨迹方程
            double t_x = robot_pos_.x_, t_y = robot_pos_.y_;
            auto k = tan(robot_ori_.radian_);
            double b = t_y - k * t_x;
            //与 x = -1100 的交点
            double y0 = -1100.0 * k + b;

            bool b1 = robot_ori_.radian_ / DEG2RAD >= 90.0 || robot_ori_.radian_ / DEG2RAD <= -90.0;
            if (b1 && y0 <= 95 && y0 >= -95 && fabs(y0 - world_model_info_.RobotInfo_[0].getLocation().y_) >= 66.0)
            {
                action_cmd_.shootPos = RUN /*FLY*/;
                action_cmd_.strength = shoot_line.length() / 15;
                if (action_cmd_.strength < 10.0)
                    action_cmd_.strength = 10.0;
                shoot_flag = true;
                std::cout << "shoot done " << std::endl;
            }
            if (fabs(shoot_line.angle().radian_ - robot_ori_.radian_) <= 0.10)
            {
                if (shoot_target == world_model_info_.field_info_.ourGoal_[GOAL_UPPER])
                    shoot_target = world_model_info_.field_info_.ourGoal_[GOAL_LOWER];
                else
                    shoot_target = world_model_info_.field_info_.ourGoal_[GOAL_UPPER];
            }
        }
    }
}
else
{
    action_cmd_.move_action = No_Action;
    action_cmd_.rotate_acton = No_Action;
    if (shoot_flag)
        shoot_count++;
    if (shoot_count > 20)
    {
        shoot_count = 0;
        shoot_flag = false;
    }
}