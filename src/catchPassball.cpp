void Plan::CatchPassedBall()
{
	if (oppDribble() >= 0)
	{
		world_model_->pass_state_.reset();
		return;
	}

	DPoint r2b = ball_pos_ - robot_pos_;

	action->rotate_acton = CatchBall;
	action->move_action = No_Action;
	action->rotate_mode = 0;
	m_behaviour_.move2orif(r2b.angle().radian_, robot_ori_.radian_, 0.087);


	DPoint headoffPoint;
	//垂直运动到球路径
	if (ball_vel_.length())
	{
		action->move_action = Positioned;
		double k1 = ball_vel_.y_ / ball_vel_.x_;
		double b1 = ball_pos_.y_ - k1 * ball_pos_.x_;
		double k2 = -1.0 / k1;
		double b2 = robot_pos_.y_ - k2 * robot_pos_.x_;
		headoffPoint = DPoint((b2 - b1) / (k1 - k2), (k1 * b2 - k2 * b1) / (k1 - k2));
	}

	if (m_behaviour_.move2target(headoffPoint, robot_pos_))
	{
		//接球
		if (robot_pos_.distance(ball_pos_) <= 50.0 && ball_is_free())
		{
			action->handle_enable = 1;
			action->move_action = CatchBall;
			action->rotate_acton = CatchBall;
			action->rotate_mode = 0;
			if (m_behaviour_.move2orif(r2b.angle().radian_, robot_ori_.radian_))
				m_behaviour_.move2target(ball_pos_, robot_pos_);
		}
	}
}