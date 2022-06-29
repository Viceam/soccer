//对方几号带球, -1表示对方无人持球
int whichopp_dribble()
{
    for (int i = 0; i < 5; ++i)
        if (world_model_info_.Opponents_[i].distance(ball_pos_) < 36)
            return i;
    return -1;
}

//球是否自由运动
bool ball_is_free()
{
    for (int i = 0; i < 5; ++i)
    {
        if (world_model_info_.RobotInfo_[i].getDribbleState())
            return false;
    }
    if (whichopp_dribble() >= 0)
        return false;
    return true;
}

bool isNearestRobot() //判断自己是否为离球最近的机器人
{
    float distance_min = 2000.0;
    float distance = distance_min;
    int robot_id = -1;

    for (int i = 1; i < 5; i++) // 排除守门员
        if (world_model_info_.RobotInfo_[i].isValid())
        {
            distance = ball_pos_.distance(world_model_info_.RobotInfo_[i].getLocation());
            if (distance < distance_min)
            {
                distance_min = distance;
                robot_id = i;
            }
        }
    if (robot_id + 1 == world_model_info_.AgentID_)
        return true;
    else
        return false;
}

//对方机器人的位置
std::queue<DPoint> opp_location[5];
//对方机器人到球的向量
std::queue<DPoint> opp2b[5];
//球速
std::queue<DPoint> vels;

//下一位置
DPoint opp_next_pos;
//下一方向
DPoint opp2b_next;

//更新最近6次球速
void update_vels()
{
    vels.push(ball_vel_);
    while (vels.size() > 6)
        vels.pop();
}
//更新最近10次向量
void update_opp2b()
{
    for (int i = 0; i < 5; i++)
    {
        opp2b[i].push(ball_pos_ - world_model_info_.Opponents_[i]);
        if (opp2b[i].size() > 10)
            opp2b[i].pop();
    }
}
//更新最近6次位置
void update_opp_location()
{
    for (int i = 0; i < 5; i++)
    {
        opp_location[i].push(world_model_info_.Opponents_[i]);
        if (opp_location[i].size() > 6)
            opp_location[i].pop();
    }
}

int ballState()
{
    if (vels.size() != 5)
        return 0;
    if ((vels.front() - vels.back()).length())
        return 1; // RUN
    return -1;    // FLY
}

//根据进6个周期对方机器人的位置获取对方速度
DPoint get_opp_vel(int i)
{
    if (opp_location[i].size() < 6)
        return DPoint(0.0, 0.0);
    double vx(0.0), vy(0.0);
    auto qlocation = opp_location[i];
    int t = 0;
    DPoint opp_pos[6], s;
    while (!qlocation.empty())
    {
        opp_pos[t++] = qlocation.front();
        qlocation.pop();
    }
    for (int i = 0; i < 3; ++i)
    {
        vx += (opp_pos[i + 3].x_ - opp_pos[i].x_) / 3;
        vy += (opp_pos[i + 3].y_ - opp_pos[i].y_) / 3;
    }
    return DPoint(vx / 0.09, vy / 0.09);
}

//获取对方角速度
double get_opp_ori(int i)
{
    if (opp2b[i].size() != 10)
        return 0.0;
    auto a2b = opp2b[i];
    int t = 0;
    double radian[10], delta_radian(0.0);
    while (!a2b.empty())
    {
        radian[t++] = a2b.front().angle().radian_;
        a2b.pop();
    }
    for (int k = 0; k < 5; k++)
    {
        delta_radian += (radian[k + 5] - radian[k]) / 5;
    }
    delta_radian /= 5;
    return delta_radian / 0.03;
}

//计算next
void predict_next(int i)
{
    DPoint opp_pos_now = world_model_info_.Opponents_[i];
    DPoint opp_vel = get_opp_vel(i);
    double a2b_radian = (ball_pos_ - opp_pos_now).angle().radian_;
    double a2b_ori = get_opp_ori(i);
    double a2b_radian_next;

    //   update  //
    opp_next_pos = opp_pos_now + 0.3 * opp_vel;
    a2b_radian_next = a2b_radian + 0.3 * a2b_ori;
    opp2b_next = DPoint(cos(a2b_radian_next), sin(a2b_radian_next));
}
