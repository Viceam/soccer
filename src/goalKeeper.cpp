
//守门员类

class goalKeeper
{
public:
    goalKeeper() : world_(), rb_pos(), rb_ori(), ball_pos() {}
    //重载() 初始化
    void operator()(const World_Model_Info &w, const DPoint &rp, const Angle &ro, const DPoint &bp, const DPoint &bv);
    DPoint position();

private:
    World_Model_Info world_;
    DPoint rb_pos;
    Angle rb_ori;
    DPoint ball_pos;
    DPoint ball_v;
    int getArea(); //分区策略
    DPoint bisector();
};

void NuBotControl::goalKeeper::operator()(const World_Model_Info &w, const DPoint &rp, const Angle &ro, const DPoint &bp, const DPoint &bv)
{
    world_ = w;
    rb_pos = rp;
    rb_ori = ro;
    ball_pos = bp;
    ball_v = bv;
}
int NuBotControl::goalKeeper::getArea()
{
    DPoint shoot_robot = world_.RobotInfo_[1].getLocation();
    // bool _b = world_.field_info_.isOurGoal(ball_pos);
    double k = ball_pos.y_ / (ball_pos.x_ + 1100.0);
    if (k > 2.5)
        return 4;
    if (k < -2.5)
        return 5;
    DPoint mid(-1100, 0);

    double s = ball_pos.distance(mid);
    if (s < 209)
        return 1;
    if (s < 412)
        return 2;
    if (s < 750)
        return 3;
    return 6;
}

//平分角，平分线斜率
inline double divk(double k1, double k2)
{
    return ((k1 * k2 - 1 + sqrt((1 - k1 * k2) * (1 - k1 * k2) + (k1 + k2) * (k1 + k2))) / (k1 + k2));
}

//角平分线站位
DPoint NuBotControl::goalKeeper::bisector()
{
    //球与门柱上下连线斜率
    auto up_k = (ball_pos.y_ - 120) / (ball_pos.x_ + 1100.0);
    auto down_k = (ball_pos.y_ + 120) / (ball_pos.x_ + 1100.0);

    //角平分线斜率
    auto k = divk(up_k, down_k);
    //确保是内角平分线
    k = ((ball_pos.y_ > 0) ? fabs(k) : -fabs(k));
    return DPoint(-1050.0, -k * 1050 + ball_pos.y_ - k * ball_pos.x_);
}

//获取目标点
DPoint NuBotControl::goalKeeper::position()
{

    auto num = getArea();
        if (num == 4)
            return DPoint(-1050, 120);
        if (num == 5)
            return DPoint(-1050, -120);

        //角平分线站位
        if (num == 3)
            return bisector();

        // num == 1,2,6 的情况
        return DPoint(-1050, 0);
}

//运行函数
void goal_keeper()
{
    //根据球位置获取站位
    DPoint target = gk.position();
    DPoint t2r = target - robot_pos_;
    auto b2r = ball_pos_ - robot_pos_;

    if (ball_is_free() && robot_pos_.distance(ball_pos_) <= 55)
    {
        action_cmd_.handle_enable = 1;
        action_cmd_.move_action = CatchBall;
        action_cmd_.rotate_acton = CatchBall;
        action_cmd_.rotate_mode = 0;
        if (m_plan_.m_behaviour_.move2orif(b2r.angle().radian_, robot_ori_.radian_))
            m_plan_.m_behaviour_.move2target(ball_pos_, robot_pos_);
    }

    if (!ball_is_free() && !world_model_info_.RobotInfo_[0].getDribbleState())
    {
        if (move2target(target, robot_pos_))
        {
            move2ori(b2r.angle().radian_, robot_ori_.radian_);
        }
    }

    else if (ball_is_free())
    {
        if (ball_vel_.x_ < 0)
        {
            //运动到入网点
            double k1 = ball_vel_.y_ / ball_vel_.x_;
            double b1 = ball_pos_.y_ - k1 * ball_pos_.x_;
            double target_y = k1 * (-1050.0) + b1;
            if (fabs(target_y) < 120)
            {
                action_cmd_.handle_enable = 1;
                action_cmd_.move_action = CatchBall;
                action_cmd_.rotate_acton = CatchBall;
                action_cmd_.rotate_mode = 0;
                m_plan_.m_behaviour_.move2target(DPoint(-1050.0, target_y), robot_pos_);
                m_plan_.m_behaviour_.move2orif(b2r.angle().radian_, robot_ori_.radian_);
            }
            // double k2 = -1.0 / k1;
            // double b2 = robot_pos_.y_ - k2 * robot_pos_.x_;
            // DPoint((b2 - b1) / (k1 - k2), (k1 * b2 - k2 * b1) / (k1 - k2));
        }
    }

    if (world_model_info_.RobotInfo_[0].getDribbleState())
    {

        // ROS_INFO("gk pre pass");

        int pass__mode = RUN;
        auto pass_vec_ = world_model_info_.RobotInfo_[2].getLocation() - robot_pos_;

        if (pass_vec_.length() > 650)
            pass__mode = FLY;

        shoot_flag = m_plan_.PassBall_Action(3, pass__mode);
    }
}