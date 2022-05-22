
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
    if (num == 6)
        return DPoint(-1050, 0);

    //角平分线站位
    if (num == 3)
        return bisector();

    if (num == 2)
    {
        //球向球门移动，垂直运动到球的路径上
        if (ball_v.x_ < 0)
        {
            double k1 = ball_v.y_ / ball_v.x_;
            double b1 = ball_pos.y_ - k1 * ball_pos.x_;
            double k2 = -1.0 / k1;
            double b2 = rb_pos.y_ - k2 * rb_pos.x_;
            return DPoint((b2 - b1) / (k1 - k2), (k1 * b2 - k2 * b1) / (k1 - k2));
        }
        //否则采用角平分线站位//
        else
            return this->bisector();
    }

    // num == 1 的情况
    return DPoint(-1050, 0);
}

//运行函数
void goal_keeper()
{
    DPoint target = gk.position();

    DPoint t2r = target - robot_pos_;
    action_cmd_.move_action = Positioned;
    action_cmd_.rotate_acton = Positioned;
    action_cmd_.rotate_mode = 0;
    // if (mv2orif_e1(t2r.angle().radian_, robot_ori_.radian_, 12.0 * DEG2RAD))
    move2target(target, robot_pos_);

    //到达目标点随球转向
    bool flag = 0;
    if (target.distance(robot_pos_) <= 20.0)
    {
        flag = 1;
        auto b2r = ball_pos_ - robot_pos_;
        mv2orif_e1(b2r.angle().radian_, robot_ori_.radian_);
    }

    if (flag && !world_model_info_.RobotInfo_[1].getDribbleState() && ball_pos_.distance(robot_pos_) <= 300.0)
    {
        action_cmd_.move_action = CatchBall;
        action_cmd_.rotate_acton = CatchBall;
        action_cmd_.rotate_mode = 0;
        move2target(ball_pos_, robot_pos_);
    }
}
