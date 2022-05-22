const double radius_robot = 50.0, radius_Obs = 50.0;

void relocate(int obs_num, double *dot, double *cros, int *obs_group, std::vector<DPoint> &Obstacles_, DPoint &point_);
//合并Min与Min_num
void _min(int n, double *nums, int &index, double &val)
{
    index = 0;
    val = nums[0];
    for (int i = 1; i < n; ++i)
    {
        if (nums[i] < val)
        {
            index = i;
            val = nums[i];
        }
    }
}
//合并Max与Max_num
void _max(int n, double *nums, int &index, double &val)
{
    index = 0;
    val = nums[0];
    for (int i = 1; i < n; ++i)
    {
        if (nums[i] > val)
        {
            index = i;
            val = nums[i];
        }
    }
}

void subtarget(DPoint target_pos_, DPoint robot_pos_)
{
    std::vector<DPoint> Obstacles_ = world_model_info_.Obstacles_;
    for (int c = Obstacles_.size(); c < 9; c++)
        Obstacles_.push_back(DPoint(10000, 10000));
    double a[9], b[9];
    DPoint point_ = target_pos_ - robot_pos_;
    int i = 0, j = 0, k = 0;
    int B[9] = {0};
    int First_num = 0;
    double minB = 0;
    int G[9] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
    int G_size = 0;
    int G_Obstacles_[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
    for (i = 0; i < 9; i++)
    {
        int temp = 0;
        DPoint point1_ = Obstacles_.at(i) - robot_pos_;
        if (point_.cross(point1_) == 0)
            temp = 0;
        else if (point_.cross(point1_) > 0)
            temp = 1;
        else
            temp = -1;
        a[i] = point_.ddot(point1_) / point_.length();
        b[i] = temp * fabs(point_.cross(point1_)) / point_.length();
    }
    for (i = 0; i < 9; i++)
    {
        if ((a[i] > 0) && (a[i] < point_.length()) && (fabs(b[i]) < (radius_robot + radius_Obs)))
        {
            B[j] = i;
            j++;
        }
    }

    if (j != 0)
    {
        First_num = B[0];
        minB = a[B[0]];
        for (i = 1; i < j; i++)
        {
            if (minB < a[B[i]])
                minB = minB;
            else
            {
                minB = a[B[i]];
                First_num = B[i];
            }
        }
        G[0] = First_num;
        G_size = 0;
        G_Obstacles_[First_num] = 0;

        for (i = 0; i < 9; i++)
        {
            if (G_Obstacles_[i] == 1)
            {
                for (k = 0; k <= G_size; k++)
                {
                    if (Obstacles_.at(i).distance(Obstacles_.at(G[k])) < (2 * radius_robot + 2 * radius_Obs))
                    {
                        G_size++;
                        G[G_size] = i;
                        G_Obstacles_[i] = 0;
                        i = -1;
                        break;
                    }
                }
            }
        }
        //分离出一个函数
        relocate(G_size, a, b, G, Obstacles_, point_);
    }
    else
        subtargets_pos_ = target_pos_;
}

void relocate(int obs_num, double *dot, double *cros, int *obs_group, std::vector<DPoint> &Obstacles_, DPoint &point_)
{
    int canpass(0);
    int bp_num(0), bn_num(0);
    int left = 0, right = 0, sign_side = 0;
    double atemp;
    double b_positive[10] = {0};
    double b_negative[10] = {0};
    double alpha[9];
    int alpha_id;
    double alpha_val;
    for (int i = 0; i <= obs_num; i++)
    {
        if (cros[obs_group[i]] > 0)
            b_positive[bp_num++] = cros[obs_group[i]];
        else if (cros[obs_group[i]] < 0)
            b_negative[bn_num++] = fabs(cros[obs_group[i]]);
    }

    if (*std::max_element(b_positive, b_positive + 10) <= *std::max_element(b_negative, b_negative + 10))
    {
        left = 1;
        sign_side = 1;
    }
    else
    {
        right = 1;
        sign_side = -1;
    }
    for (int i = 0; i <= obs_num; i++)
    {
        atemp = Obstacles_.at(obs_group[i]).distance(robot_pos_);
        if (atemp < 100.0)
        {
            atemp = radius_robot + radius_Obs + 0.0001;
            canpass = 1;
        }
        alpha[i] = atan2(cros[obs_group[i]], dot[obs_group[i]]) + sign_side * asin((radius_robot + radius_Obs) / atemp);
    }
    if (left == 1)
        _max(obs_num + 1, alpha, alpha_id, alpha_val);
    else
        _min(obs_num + 1, alpha, alpha_id, alpha_val);
    subtargets_pos_.x_ = robot_pos_.x_ + (cos(alpha_id) * point_.x_ -
                                          sin(alpha_id) * point_.y_) *
                                             Obstacles_.at(obs_group[alpha_id]).distance(robot_pos_) / point_.length();
    subtargets_pos_.y_ = robot_pos_.y_ +
                         (sin(alpha_val) * point_.x_ + cos(alpha_val) * point_.y_) *
                             Obstacles_.at(obs_group[alpha_id]).distance(robot_pos_) / point_.length();
}