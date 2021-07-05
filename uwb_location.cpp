#include "uwb_location.h"
#include "load_config.h"

int ANCHOR_NUM;
int ANCHOR_DIS_START;
int MAX_BUFF_SIZE;

vec2d last_result;

void loadUWBParams()
{
    ANCHOR_NUM = getParam("ANCHOR_NUM");
    ANCHOR_DIS_START = getParam("ANCHOR_DIS_START");
    MAX_BUFF_SIZE = getParam("MAX_BUFF_SIZE");
}

//多球交汇原理
vec2d trilateration(const int *radius)
{
    vec2d result;
    int count = 0;
    //初始化圆
    vector<Lcircle> circles;
    map<int, double *>::iterator iter;
    for (int i = 0; i < ANCHOR_NUM; i++)
    {
        iter = globalAnchors.find(i);
        if (iter != globalAnchors.end())
        {
            if (radius[i] > 0 && circles.size() < 4)
            {
                Lcircle circle;
                circle.x = iter->second[0];
                circle.y = iter->second[1];
                circle.r = radius[i];
                circles.push_back(circle);
            }
            else if (radius[i] > 0 && circles.size() >= 4)
            {
                int max_id = 0;
                bool flag = 0;
                for (int k = 0; k < circles.size(); k++)
                {
                    if (circles[k].r > radius[i])
                    {
                        flag = true;
                        if (circles[k].r > circles[max_id].r)
                        {
                            max_id = k;
                        }
                    }
                }
                if (flag)
                {
                    circles[max_id].r = radius[i];
                    circles[max_id].x = iter->second[0];
                    circles[max_id].y = iter->second[1];
                }
            }
        }
    }

    vector<vec2d> alter_points;
    int valid_insect = 0;
    if (count >= 3) //一次三角质心
    {
        for (int i = 0; i < count; i++)
        {
            for (int j = i + 1; j < count; j++)
            {
                vector<vec2d> points = insect(circles[i], circles[j]);
                if (!points.empty() && points.size() == 1)
                {
                    alter_points.push_back(points[0]);
                    valid_insect++;
                }
                else if (!points.empty())
                {
                    for (int k = 0; k < count; k++)
                    {
                        if (k == i || k == j)
                        {
                            continue;
                        }
                        alter_points.push_back(selectPoint(points, circles[k]));
                        valid_insect++;
                    }
                }
            }
        }

        if (alter_points.empty() || valid_insect == 0)
        {
            return last_result;
        }
        else if (valid_insect == 1)
        {
            result = alter_points[0];
        }
        else if (valid_insect == 2)
        {
            result.x = (alter_points[0].x + alter_points[1].x) / 2;
            result.y = (alter_points[0].y + alter_points[1].y) / 2;
        }
        else
        {
            result = optimizeByRatio(alter_points);
        }
        last_result = result;
        return result;
    }
    else
    {
        cout << "基站个数异常" << endl;
        return last_result;
    }
}

//确定备选三角顶点
vec2d selectPoint(const vector<vec2d> points, const Lcircle circle)
{
    vec2d alter_point;
    if (!isOutsideCircle(points[0], circle))
    {
        alter_point = points[0];
    }
    else if (!isOutsideCircle(points[1], circle))
    {
        alter_point = points[1];
    }
    else
    {
        double p1_dis_sqr = pow(points[0].x - circle.x, 2) + pow(points[0].y - circle.y, 2);
        double p2_dis_sqr = pow(points[1].x - circle.x, 2) + pow(points[1].y - circle.y, 2);
        alter_point = (p1_dis_sqr <= p2_dis_sqr) ? points[0] : points[1];
    }
    return alter_point;
}

//判断一点是否在圆外
bool isOutsideCircle(const vec2d point, const Lcircle circle)
{
    double dis_sqr = pow(point.x - circle.x, 2) + pow(point.y - circle.y, 2);
    return (dis_sqr > circle.y * circle.y) ? true : false;
}

//两圆求交点
vector<vec2d> insect(const Lcircle circle1, const Lcircle circle2)
{
    vector<vec2d> results;
    double dis, a, b, c, p, q, r;
    double cos_value[2], sin_value[2];

    //判断是否内含或外离
    dis = sqrt(pow(circle1.x - circle2.x, 2) + pow(circle1.y - circle2.y, 2));
    if (dis < fabs(circle1.r - circle2.r) || dis > circle1.r + circle2.r)
    {
        cout << "两圆没有交点" << endl;
        return results;
    }
    //一个内交点为异常
    if (dis == fabs(circle1.r - circle2.r))
    {
        cout << "两圆有一个内交点" << endl;
        return results;
    }

    vec2d result1, result2;
    //初始化参数
    a = 2 * circle1.r * (circle1.x - circle2.x);
    b = 2 * circle1.r * (circle1.y - circle2.y);
    c = pow(circle2.r, 2) - pow(circle1.r, 2) - pow(circle1.x - circle2.x, 2) - pow(circle1.y - circle2.y, 2);
    p = a * a + b * b;
    q = -2 * a * c;
    r = c * c - b * b;

    //如果只有一个外交点
    if (fabs(dis - (circle1.r + circle2.r)) < ZERO)
    {
        cos_value[0] = -q / p / 2;
        sin_value[0] = sqrt(1 - pow(cos_value[0], 2));

        result1.x = circle1.x + circle1.r * cos_value[0];
        result1.y = circle1.y + circle1.r * sin_value[0];
        //验证sin正负
        if (pow(result1.x - circle2.x, 2) + pow(result1.y - circle2.y, 2) - pow(circle2.r, 2) > ZERO)
        {
            result1.y = circle1.y - circle1.r * sin_value[0];
        }
        results.push_back(result1);
        return results;
    }

    //如果有两个交点
    cos_value[0] = (sqrt(q * q - 4 * p * r) - q) / p / 2;
    cos_value[1] = (-sqrt(q * q - 4 * p * r) - q) / p / 2;
    sin_value[0] = sqrt(1 - pow(cos_value[0], 2));
    sin_value[1] = sqrt(1 - pow(cos_value[1], 2));

    result1.x = circle1.x + circle1.r * cos_value[0];
    result2.x = circle1.x + circle1.r * cos_value[1];
    result1.y = circle1.y + circle1.r * sin_value[0];
    result2.y = circle1.y + circle1.r * sin_value[1];
    //验证解
    if (pow(result1.x - circle2.x, 2) + pow(result1.y - circle2.y, 2) - pow(circle2.r, 2) > ZERO)
    {
        result1.y = circle1.y - circle1.r * sin_value[0];
    }
    if (pow(result2.x - circle2.x, 2) + pow(result2.y - circle2.y, 2) - pow(circle2.r, 2) > ZERO)
    {
        result2.y = circle1.y - circle1.r * sin_value[1];
    }
    //一种特殊情况当已经确定有两个不同的交点，但解出来的cos值只有一个，则sin值必定一正一负
    if (fabs(result1.x - result2.x) < ZERO && fabs(result1.y - result2.y) < ZERO)
    {
        if (result1.y > 0)
        {
            result2.y = -result2.y;
        }
        else
        {
            result1.y = -result1.y;
        }
    }
    results.push_back(result1);
    results.push_back(result2);
    return results;
}

//三角质心优化---改为误差加权平均优化
vec2d optimizeByRatio(const vector<vec2d> points)
{
    vec2d result;
    int length = points.size();
    double avg_x, avg_y, sum_e_x, sum_e_y, sum_r_x, sum_r_y;
    avg_x = avg_y = sum_e_x = sum_e_y = sum_r_x = sum_r_y = 0;
    for (int i = 0; i < length; i++)
    {
        avg_x += points[i].x;
        avg_y += points[i].y;
    }
    avg_x /= points.size();
    avg_y /= points.size();

    //计算与平均值的差值
    double error_x[length] = {0};
    double error_y[length] = {0};
    double ratio_x[length] = {0};
    double ratio_y[length] = {0};
    for (int i = 0; i < length; i++)
    {
        error_x[i] = fabs(points[i].x - avg_x);
        error_y[i] = fabs(points[i].y - avg_y);
        sum_e_x += error_x[i];
        sum_e_y += error_y[i];
    }

    for (int i = 0; i < length; i++)
    {
        if (ZERO == error_x[i])
        {
            error_x[i] = 0.001;
        }
        if (ZERO == error_x[i])
        {
            error_x[i] = 0.001;
        }
        ratio_x[i] = sum_e_x / error_x[i];
        ratio_y[i] = sum_e_y / error_y[i];
        sum_r_x += ratio_x[i];
        sum_r_y += ratio_y[i];
    }

    for (int i = 0; i < length; i++)
    {
        result.x += points[i].x * ratio_x[i] / sum_r_x;
        result.y += points[i].y * ratio_y[i] / sum_r_y;
    }

    return result;
}