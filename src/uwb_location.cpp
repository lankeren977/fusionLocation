#include "uwb_location.h"
#include "load_config.h"

int ANCHOR_NUM;
int ANCHOR_DIS_START;
int MAX_BUFF_SIZE;
int *LAST_RANGES;
int *ANCHOR_STATE;
double THREADHOLD;
vector<vec2d> ANCHORS;
bool INIT_FLAG;


vec2d last_result;

void loadUWBParams()
{
    INIT_FLAG = true;
    ANCHOR_NUM = getParam("ANCHOR_NUM");
    ANCHOR_DIS_START = getParam("ANCHOR_DIS_START");
    MAX_BUFF_SIZE = getParam("MAX_BUFF_SIZE");
    THREADHOLD = getParam("THREADHOLD");
    LAST_RANGES = new int[ANCHOR_NUM];
    ANCHOR_STATE = new int[ANCHOR_NUM];
    //加载基站坐标
    map<int, double *>::iterator iter;
    for(int i = 0; i < ANCHOR_NUM; i++){
        iter = globalAnchors.find(i);
        if (iter != globalAnchors.end()){
            vec2d anchor;
            anchor.x = iter->second[0];
            anchor.y = iter->second[1];
            ANCHORS.push_back(anchor);
        }
    }
}

//多球交汇原理
vec2d trilateration(const int *radius)
{
    vec2d result;
    vector<Lcircle> circles;
    map<int, double *>::iterator iter;
    double *nlos_ranges = new double[ANCHOR_NUM];
    int index = 0;
    //比较测距信息，判断NLOS状态
    if(INIT_FLAG){
        for(int i = 0;i < ANCHOR_NUM; i++){
            if(radius[i] > 0){
                LAST_RANGES[i] = radius[i];
                Lcircle circle;
                circle.x = ANCHORS[i].x;
                circle.y = ANCHORS[i].y;
                circle.r = radius[i];
                circles.push_back(circle);
            }
        }
        INIT_FLAG = false;
    }else{
        for(int i = 0;i < ANCHOR_NUM; i++){
            if(radius[i] > 0){
                double delta = radius[i] - LAST_RANGES[i];
                LAST_RANGES[i] = radius[i];
                if(0 == ANCHOR_STATE[i] && delta > THREADHOLD){
                    ANCHOR_STATE[i] = 1;
                }else if(0 == ANCHOR_STATE[i]){
                    Lcircle circle;
                    circle.x = ANCHORS[i].x;
                    circle.y = ANCHORS[i].y;
                    circle.r = radius[i];
                    circles.push_back(circle);
                }else if(1 == ANCHOR_STATE[i] && delta < 0 && fabs(delta) > THREADHOLD){
                    ANCHOR_STATE[i] = 0;
                    Lcircle circle;
                    circle.x = ANCHORS[i].x;
                    circle.y = ANCHORS[i].y;
                    circle.r = radius[i];
                    circles.push_back(circle);
                }else{
                    nlos_ranges[index++] = radius[i];
                }
            }
        }
        //补位测距最短的NLOS基站
        int lack_num = 3 - circles.size();
        if (lack_num > 0){
             //冒泡排序
            for(int i = 0; i < index ;i++){
                for(int j = 0;j < index - i; j++){
                    if(nlos_ranges[j] > nlos_ranges[j+1]){
                        double temp = nlos_ranges[j];
                        nlos_ranges[j] = nlos_ranges[j+1];
                        nlos_ranges[j+1] = temp;
                    }
                }
            }
            for(int i = 0; i < lack_num ; i++){
                for(int j = 0;j < ANCHOR_NUM; j++){
                    if(nlos_ranges[i] == radius[j] && ANCHOR_STATE[j] == 1){
                        Lcircle circle;
                        circle.x = ANCHORS[j].x;
                        circle.y = ANCHORS[j].y;
                        circle.r = radius[j];
                        circles.push_back(circle);
                        break;
                    }
                }
            }
        }
        delete nlos_ranges;
    }
    
    int count = circles.size();

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
                        break;
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
    //求一个在第三个圆边界附近的点
    double p0_dis = sqrt(pow(points[0].x - circle.x, 2) + pow(points[0].y - circle.y, 2));
    double p1_dis = sqrt(pow(points[1].x - circle.x, 2) + pow(points[1].y - circle.y, 2));
    return (fabs(p0_dis - circle.r) <= (fabs(p1_dis - circle.r))) ? points[0] : points[1];
}

//两圆求交点
vector<vec2d> insect(const Lcircle circle1, const Lcircle circle2)
{
    vector<vec2d> results;
    double x1 = circle1.x;
    double y1 = circle1.y;
    double r1 = circle1.r;
    double x2 = circle2.x;
    double y2 = circle2.y;
    double r2 = circle2.r;
    //判断是否内含或外离
    double dis = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    if (dis < fabs(r1 - r2) || dis > r1 + r2)
    {
        cout << "两圆没有交点" << endl;
        r1 += 5;
        r2 += 5;
        return results;
    }
    //一个内交点为异常
    if (dis == fabs(r1 - r2))
    {
        cout << "两圆有一个内交点" << endl;
        return results;
    }

    vec2d result1, result2;
    double a, b, c;
    double delta = -1;
    if (y1 != y2)
    {
        double A = (x1 * x1 - x2 * x2 + y1 * y1 - y2 * y2 + r2 * r2 - r1 * r1) / (2 * (y1 - y2));
        double B = (x1 - x2) / (y1 - y2);
        a = 1 + B * B;
        b = -2 * (x1 + (A - y1) * B);
        c = x1 * x1 + (A - y1) * (A - y1) - r1 * r1;

        delta = b * b - 4 * a * c;
        if (delta > 0)
        {
            result1.x = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
            result2.x = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
            result1.y = A - B * result1.x;
            result2.y = A - B * result2.x;
            results.push_back(result1);
            results.push_back(result2);
            return results;
        }
        else if (delta == 0)
        {
            result1.x = -b / (2 * a);
            result1.y = A - B * result1.x;
            results.push_back(result1);
            return results;
        }
        else
        {
            cout << "无解1" << endl;
            return results;
        }
    }
    else if (x1 != x2)
    {
        result1.x = result2.x = (x1 * x1 - x2 * x2 + r2 * r2 - r1 * r1) / (2 * (x1 - x2));
        a = 1;
        b = -2 * y1;
        c = y1 * y1 - r1 * r1 + (x1 - result1.x) * (x1 - result1.x);
        delta = b * b - 4 * a * c;
        if (delta > 0)
        {
            result1.y = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
            result2.y = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
            results.push_back(result1);
            results.push_back(result2);
            return results;
        }
        else if (delta == 0)
        {
            result1.y = -b / (2 * a);
            results.push_back(result1);
            return results;
        }
        else
        {
            cout << "无解2" << endl;
            return results;
        }
    }
    else
    {
        cout << "无解3" << endl;
        return results;
    }
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