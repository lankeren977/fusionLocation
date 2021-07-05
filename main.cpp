#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <iomanip>
#include "open_dev.h"
#include "uwb_location.h"
#include "load_config.h"
#include "visual_location.h"
#include "kalman.h"
#include "kalmany.h"

using namespace std;
using namespace cv;


pthread_t ntid_u;
pthread_t ntid_v;
pthread_t ntid_f;
float zx[LENGTH] = {0};
float zy[LENGTH] = {0};
int dev;
unsigned char* buffer;
VideoCapture cap(0);
VisualData visual_result;
vec2d uwb_result;
vec2d fusion_result;

bool openSerial()
{
    //打开串口
    dev = OpenDev();
    if (dev > 0)
    {
        set_speed(dev, SPEED);
    }
    else
    {
        perror("Can't Open Serial Port");
        return 0;
    }
    if (set_Parity(dev, 8, 1, 'N') == -1)
    {
        printf("Set Parity Error\n");
        return 0;
    }
    tcflush(dev, TCIFLUSH);
    return 1;
}

bool openCamera()
{
    //打开摄像头
    cap.set(3, 640);
    cap.set(4, 480);
    if (!cap.isOpened())
    {
        cout << "摄像头未成功打开" << endl;
        return 0;
    }
    return 1;
}

void *getUWBData(void *data)
{
    ssize_t nread;
    buffer = new unsigned char[MAX_BUFF_SIZE];
    while (1)
    {
        nread = read(dev, buffer, MAX_BUFF_SIZE);
        if (nread != 0 && 'm' == buffer[0])
        {
            int offset = 0;
            int *radius = new int[ANCHOR_NUM];
            switch (buffer[1])
            {
            case 'r': //原始测距数据
                for (int i = 0; i < ANCHOR_NUM; i++)
                {
                    unsigned int dis = buffer[ANCHOR_DIS_START + offset + 1] << 8;
                    dis = dis ^ buffer[ANCHOR_DIS_START + offset];
                    radius[i] = dis;
                    offset = offset + 2;
                }
                uwb_result = trilateration(radius);
                break;
            case 'c': //校正后测距数据
                break;
            case 'a': //基站间测距数据
                break;
            default:
                break;
            }
            delete radius;
            delete ids;
        }
    }
    delete buffer;
}

void *getVisulData(void *data)
{
    Mat srcImage;
    while (1)
    {
        cap >> srcImage;
        if (!srcImage.empty())
        {
            visual_result = getVisualLocalizeData(srcImage);
            namedWindow("show", WINDOW_NORMAL);
            imshow("show", srcImage);
            waitKey(20);
        }
    }
}

void *getFusionDataX(void *data)
{
    while (1)
    {
        KalMan_Parmset(visual_result.visual_x);
        zx[0] = uwb_result.x;
        fusion_result.x = KalMan_Update(zx);
    }
}

void *getFusionDataY(void *data)
{
    while (1)
    {
        KalMan_Parmsety(visual_result.visual_y);
        zy[0] = uwb_result.y;
        fusion_result.y = KalMan_Updatey(zy);
    }
}


int main()
{
    ofstream outFile;
    outFile.open("./output/data.csv", ios::out); // 打开模式可省略

    //打开
    if (!openSerial() || !openCamera())
    {
        exit(1);
    }
    // 加载配置文件
    loadConfig();

    //取配置参数
    loadVisualParams();
    loadUWBParams();

    clock_t startTime, endTime;

    int err1, err2, err3, err4;
    err1 = pthread_create(&ntid_v, NULL, getVisulData, NULL);
    err2 = pthread_create(&ntid_u, NULL, getUWBData, NULL);
    err3 = pthread_create(&ntid_f, NULL, getFusionDataX, NULL);
    err4 = pthread_create(&ntid_f, NULL, getFusionDataY, NULL);
    if (err1 != 0)
    {
        cout << "visual thread creates fail" << endl;
    }
    if (err2 != 0)
    {
        cout << "uwb thread creates fail" << endl;
    }
    if (err3 != 0)
    {
        cout << "fux thread creates fail" << endl;
    }
    if (err4 != 0)
    {
        cout << "fuy thread creates fail" << endl;
    }

    while (1)
    {
        cout << setiosflags(ios::fixed) << setprecision(2) << visual_result.visual_x << "," << visual_result.visual_y << "," << uwb_result.x << "," << uwb_result.y << "," << fusion_result.x << "," << fusion_result.y << endl;
        outFile << setiosflags(ios::fixed) << setprecision(2) << visual_result.visual_x << "," << visual_result.visual_y << "," << uwb_result.x << "," << uwb_result.y << "," << fusion_result.x << "," << fusion_result.y << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    pthread_cancel(ntid_v);
    pthread_cancel(ntid_u);
    pthread_cancel(ntid_f);
    //关闭串口
    close(dev);
    //释放摄像头
    cap.release();

    return 0;
}

/*
int main()
{
    KalMan_PramInit();
    ifstream fin("/home/linux/Desktop/data/data10.csv"); //打开文件流操作
    string line;
    float z[1];
    ofstream outFile;
    outFile.open("/home/linux/Desktop/data/new_data11.csv", ios::out); // 打开模式可省略
    while (getline(fin, line))                                  //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
    {
        istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
        vector<string> fields;   //声明一个字符串向量
        string field;
        while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
        {
            fields.push_back(field); //将刚刚读取的字符串添加到向量fields中
        }
        string v_x = fields[0]; //清除掉向量fields中第一个元素的无效字符，并赋值给变量name
        string v_y = fields[1];
        string u_x = fields[2]; //清除掉向量fields中第一个元素的无效字符，并赋值给变量name
        string u_y = fields[3];

        float new_v_x = atof(v_x.c_str());
        float new_v_y = atof(v_y.c_str());
        float new_u_x = atof(u_x.c_str());
        float new_u_y = atof(u_y.c_str());

        KalMan_Parmset(new_v_y);
        z[0] = new_u_y;
        outFile << setiosflags(ios::fixed) << setprecision(2) << KalMan_Update(z) << endl;

        // z[0] = new_v_y;
        // new_v_y = KalMan_Update(z);
        // z[0] = new_u_x;
        // new_u_x = KalMan_Update(z);
        // z[0] = new_u_y;
        // new_u_y = KalMan_Update(z);
        // outFile << setiosflags(ios::fixed) << setprecision(2) << new_v_x << "," << new_v_y << "," << new_u_x << "," << new_u_y << endl;
    }
    return 0;
}*/