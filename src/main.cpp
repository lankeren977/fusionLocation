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

using namespace std;
using namespace cv;


pthread_t ntid_u;
pthread_t ntid_v;
pthread_t ntid_f;
int dev;
unsigned char* buffer;
int *radius;
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
    radius = new int[ANCHOR_NUM];
    while (1)
    {
        nread = read(dev, buffer, MAX_BUFF_SIZE);
        if (nread != 0 && 'm' == buffer[0])
        {
            int offset = 0;
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
        }
    }
    delete radius;
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
            //namedWindow("show", WINDOW_NORMAL);
            //imshow("show", srcImage);
            //waitKey(20);
        }
    }
}

void *getFusionData(void *data)
{
    while (1)
    {
       
    }
}


int main()
{
    ofstream outFile;
    outFile.open("../output/data.csv", ios::out); // 打开模式可省略

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

    int err1, err2, err3;
    err1 = pthread_create(&ntid_v, NULL, getVisulData, NULL);
    err2 = pthread_create(&ntid_u, NULL, getUWBData, NULL);
    //err3 = pthread_create(&ntid_f, NULL, getFusionData, NULL);
    if (err1 != 0)
    {
        cout << "visual thread creates fail" << endl;
    }
    if (err2 != 0)
    {
        cout << "uwb thread creates fail" << endl;
    }
    /*if (err3 != 0)
    {
        cout << "fusion thread creates fail" << endl;
    }*/

    while (1)
    {
        cout << setiosflags(ios::fixed) << setprecision(2) <<  visual_result.visual_theta << "," << visual_result.visual_x << "," << visual_result.visual_y << "," << uwb_result.x << "," << uwb_result.y <<","<< radius[0] <<","<< radius [1] <<","<< radius[2] <<","<< radius[3] << endl;
        outFile << setiosflags(ios::fixed) << setprecision(2) << visual_result.visual_theta << "," <<visual_result.visual_x << "," << visual_result.visual_y << "," << uwb_result.x << "," << uwb_result.y <<","<< radius[0] <<","<< radius [1] <<","<< radius[2] <<","<< radius[3]<<  endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    pthread_cancel(ntid_v);
    pthread_cancel(ntid_u);
    //pthread_cancel(ntid_f);
    //关闭串口
    close(dev);
    //释放摄像头
    cap.release();

    return 0;
}
