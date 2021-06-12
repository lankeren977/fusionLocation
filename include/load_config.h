#ifndef LOAD_CONFIG_H
#define LOAD_CONFIG_H

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <map>
using namespace std;

extern  map<int,double*> globalLandmarks;
extern  map<string,double> globalParams;

static string landmarks_path = "/home/linux/zhangqr/fusionLocalize/config/landmarks.conf";
static string visual_params_path = "/home/linux/zhangqr/fusionLocalize/config/visual_params.conf";
static string uwb_params_path = "/home/linux/zhangqr/fusionLocalize/config/uwb_params.conf";

double getParam(string key);
void loadConfig();

#endif