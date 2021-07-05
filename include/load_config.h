#ifndef LOAD_CONFIG_H
#define LOAD_CONFIG_H

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <map>
using namespace std;

extern  map<int,double*> globalLandmarks;
extern  map<int,double*> globalAnchors;
extern  map<string,double> globalParams;

static string landmarks_path = "/home/linux/zhangqr/fusionLocalize/config/landmarks.conf";
static string anchors_path = "/home/linux/zhangqr/fusionLocalize/config/anchors.conf";
static string sys_params_path = "/home/linux/zhangqr/fusionLocalize/config/sys_params.conf";

double getParam(string key);
void loadConfig();

#endif
