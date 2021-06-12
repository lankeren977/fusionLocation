#ifndef _KALMANY_H
#define _KALMANY_H


#define   LENGTH      1*1
#define   ORDER       1
#define   N           100
#define   SEED        1567

typedef struct  _tOptimaly
{
  float XNowOpt[LENGTH];
  float XPreOpt[LENGTH];
}tOptimaly;

extern void   KalMan_PramInity(void);
extern float KalMan_Updatey(float *Z);

extern void   KalMan_Parmsety(float x);


#endif
