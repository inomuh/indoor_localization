#ifndef LOCALIZATION_NODE
#define LOCALIZATION_NODE

#include <ros/ros.h>
#include <indoor_localization/AnchorScan.h>

typedef struct
{
  int TXcm;
  int TYcm;
  int TZcm;

} pos;

pos calcPos2D3AIte(double SX, double SY, double SZ, double AX, double AY, double AZ, double BX, double BY, double BZ,
                      double TZ, double dist_diff_A_S, double dist_diff_B_S);

#endif
